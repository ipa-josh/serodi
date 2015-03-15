#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <vector>
#include <std_msgs/Bool.h>
#include <serodi_mapping/map2cv.h>
#include <cob_srvs/Trigger.h>
#include <cob_srvs/GetPoseStampedTransformed.h>

#define UTRUE 255
#define UFALSE 0
bool check_closure(int x, int y, std::vector<bool> &occ, const cv::Mat &costmap, const cv::Mat &unk_map, int &aim_x, int &aim_y) {
	if(x<0 || y<0 || x>=costmap.rows || y>=costmap.cols)
		return false;
		
	if(occ[x*costmap.cols+y] || costmap.at<uint8_t>(x,y)==UFALSE)
		return true;
	occ[x*costmap.cols+y] = true;

	if(aim_x<0 && unk_map.at<uint8_t>(x,y)==UFALSE) {
		aim_x=x;
		aim_y=y;
	}
	
	const int ar_x[4] = {x-1,x+1,x,x};
	const int ar_y[4] = {y,y,y-1,y+1};
	const int roff = rand()%4;	//select random order
	
	if(	!check_closure(ar_x[(roff+0)%4],ar_y[(roff+0)%4],occ,costmap, unk_map,aim_x,aim_y) ||
		!check_closure(ar_x[(roff+1)%4],ar_y[(roff+1)%4],occ,costmap, unk_map,aim_x,aim_y) ||
		!check_closure(ar_x[(roff+2)%4],ar_y[(roff+2)%4],occ,costmap, unk_map,aim_x,aim_y) ||
		!check_closure(ar_x[(roff+3)%4],ar_y[(roff+3)%4],occ,costmap, unk_map,aim_x,aim_y)) {
		
		if(aim_x<0) {
			aim_x=x;
			aim_y=y;
		}
		
		return false;
	}
	
	return true;
}

bool check_closure(int x, int y, const cv::Mat &costmap, const cv::Mat &unk_map, int &aim_x, int &aim_y) {
  aim_x = aim_y = -1;
  std::vector<bool> occ(costmap.rows*costmap.cols, false);
  return check_closure(x,y, occ, costmap, unk_map, aim_x, aim_y);
}

bool getRobotPose(tf::Stamped<tf::Pose>& global_pose, tf::TransformListener &tf_, const std::string &robot_base_frame_, const std::string &global_frame_)
{

  global_pose.setIdentity();
  tf::Stamped < tf::Pose > robot_pose;
  robot_pose.setIdentity();
  robot_pose.frame_id_ = robot_base_frame_;
  robot_pose.stamp_ = ros::Time();
  ros::Time current_time = ros::Time::now(); // save time for checking tf delay later

  //get the global pose of the robot
  try
  {
    tf_.transformPose(global_frame_, robot_pose, global_pose);
  }
  catch (tf::LookupException& ex)
  {
    ROS_ERROR_THROTTLE(1.0, "No Transform available Error looking up robot pose: %s\n", ex.what());
    return false;
  }
  catch (tf::ConnectivityException& ex)
  {
    ROS_ERROR_THROTTLE(1.0, "Connectivity Error looking up robot pose: %s\n", ex.what());
    return false;
  }
  catch (tf::ExtrapolationException& ex)
  {
    ROS_ERROR_THROTTLE(1.0, "Extrapolation Error looking up robot pose: %s\n", ex.what());
    return false;
  }
  // check global_pose timeout
  double transform_tolerance_ = 0.5;
  if (current_time.toSec() - global_pose.stamp_.toSec() > transform_tolerance_)
  {
    ROS_WARN_THROTTLE(1.0,
                      "Costmap2DROS transform timeout. Current time: %.4f, global_pose stamp: %.4f, tolerance: %.4f",
                      current_time.toSec(), global_pose.stamp_.toSec(), transform_tolerance_);
    return false;
  }

  return true;
}

bool check_free_line(const int x1, const int y1, const int x2, const int y2, const cv::Mat &costmap) {
	// grabs pixels along the line (pt1, pt2)
	// from 8-bit 3-channel image to the buffer
	cv::LineIterator it(costmap, cv::Point(x1,y1), cv::Point(x2,y2), 8);
	cv::LineIterator it2 = it;
	std::vector<uint8_t> buf(it.count);

	for(int i = 0; i < it.count; i++, ++it)
		if( *(const uint8_t*)*it == UFALSE )
			return false;
			
	return true;
}

bool vis_graph(const int x, const int y, const double last_alpha, const cv::Mat &costmap, const double radius, const double limit_alpha, const int limit_depth, std::vector<geometry_msgs::Pose> &poses)
{
	geometry_msgs::Pose pose;
	pose.position.x = x;
	pose.position.y = y;
	pose.position.z = 0;
	pose.orientation = tf::createQuaternionMsgFromYaw(last_alpha);
	
	poses.push_back(pose);
	if(poses.size()>limit_depth)
		return true;
	
	for(int i=0; i<10; i++) {
	
		const float r = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
		double new_alpha = r*(2*(M_PI-limit_alpha)) + last_alpha;
		while(new_alpha>2*M_PI) new_alpha-=2*M_PI;
		
		const int new_x = std::cos(new_alpha)*radius+x;
		const int new_y = std::sin(new_alpha)*radius+y;
		
		if(check_free_line(x,y, new_x, new_y, costmap))
			return vis_graph(new_x, new_y, new_alpha, costmap, radius, limit_alpha, limit_depth, poses);
	}
	
	return false;
}

class MapNode {
	ros::NodeHandle node_;
	Map2cv map_recv_;
	
	//params
	double radius_;
	std::string robot_base_frame_, global_frame_, map_dir_;
	
	//ros
	ros::Publisher pub_map_update_;
	ros::ServiceServer srv_check_, srv_free_;	
	tf::TransformListener tf_;
	
	void on_map_update() {
		//save
		cv::imwrite(map_dir_+"/map_occ.png", map_recv_.getOccMap());
		cv::imwrite(map_dir_+"/map_unk.png", map_recv_.getUnkMap());
		
		pub_map_update_.publish(map_recv_.getMapInfo());
	}
	
	bool srv_check_closure(cob_srvs::GetPoseStampedTransformed::Request  &req, cob_srvs::GetPoseStampedTransformed::Response &res)
	{
		res.success = false;
		
		tf::Stamped < tf::Pose > pose;
		if (!getRobotPose (pose, tf_, robot_base_frame_, global_frame_)) {
			ROS_ERROR("could not get robot pose from %s to %s", robot_base_frame_.c_str(), global_frame_.c_str());
			return true;
		}
		
		const double resolution = map_recv_.getMapInfo().resolution;
		int x,y;
		const int cur_x = boost::math::round( (pose.getOrigin().x()-map_recv_.getMapInfo().origin.position.x)/resolution);
		const int cur_y = boost::math::round( map_recv_.getMapInfo().height-1-(pose.getOrigin().y()-map_recv_.getMapInfo().origin.position.y)/resolution);
		res.success =
		check_closure(	cur_x, cur_y,
						map_recv_*radius_*map_recv_.getOccMap(), map_recv_.getUnkMap(), x,y);
						
		res.result.pose.position.x = x*resolution+map_recv_.getMapInfo().origin.position.x;
		res.result.pose.position.y = (map_recv_.getMapInfo().height-1-y)*resolution+map_recv_.getMapInfo().origin.position.y;
		res.result.pose.position.z = 0;
		
		res.result.pose.orientation = tf::createQuaternionMsgFromYaw( std::atan2(y-cur_y, x-cur_x) );
		
		return true;
	}
	
	bool srv_get_pose_free(cob_srvs::GetPoseStampedTransformed::Request  &req, cob_srvs::GetPoseStampedTransformed::Response &res)
	{
		res.success = false;
		
		tf::Stamped < tf::Pose > pose;
		if (!getRobotPose (pose, tf_, robot_base_frame_, global_frame_)) {
			ROS_ERROR("could not get robot pose from %s to %s", robot_base_frame_.c_str(), global_frame_.c_str());
			return true;
		}
		
		const double resolution = map_recv_.getMapInfo().resolution;
		std::vector<geometry_msgs::Pose> poses;
		
		res.success = vis_graph(boost::math::round( (pose.getOrigin().x()-map_recv_.getMapInfo().origin.position.x)/resolution),
								boost::math::round( map_recv_.getMapInfo().height-1-(pose.getOrigin().y()-map_recv_.getMapInfo().origin.position.y)/resolution),
					tf::getYaw(pose.getRotation()), map_recv_*radius_*map_recv_.getOccMap(), (int)(radius_/resolution+0.99), 0.8, 1, poses);
					
		if(poses.size()<1) return false;
		res.result.pose = poses[0];
		res.result.pose.position.x = res.result.pose.position.x*resolution+map_recv_.getMapInfo().origin.position.x;
		res.result.pose.position.y = (map_recv_.getMapInfo().height-1-res.result.pose.position.y)*resolution+map_recv_.getMapInfo().origin.position.y;
		
		return true;
	}
	
public:

	MapNode(): tf_(ros::Duration(10)) {
		map_recv_.getSignal().connect(boost::bind(&MapNode::on_map_update, this));
		
		pub_map_update_ = node_.advertise<nav_msgs::MapMetaData>("map_update", 10);
		srv_check_ = node_.advertiseService<cob_srvs::GetPoseStampedTransformed::Request, cob_srvs::GetPoseStampedTransformed::Response>
			("check_closure", boost::bind(&MapNode::srv_check_closure, this, _1, _2));
		srv_free_ = node_.advertiseService<cob_srvs::GetPoseStampedTransformed::Request, cob_srvs::GetPoseStampedTransformed::Response>
			("get_pose_free", boost::bind(&MapNode::srv_get_pose_free, this, _1, _2));
			
		ros::NodeHandle pn("~");
		
		pn.param<double>("radius", radius_, 0.5);
		
		pn.param<std::string>("robot_base_frame", robot_base_frame_, std::string("/base_link"));
		pn.param<std::string>("global_frame", global_frame_, std::string("/map"));
		pn.param<std::string>("map_dir", map_dir_, std::string("/tmp"));
		
		ROS_INFO("params %s %s %s", robot_base_frame_.c_str(), global_frame_.c_str(), map_dir_.c_str());
	}
};

#include <boost/program_options.hpp>
int main(int argc, char **argv) {
	ros::init(argc, argv, "check_map_closure");
	
  using namespace boost::program_options;
  
  std::string gtest_file;
  options_description cmd_line("Options");
  cmd_line.add_options()
    ("gtest", value<std::string>(&gtest_file),
     "run gtest")
    ;
  variables_map vm;
  store(command_line_parser(argc, argv).options(cmd_line).run(), vm);
  notify(vm);

  /*if (vm.count("gtest"))
  {
		Map2cv map_recv;
	  bool r;
	  r = map_recv.loadFromFile(gtest_file);
	  ROS_INFO("result=%d",(int)r);
	  
	  r = check_closure(100,100, map_recv.getOccMap());
	  ROS_INFO("result=%d",(int)r);
	  
	  r = check_closure(100,100, map_recv*1.f*map_recv.getOccMap());
	  ROS_INFO("result=%d",(int)r);
	  
	  std::vector<geometry_msgs::Pose> poses;
	  r = vis_graph(100, 100, 0, map_recv*1.f*map_recv.getOccMap(), 10., 0.8, 5, poses);
	  ROS_INFO("result=%d",(int)r);
	  
	  for(size_t i=0; i<poses.size(); i++) ROS_INFO("pose %f %f", poses[i].position.x, poses[i].position.y);
	  
	  return 0;
  }*/
  
  MapNode node;
	
	ros::spin();
	/*
	ros::Rate loop_rate(5);
	while(ros::ok()) {
		ROS_INFO("y");
		tf::Stamped < tf::Pose > pose;
		if (!getRobotPose (pose, tf,robot_base_frame, global_frame)) continue;
		
		layers.updateMap(pose.getOrigin().x(), pose.getOrigin().y(), tf::getYaw(pose.getRotation()));
		ROS_INFO("x");
		
		printMap(*costmap);
		
		int x,y; //TODO:
		std_msgs::Bool msg;
		std::vector<bool> occ(costmap->getSizeInCellsX()*costmap->getSizeInCellsY(), false);
		msg.data = check_closure(x,y,occ,*costmap);
		
		pub.publish(msg);
		
		ros::spinOnce();
		loop_rate.sleep();
	}*/
	
	return 0;
}
