#pragma once

#include "ros/ros.h"
#include "ros/console.h"
#include <nav_msgs/GetMap.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <boost/signals.hpp>

struct Erode {
	const int ir_;
	Erode(const int ir) : ir_(ir) {
	}
	
	cv::Mat operator*(const cv::Mat &map) {
		cv::Mat kernel = cv::Mat::ones(cv::Size(ir_,ir_), CV_8U);
		cv::Mat img_binary_erode = map.clone();

		// do the erosion
		erode(map, img_binary_erode, kernel);
		
		return img_binary_erode;
	}
};


class Map2cv
{
	nav_msgs::MapMetaData map_info_;
	cv::Mat map_occ_, map_unk_;
    ros::Subscriber map_sub_;
    boost::signal<void ()> sig_map_update_;

  public:
    Map2cv();
    
    const cv::Mat &getOccMap() const {return map_occ_;}
    const cv::Mat &getUnkMap() const {return map_unk_;}
    
    const nav_msgs::MapMetaData &getMapInfo() const {return map_info_;}
    
    Erode operator*(const float r) const {return Erode((int)(r/map_info_.resolution+0.5));}
    
    bool loadFromFile(const std::string &fn);
    
    boost::signal<void ()> &getSignal() {return sig_map_update_;}

private:
    void mapCallback(const nav_msgs::OccupancyGridConstPtr& map);

};
