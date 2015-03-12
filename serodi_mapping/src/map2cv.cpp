/*
 * map_saver
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the <ORGANIZATION> nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <cstdio>
#include "tf/LinearMath/Matrix3x3.h"
#include "geometry_msgs/Quaternion.h"
#include <serodi_mapping/map2cv.h>

Map2cv::Map2cv()
{
  ros::NodeHandle n;
  map_sub_ = n.subscribe("map", 1, &Map2cv::mapCallback, this);
}

bool Map2cv::loadFromFile(const std::string &fn) {
	cv::Mat img = cv::imread(fn, CV_8U);
	map_info_.width = img.rows;
    map_info_.height = img.cols;
    map_info_.resolution = 0.05;
    
  ROS_INFO("Loaded a %d X %d map @ %.3f m/pix",
		   map_info_.width,
		   map_info_.height,
		   map_info_.resolution);
	
  map_occ_ = cv::Mat(img.size(), CV_8U, 255);
  map_unk_ = cv::Mat(img.size(), CV_8U, 255);
  for(unsigned int y = 0; y < img.cols; y++) {
	for(unsigned int x = 0; x < img.rows; x++) {
	  if (img.at<uint8_t>(x,y) == 0) { //occ [0,0.1)
		map_occ_.at<uint8_t>(x,y) = 0;
	  } else if (img.at<uint8_t>(x,y) < 230) { //occ (0.65,1]
		map_unk_.at<uint8_t>(x,y) = 0;
	  } else { //occ [0.1,0.65]
	  }
	}
  }
  
	cv::imwrite("/tmp/map_occ_.png",map_occ_);
	cv::imwrite("/tmp/map_unk_.png",map_unk_);
  
  return img.rows*img.cols>0;
}

void Map2cv::mapCallback(const nav_msgs::OccupancyGridConstPtr& map)
{
  ROS_INFO("Received a %d X %d map @ %.3f m/pix",
		   map->info.width,
		   map->info.height,
		   map->info.resolution);
		   
  map_info_ = map->info;
		   
  map_occ_ = cv::Mat(map->info.height, map->info.width, CV_8U, 255);
  map_unk_ = cv::Mat(map->info.height, map->info.width, CV_8U, 255);
  for(unsigned int y = 0; y < map->info.height; y++) {
	for(unsigned int x = 0; x < map->info.width; x++) {
	  unsigned int i = x + (map->info.height - y - 1) * map->info.width;
	  if (map->data[i] == 0) { //occ [0,0.1)
	  } else if (map->data[i] == +100) { //occ (0.65,1]
		map_occ_.at<uint8_t>(y,x) = 0;
	  } else { //occ [0.1,0.65]
		map_unk_.at<uint8_t>(y,x) = 0;
	  }
	}
  }
  
  sig_map_update_();
}
	
