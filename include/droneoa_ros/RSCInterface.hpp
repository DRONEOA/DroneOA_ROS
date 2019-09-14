/* Copyright (C) 2019 DroneOA Group - All Rights Reserved
 * This file is part of DroneOA_ROS.
 *
 * DroneOA_ROS is free software: you can redistribute it and/or 
 * modify it under the terms of the GNU Affero General Public License
 * as published by the Free Software Foundation.
 *
 * DroneOA_ROS is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Affero General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public
 * License along with DroneOA_ROS. 
 * If not, see <https://www.gnu.org/licenses/>.
 *
 * Written by Bohan Shi <b34shi@edu.uwaterloo.ca>, August 2019
 */

#ifndef INCLUDE_DRONEOA_ROS_RSCINTERFACE_HPP_  // NOLINT
#define INCLUDE_DRONEOA_ROS_RSCINTERFACE_HPP_  // NOLINT

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <opencv2/core/core.hpp>

#include <boost/bind.hpp>
#include <boost/thread/thread.hpp>

#define ENABLE_POINTCLOUD

class RSCInterface {
 public:
    RSCInterface();
    virtual ~RSCInterface();
    void init(ros::NodeHandle nh, ros::Rate r);

    cv::Mat depthImgForDesiredDistanceRange(float min, float max, cv::Mat input);
    void setRangeSwitch(bool status);
    void setRange(float min, float max);

    // Callback
    void depthImg_callback(const sensor_msgs::ImageConstPtr& msg);
    void pointcloud_callback(const sensor_msgs::PointCloud2ConstPtr& msg);

    // Debug Print
    void printImgInfo();
    static void mouseCallback(int event, int x, int y, int flags, void* userdata);

 private:
    ros::NodeHandle n;
    ros::Rate r_ = ros::Rate(10.0);
    bool rangeSwitch = false;

    // Data
    sensor_msgs::Image depthImage_;
    sensor_msgs::PointCloud2 pointCloud_;
    cv::Mat depthFrame_;
    float rangeMin;
    float rangeMax;

    // Threads
    boost::thread* thread_watch_depth_img_ = nullptr;
    boost::thread* thread_watch_pointcloud_ = nullptr;
    void watchDepthImgThread();
    void watchPointCloudThread();

    // Debug
    void drawDebugOverlay();
    static cv::Point debugMousePos;
};

#endif  // NOLINT