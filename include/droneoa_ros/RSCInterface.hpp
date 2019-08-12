/* Copyright (C) DroneOA Group - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 * Written by Bohan Shi <b34shi@edu.uwaterloo.ca>, August 2019
 */

#ifndef INCLUDE_DRONEOA_ROS_RSCINTERFACE_HPP_  // NOLINT
#define INCLUDE_DRONEOA_ROS_RSCINTERFACE_HPP_  // NOLINT

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <opencv2/core/core.hpp>

#include <boost/bind.hpp>
#include <boost/thread/thread.hpp>

class RSCInterface {
 public:
    RSCInterface();
    virtual ~RSCInterface();
    void init(ros::NodeHandle nh, ros::Rate r);

    // Getter

    // Callback
    void depthImg_callback(const sensor_msgs::ImageConstPtr& msg);

    // Debug Print
    void printImgInfo();
    static void mouseCallback(int event, int x, int y, int flags, void* userdata);

 private:
    ros::NodeHandle n;
    ros::Rate r_ = ros::Rate(10.0);

    // Data
    sensor_msgs::Image depthImage_;
    cv::Mat depthFrame_;

    // Threads
    boost::thread* thread_watch_depth_img_ = nullptr;
    void watchDepthImgThread();

    // Debug
    void drawDebugOverlay();
    static cv::Point debugMousePos;
};

#endif  // NOLINT