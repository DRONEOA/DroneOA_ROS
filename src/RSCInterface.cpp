/* Copyright (C) DroneOA Group - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 * Written by Bohan Shi <b34shi@edu.uwaterloo.ca>, August 2019
 */

#include <droneoa_ros/RSCInterface.hpp>

RSCINterface::RSCINterface() {}

RSCINterface::~RSCINterface() {
    delete thread_watch_depth_img_;
}

void RSCINterface::init(ros::NodeHandle nh, ros::Rate r) {
    n = nh;
    r_ = r;
    thread_watch_depth_img_ = new boost::thread(boost::bind(&RSCINterface::watchDepthImgThread, this));
}

/* Callback */
void RSCINterface::depthImg_callback(const sensor_msgs::ImageConstPtr& msg) {
    ROS_INFO("NEW IMAGE");
    depthImage_ = *msg;
}

/* Threads */
void RSCINterface::watchDepthImgThread() {
    auto node = boost::make_shared<ros::NodeHandle>();  // @TODO: can we remove this ?
    auto relative_pos_sub =
        node->subscribe<sensor_msgs::Image>("/camera/depth/image_rect_raw", 1,
                boost::bind(&RSCINterface::depthImg_callback, this, _1));

    while (ros::ok()) {
        ros::spinOnce();
        r_.sleep();
    }
}
