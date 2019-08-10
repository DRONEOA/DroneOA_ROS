/* Copyright (C) DroneOA Group - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 * Written by Bohan Shi <b34shi@edu.uwaterloo.ca>, August 2019
 */

#include <droneoa_ros/RSCInterface.hpp>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

static const char* OPENCV_WINDOW = "Debug window";

RSCINterface::RSCINterface() {
    cv::namedWindow(OPENCV_WINDOW);
}

RSCINterface::~RSCINterface() {
    cv::destroyWindow(OPENCV_WINDOW);
    delete thread_watch_depth_img_;
}

void RSCINterface::init(ros::NodeHandle nh, ros::Rate r) {
    n = nh;
    r_ = r;
    thread_watch_depth_img_ = new boost::thread(boost::bind(&RSCINterface::watchDepthImgThread, this));
    cv::startWindowThread();
}

/* Callback */
void RSCINterface::depthImg_callback(const sensor_msgs::ImageConstPtr& msg) {
    depthImage_ = *msg;

    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(depthImage_, depthImage_.encoding);
    } catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
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

/* Debug */
void RSCINterface::printImgInfo() {
    ROS_INFO("[IMG] height: %d width: %d", depthImage_.height, depthImage_.width);
    ROS_INFO("[IMG] encoding: %s", depthImage_.encoding.c_str());
}
