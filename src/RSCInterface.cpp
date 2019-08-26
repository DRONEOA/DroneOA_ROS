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

#include <droneoa_ros/RSCInterface.hpp>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

static const char* OPENCV_WINDOW = "Debug window";
cv::Point RSCInterface::debugMousePos = cv::Point(0, 0);

RSCInterface::RSCInterface() {
    cv::namedWindow(OPENCV_WINDOW);
}

RSCInterface::~RSCInterface() {
    cv::destroyWindow(OPENCV_WINDOW);
    delete thread_watch_depth_img_;
}

void RSCInterface::init(ros::NodeHandle nh, ros::Rate r) {
    n = nh;
    r_ = r;
    cv::startWindowThread();  // DEBUG
    cv::setMouseCallback(OPENCV_WINDOW, RSCInterface::mouseCallback, NULL);  // DEBUG
    thread_watch_depth_img_ = new boost::thread(boost::bind(&RSCInterface::watchDepthImgThread, this));
    ROS_INFO("[RSC] init");
}

/* Callback */
void RSCInterface::depthImg_callback(const sensor_msgs::ImageConstPtr& msg) {
    depthImage_ = *msg;

    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(depthImage_, sensor_msgs::image_encodings::TYPE_32FC1);
    } catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    depthFrame_ = cv_ptr->image;
    drawDebugOverlay();
}

/* Threads */
void RSCInterface::watchDepthImgThread() {
    auto node = boost::make_shared<ros::NodeHandle>();  // @TODO: can we remove this ?
    auto relative_pos_sub =
        node->subscribe<sensor_msgs::Image>("/camera/depth/image_rect_raw", 1,
                boost::bind(&RSCInterface::depthImg_callback, this, _1));

    while (ros::ok()) {
        ros::spinOnce();
        r_.sleep();
    }
}

/* Debug */
void RSCInterface::printImgInfo() {
    ROS_INFO("[IMG] height: %d width: %d", depthImage_.height, depthImage_.width);
    ROS_INFO("[IMG] encoding: %s", depthImage_.encoding.c_str());
}

void drawText(cv::Mat targetImg, cv::Point origin, std::string text, double font_scale = 1, int thickness = 1) {
    int font_face = cv::FONT_HERSHEY_COMPLEX;
    int baseline;
    cv::Size text_size = cv::getTextSize(text, font_face, font_scale, thickness, &baseline);
    cv::Point originTop;
    originTop.x = origin.x;
    originTop.y = origin.y - text_size.height;
    cv::Point pt2;
    pt2.x = originTop.x + text_size.width;
    pt2.y = origin.y;
    cv::rectangle(targetImg, originTop, pt2, cv::Scalar(0), -1, CV_AA);
    cv::putText(targetImg, text, origin, font_face, font_scale, cv::Scalar(0xffff), thickness, 8, 0);
}

// Warning: do not use the ouput as depth data
cv::Mat getBetterImageDebug(cv::Mat input) {
    double min = 0;
    double max = 12000;
    cv::minMaxIdx(input, &min, &max);
    cv::Mat adjMap;
    cv::convertScaleAbs(input, adjMap, 255 / max);
    return adjMap;
}

void RSCInterface::drawDebugOverlay() {
    if (cv::countNonZero(depthFrame_) < 1) {
        ROS_WARN("Empty Image Ignored");
        return;
    }
    float centerDist = depthFrame_.at<float>(debugMousePos.y, debugMousePos.x);  // NOte: row, col order
    std::string centerDistStr = std::to_string(centerDist) + " mm";

    cv::Mat debugImage255 = getBetterImageDebug(depthFrame_);
    drawText(debugImage255, cv::Point(20, 20), "Debug Overlay:", 0.5, 1);
    drawText(debugImage255, cv::Point(20, 40), centerDistStr, 0.5, 1);
    cv::line(debugImage255, cv::Point(debugMousePos.x - 7, debugMousePos.y - 7),
            cv::Point(debugMousePos.x + 7, debugMousePos.y + 7), cv::Scalar(0xffff), 2);
    cv::line(debugImage255, cv::Point(debugMousePos.x - 7, debugMousePos.y + 7),
            cv::Point(debugMousePos.x + 7, debugMousePos.y - 7), cv::Scalar(0xffff), 2);
    cv::imshow(OPENCV_WINDOW, debugImage255);
}

void RSCInterface::mouseCallback(int event, int x, int y, int flags, void* userdata) {
    if (event == cv::EVENT_LBUTTONDOWN) {
        // std::cout << "Left button of the mouse is clicked - position (" << x << ", " << y << ")" << std::endl;
    } else if (event == cv::EVENT_RBUTTONDOWN) {
        // std::cout << "Right button of the mouse is clicked - position (" << x << ", " << y << ")" << std::endl;
    } else if (event == cv::EVENT_MBUTTONDOWN) {
        // std::cout << "Middle button of the mouse is clicked - position (" << x << ", " << y << ")" << std::endl;
    } else if (event == cv::EVENT_MOUSEMOVE) {
        debugMousePos.x = x;
        debugMousePos.y = y;
    }
}
