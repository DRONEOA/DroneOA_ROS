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
#include <pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/radius_outlier_removal.h>

#include <droneoa_ros/Utils.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

static const char* OPENCV_WINDOW = "Debug window";
cv::Point RSCInterface::debugMousePos = cv::Point(0, 0);

RSCInterface::RSCInterface() {
    cv::namedWindow(OPENCV_WINDOW);
#ifdef PCL_DEBUG_VIEWER
    viewer = new pcl::visualization::CloudViewer("Depth Cloud Viewer");
#endif
}

RSCInterface::~RSCInterface() {
    cv::destroyWindow(OPENCV_WINDOW);
    if (thread_watch_depth_img_) {
        delete thread_watch_depth_img_;
    }
    if (thread_watch_pointcloud_) {
        delete thread_watch_pointcloud_;
    }
#ifdef PCL_DEBUG_VIEWER
    if (viewer) {
        delete viewer;
    }
#endif
}

void RSCInterface::init(ros::NodeHandle nh, ros::Rate r) {
    n = nh;
    r_ = r;
#ifdef IMG_DEBUG_POPUP
    cv::startWindowThread();  // DEBUG
    cv::setMouseCallback(OPENCV_WINDOW, RSCInterface::mouseCallback, NULL);  // DEBUG
#endif
    thread_watch_depth_img_ = new boost::thread(boost::bind(&RSCInterface::watchDepthImgThread, this));
#ifdef ENABLE_POINTCLOUD
    thread_watch_pointcloud_ = new boost::thread(boost::bind(&RSCInterface::watchPointCloudThread, this));
#endif
    ROS_INFO("[RSC] init");
}

/* Callback */
void RSCInterface::depthImg_callback(const sensor_msgs::ImageConstPtr& msg) {
    boost::unique_lock<boost::shared_mutex> uniqueLock(depth_img_mutex);
    depthImage_ = *msg;

    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(depthImage_, sensor_msgs::image_encodings::TYPE_32FC1);
    } catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    depthFrame_ = cv_ptr->image;
#ifdef UE4_SITL
    for (int i = 0; i < depthFrame_.rows; i++) {
        float* Mi = depthFrame_.ptr<float>(i);
        for (int j = 0; j < depthFrame_.cols; j++) {
            Mi[j] *= UE4_SITL_SCALE;
        }
    }
#endif
#ifdef IMG_DEBUG_POPUP
    drawDebugOverlay();
#endif
}

void RSCInterface::pointcloud_callback(const sensor_msgs::PointCloud2ConstPtr& msg) {
    boost::unique_lock<boost::shared_mutex> uniqueLock(pointcloud_mutex);
    pointCloud_ = *msg;
    // Convert to pointcloud
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(pointCloud_, pcl_pc2);
    pcl::fromPCLPointCloud2(pcl_pc2, pcl_pointCloud_);
#ifdef UE4_SITL
    pcl::PointCloud<pcl::PointXYZRGB> temp;
    Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();
    transform_2.scale(UE4_SITL_SCALE);
    pcl::transformPointCloud(pcl_pointCloud_, temp, transform_2);
    pcl_pointCloud_ = temp;
#endif
    // @TODO need to change coordinates if needed
    /***
     * Note: 
     * X axis goes horizontaly, with right to be the positive axis.
     * Y axis goes vertically, with up to be the positive axis.
     * The coordinate has the unit Meter.
     */
}

/* Threads */
void RSCInterface::watchDepthImgThread() {
    auto node = boost::make_shared<ros::NodeHandle>();  // @TODO: can we remove this ?
#if defined(UE4_SITL)
    auto relative_pos_sub =
        node->subscribe<sensor_msgs::Image>("/unreal_ros/image_depth", 1,
                boost::bind(&RSCInterface::depthImg_callback, this, _1));
#else
    auto relative_pos_sub =
        node->subscribe<sensor_msgs::Image>("/d435/depth/image_rect_raw", 1,
                boost::bind(&RSCInterface::depthImg_callback, this, _1));
#endif

    while (ros::ok()) {
        ros::spinOnce();
        r_.sleep();
    }
}

void RSCInterface::watchPointCloudThread() {
    auto node = boost::make_shared<ros::NodeHandle>();  // @TODO: can we remove this ?
#if defined(UE4_SITL)
    auto relative_pos_sub =
        node->subscribe<sensor_msgs::PointCloud2>("/depth_registered/points", 1,
                boost::bind(&RSCInterface::pointcloud_callback, this, _1));
#else
    auto relative_pos_sub =
        node->subscribe<sensor_msgs::PointCloud2>("/d435/depth/color/points", 1,
                boost::bind(&RSCInterface::pointcloud_callback, this, _1));
#endif

    while (ros::ok()) {
#ifdef PCL_DEBUG_VIEWER
        if (!thread_pointcloud_viewer_) {
            thread_pointcloud_viewer_ =
                new boost::thread(boost::bind(&RSCInterface::updatePointCloudViewerThread, this));
        }
#endif
        ros::spinOnce();
        r_.sleep();
    }
}

#ifdef PCL_DEBUG_VIEWER
void RSCInterface::updatePointCloudViewerThread() {
    boost::shared_lock<boost::shared_mutex> lock(pointcloud_mutex);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(&pcl_pointCloud_);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
    // PCL Viewer
    while (!viewer->wasStopped()) {
        viewer->showCloud(cloud);
        boost::this_thread::sleep_for(boost::chrono::milliseconds(100));
    }
}
#endif

/* Debug */
void RSCInterface::printImgInfo() {
    ROS_INFO("[IMG] height: %d width: %d", depthImage_.height, depthImage_.width);
    ROS_INFO("[IMG] encoding: %s", depthImage_.encoding.c_str());
#ifdef ENABLE_POINTCLOUD
    ROS_INFO("[Pointcloud] frameID: %s", pointCloud_.header.frame_id.c_str());
    ROS_INFO("[Pointcloud] field size: %zd", pointCloud_.fields.size());
    for (unsigned int i = 0; i < pointCloud_.fields.size(); ++i) {
        ROS_INFO("      field: %s", pointCloud_.fields[i].name.c_str());
    }
    ROS_INFO("[Pointcloud] pcl data size: %zd", pcl_pointCloud_.size());
#endif
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

    float centerDist = depthFrame_.at<float>(debugMousePos.y, debugMousePos.x);  // Note: row, col order
    std::string centerDistStr = std::to_string(centerDist) + " mm";

    cv::Mat debugImage255;
    if (rangeSwitch) {
        cv::Mat rangedDebugImage = depthImgForDesiredDistanceRange(rangeMin, rangeMax, depthFrame_);
        debugImage255 = getBetterImageDebug(rangedDebugImage);
    } else {
        debugImage255 = getBetterImageDebug(depthFrame_);
    }

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

/*****************************************************
 * Range Filter
 */

cv::Mat RSCInterface::depthImgForDesiredDistanceRange(float min, float max, cv::Mat input) {
    cv::Mat adjMap = input.clone();
    for (int y=0; y < adjMap.rows; y++) {
        for (int x=0; x < adjMap.cols; x++) {
            if (adjMap.at<float>(cv::Point(x, y)) < min || adjMap.at<float>(cv::Point(x, y)) > max) {
                adjMap.at<float>(cv::Point(x, y)) = 0;
            }
        }
    }
    return adjMap;
}

void RSCInterface::setRangeSwitch(bool status) {
    rangeSwitch = status;
}

void RSCInterface::setRange(float min, float max) {
    rangeMin = min;
    rangeMax = max;
}

/*****************************************************
 * Hit Pencentage
 */

std::vector<float> RSCInterface::pointCloudZCoordsInRange(float width, float height, float dist) {
    boost::shared_lock<boost::shared_mutex> lock(pointcloud_mutex);
    unsigned int pointCount = 0;
    float x = width/2;
    float y = width/2;
    if (dist < 200.0f) {
        dist = 200.0f;
    }
    std::vector<float> zCoords;
    for ( auto i = 0; i < pcl_pointCloud_.points.size(); i++ ) {
        pcl::PointXYZRGB pt = pcl_pointCloud_.points.at(i);
        if ( inRange<float>(-x, x, pt.x*1000) && inRange<float>(-y, y, pt.y*1000) ) {
            pointCount++;
            zCoords.push_back(pt.z*1000);
        }
    }
    return zCoords;
}
