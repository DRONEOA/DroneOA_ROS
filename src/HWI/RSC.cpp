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

#include <cv_bridge/cv_bridge.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/radius_outlier_removal.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <droneoa_ros/HWI/RSC.hpp>
#include <droneoa_ros/Utils/GeneralUtils.hpp>

namespace Depth {

static const char* OPENCV_WINDOW = "Debug window";
cv::Point RSC::mDebugMousePos = cv::Point(0, 0);

RSC::RSC(ros::NodeHandle node, ros::Rate rate) : mNodeHandle(node), mRate(rate) {
    cv::namedWindow(OPENCV_WINDOW);
#ifdef PCL_DEBUG_VIEWER
    viewer = new pcl::visualization::CloudViewer("Depth Cloud Viewer");
#endif
}

RSC::~RSC() {
    try {
        cv::destroyWindow(OPENCV_WINDOW);
    } catch(...) {
        ROS_INFO("cv::destroyWindow warn");
    }
    if (mpThreadWatchDepthImg) delete mpThreadWatchDepthImg;
    if (mpThreadWatchPointcloud) delete mpThreadWatchPointcloud;
#ifdef PCL_DEBUG_VIEWER
    if (viewer) {
        delete viewer;
    }
#endif
    ROS_INFO("Destroy RSCInterface");
}

void RSC::initWatcherThread() {
    mCurrentDepthSource = DEPTH_SOURCE_RSC;
    mCurrentPCSource = PC_SOURCE_RSC;
#ifdef IMG_DEBUG_POPUP
    cv::startWindowThread();  // DEBUG
    cv::setMouseCallback(OPENCV_WINDOW, RSC::mouseCallback, NULL);  // DEBUG
#endif
    mpThreadWatchDepthImg = new boost::thread(boost::bind(&RSC::watchDepthImgThread, this));
#ifdef ENABLE_POINTCLOUD
    mpThreadWatchPointcloud = new boost::thread(boost::bind(&RSC::watchPointCloudThread, this));
#endif
    ROS_INFO("[RSC] init");
}

void RSC::changeDepthSource(std::string depthSource) {
    mCurrentDepthSource = depthSource;
    mDepthSub.shutdown();
    mDepthSub =
        mNodeHandle.subscribe<sensor_msgs::Image>(mCurrentDepthSource, 1,
                boost::bind(&RSC::depthImg_callback, this, _1));
}

void RSC::changePC2Source(std::string pc2Source) {
    mCurrentPCSource = pc2Source;
    mPC2Sub.shutdown();
    mPC2Sub =
        mNodeHandle.subscribe<sensor_msgs::PointCloud2>(mCurrentPCSource, 1,
                boost::bind(&RSC::pointcloud_callback, this, _1));
}

/* Callback */
void RSC::depthImg_callback(const sensor_msgs::ImageConstPtr& msg) {
    boost::unique_lock<boost::shared_mutex> uniqueLock(mDepthImgMutex);
    mDepthImage = *msg;

    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(mDepthImage, sensor_msgs::image_encodings::TYPE_32FC1);
    } catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    mDepthFrame = cv_ptr->image;

    // Scale Depth Data For UE4
    if (mCurrentDepthSource == DEPTH_SOURCE_UE4) {
        for (int32_t i = 0; i < mDepthFrame.rows; i++) {
            float* Mi = mDepthFrame.ptr<float>(i);
            for (int32_t j = 0; j < mDepthFrame.cols; j++) {
                Mi[j] *= UE4_SITL_SCALE;
            }
        }
    }

#ifdef IMG_DEBUG_POPUP
    drawDebugOverlay();
#endif
}

void RSC::pointcloud_callback(const sensor_msgs::PointCloud2ConstPtr& msg) {
    // boost::unique_lock<boost::shared_mutex> uniqueLock(mPointcloudMutex);
    mPointCloud = *msg;
    // Convert to pointcloud
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(mPointCloud, pcl_pc2);
    pcl::fromPCLPointCloud2(pcl_pc2, mPclPointCloud);

    // @TODO need to change coordinates if needed
    /***
     * Note: 
     * X axis goes horizontaly, with right to be the positive axis.
     * Y axis goes vertically, with up to be the positive axis.
     * The coordinate has the unit Meter.
     */
}

/* Threads */
void RSC::watchDepthImgThread() {
    mDepthSub =
        mNodeHandle.subscribe<sensor_msgs::Image>(mCurrentDepthSource, 1,
                boost::bind(&RSC::depthImg_callback, this, _1));

    while (ros::ok()) {
        ros::spinOnce();
        mRate.sleep();
    }
}

void RSC::watchPointCloudThread() {
    mPC2Sub =
        mNodeHandle.subscribe<sensor_msgs::PointCloud2>(mCurrentPCSource, 1,
                boost::bind(&RSC::pointcloud_callback, this, _1));

    while (ros::ok()) {
#ifdef PCL_DEBUG_VIEWER
        if (!thread_pointcloud_viewer_) {
            thread_pointcloud_viewer_ =
                new boost::thread(boost::bind(&RSC::updatePointCloudViewerThread, this));
        }
#endif
        ros::spinOnce();
        mRate.sleep();
    }
}

#ifdef PCL_DEBUG_VIEWER
void RSC::updatePointCloudViewerThread() {
    // boost::shared_lock<boost::shared_mutex> lock(mPointcloudMutex);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(&mPclPointCloud);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
    // PCL Viewer
    while (!viewer->wasStopped()) {
        viewer->showCloud(cloud);
        boost::this_thread::sleep_for(boost::chrono::milliseconds(100));
    }
}
#endif

/* Debug */
void RSC::printImgInfo() {
    ROS_INFO("[IMG] height: %d width: %d", mDepthImage.height, mDepthImage.width);
    ROS_INFO("[IMG] encoding: %s", mDepthImage.encoding.c_str());
#ifdef ENABLE_POINTCLOUD
    ROS_INFO("[Pointcloud] frameID: %s", mPointCloud.header.frame_id.c_str());
    ROS_INFO("[Pointcloud] field size: %zd", mPointCloud.fields.size());
    for (uint32_t i = 0; i < mPointCloud.fields.size(); ++i) {
        ROS_INFO("      field: %s", mPointCloud.fields[i].name.c_str());
    }
    ROS_INFO("[Pointcloud] pcl data size: %zd", mPclPointCloud.size());
#endif
}

void drawText(cv::Mat targetImg, cv::Point origin, std::string text, double font_scale = 1, int32_t thickness = 1) {
    int32_t font_face = cv::FONT_HERSHEY_COMPLEX;
    int32_t baseline;
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

void RSC::drawDebugOverlay() {
    if (cv::countNonZero(mDepthFrame) < 1) {
        ROS_WARN("Empty Image Ignored");
        return;
    }

    float centerDist = mDepthFrame.at<float>(mDebugMousePos.y, mDebugMousePos.x);  // Note: row, col order
    std::string centerDistStr = std::to_string(centerDist) + " mm";

    cv::Mat debugImage255;
    if (mRangeSwitch) {
        cv::Mat rangedDebugImage = depthImgForDesiredDistanceRange(mRangeMin, mRangeMax, mDepthFrame);
        debugImage255 = getBetterImageDebug(rangedDebugImage);
    } else {
        debugImage255 = getBetterImageDebug(mDepthFrame);
    }

    drawText(debugImage255, cv::Point(20, 20), "Debug Overlay:", 0.5, 1);
    drawText(debugImage255, cv::Point(20, 40), centerDistStr, 0.5, 1);
    cv::line(debugImage255, cv::Point(mDebugMousePos.x - 7, mDebugMousePos.y - 7),
            cv::Point(mDebugMousePos.x + 7, mDebugMousePos.y + 7), cv::Scalar(0xffff), 2);
    cv::line(debugImage255, cv::Point(mDebugMousePos.x - 7, mDebugMousePos.y + 7),
            cv::Point(mDebugMousePos.x + 7, mDebugMousePos.y - 7), cv::Scalar(0xffff), 2);
    cv::imshow(OPENCV_WINDOW, debugImage255);
}

void RSC::mouseCallback(int32_t event, int32_t x, int32_t y, int32_t flags, void* userdata) {
    if (event == cv::EVENT_LBUTTONDOWN) {
        // std::cout << "Left button of the mouse is clicked - position (" << x << ", " << y << ")" << std::endl;
    } else if (event == cv::EVENT_RBUTTONDOWN) {
        // std::cout << "Right button of the mouse is clicked - position (" << x << ", " << y << ")" << std::endl;
    } else if (event == cv::EVENT_MBUTTONDOWN) {
        // std::cout << "Middle button of the mouse is clicked - position (" << x << ", " << y << ")" << std::endl;
    } else if (event == cv::EVENT_MOUSEMOVE) {
        mDebugMousePos.x = x;
        mDebugMousePos.y = y;
    }
}

/*****************************************************
 * Range Filter
 */

cv::Mat RSC::depthImgForDesiredDistanceRange(float min, float max, cv::Mat input) {
    cv::Mat adjMap = input.clone();
    for (int32_t y=0; y < adjMap.rows; y++) {
        for (int32_t x=0; x < adjMap.cols; x++) {
            if (adjMap.at<float>(cv::Point(x, y)) < min || adjMap.at<float>(cv::Point(x, y)) > max) {
                adjMap.at<float>(cv::Point(x, y)) = 0;
            }
        }
    }
    return adjMap;
}

void RSC::setRangeSwitch(bool status) {
    mRangeSwitch = status;
}

void RSC::setRange(float min, float max) {
    mRangeMin = min;
    mRangeMax = max;
}

/*****************************************************
 * Hit Pencentage
 */

std::vector<float> RSC::pointCloudZCoordsInRange(float width, float height, float dist) {
    // boost::shared_lock<boost::shared_mutex> lock(mPointcloudMutex);
    float x = width/2;
    float y = width/2;
    if (dist < 200.0f) {
        dist = 200.0f;
    }  // Reserved for range filter
    std::vector<float> zCoords;
    for ( auto i = 0; i < mPclPointCloud.points.size(); i++ ) {
        pcl::PointXYZRGB pt = mPclPointCloud.points.at(i);
        if ( GeneralUtility::inRange<float>(-x, x, pt.x*1000) && GeneralUtility::inRange<float>(-y, y, pt.y*1000) ) {
            zCoords.push_back(pt.z*1000);
        }
    }
    return zCoords;
}

/*****************************************************
 * Accessor
 */

float RSC::getMaxRange() {
    if (mRangeSwitch && mRangeMax < DEPTH_MAX_RANGE) {
        return mRangeMax;
    }
    return DEPTH_MAX_RANGE;
}

float RSC::getMinRange() {
    if (mRangeSwitch && mRangeMin > DEPTH_MIN_RANGE) {
        return mRangeMin;
    }
    return DEPTH_MIN_RANGE;
}

sensor_msgs::Image RSC::getDepthImage() {
    return mDepthImage;
}

sensor_msgs::PointCloud2 RSC::getPC2Cloud() {
    return mPointCloud;
}

}  // namespace Depth
