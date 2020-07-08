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
#include <droneoa_ros/GUI/Debug/RSCPopup.hpp>

namespace Depth {

RSC::RSC(ros::NodeHandle node, ros::Rate rate) : mNodeHandle(node), mRate(rate) {}

RSC::~RSC() {
    if (mpThreadWatchDepthImg) {
        mpThreadWatchDepthImg->join();
        delete mpThreadWatchDepthImg;
    }
    if (mpThreadWatchPointcloud) {
        mpThreadWatchPointcloud->join();
        delete mpThreadWatchPointcloud;
    }
    ROS_INFO("Destroy RSCInterface");
}

void RSC::initWatcherThread() {
    mCurrentDepthSource = DEPTH_SOURCE_RSC;
    mCurrentPCSource = PC_SOURCE_RSC;
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

    GUI::GUISubject::notifyGUIPopups();
}

void RSC::pointcloud_callback(const sensor_msgs::PointCloud2ConstPtr& msg) {
    // boost::unique_lock<boost::shared_mutex> uniqueLock(mPointcloudMutex);
    mPointCloud = *msg;
    // Convert to pointcloud
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(mPointCloud, pcl_pc2);
    pcl::fromPCLPointCloud2(pcl_pc2, mPclPointCloud);
    GUI::GUISubject::notifyGUIPopups();

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
        ros::spinOnce();
        mRate.sleep();
    }
}

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

std::pair<float, float> RSC::getRangeFilterSetting() {
    if (mRangeSwitch) {
        return {mRangeMin, mRangeMax};
    } else {
        return {-1.0, -1.0};
    }
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

cv::Mat RSC::getDepthCVFrame() {
    return mDepthFrame;
}

sensor_msgs::PointCloud2 RSC::getPC2Cloud() {
    return mPointCloud;
}

pcl::PointCloud<pcl::PointXYZRGB> RSC::getPCLCloud() {
    return mPclPointCloud;
}

}  // namespace Depth
