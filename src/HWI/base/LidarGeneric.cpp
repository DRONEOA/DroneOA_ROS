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

#include <limits>

#include <droneoa_ros/HWI/base/LidarGeneric.hpp>
#include <droneoa_ros/Utils/GeneralUtils.hpp>

#include <opencv2/imgproc/imgproc.hpp>

namespace Lidar {

static const char* OPENCV_WINDOW_LIDAR = "Lidar Debug window";

LidarGeneric::LidarGeneric(ros::NodeHandle node, ros::Rate rate) : mNodeHandle(node), mRate(rate) {
    mCurrentLidarSource = LIDAR_SOURCE_YDLIDAR;
#ifdef DEBUG_LIDAR_POPUP
    cv::namedWindow(OPENCV_WINDOW_LIDAR);
#endif
}

LidarGeneric::~LidarGeneric() {
#ifdef DEBUG_LIDAR_POPUP
    cv::destroyWindow(OPENCV_WINDOW_LIDAR);
#endif
    if (mpThreadWatchLidar) {
        mpThreadWatchLidar->join();
        delete mpThreadWatchLidar;
    }
    ROS_INFO("Destroy LidarGeneric");
}

void LidarGeneric::initWatcherThread() {
    mCurrentLidarSource = LIDAR_SOURCE_YDLIDAR;
    mpThreadWatchLidar = new boost::thread(boost::bind(&LidarGeneric::watchLidarThread, this));
#ifdef DEBUG_LIDAR_POPUP
    cv::startWindowThread();  // DEBUG
    drawLidarData();
#endif
    ROS_INFO("[LIDAR Generic] init");
}

/* Callback */
void LidarGeneric::lidar_callback(const sensor_msgs::LaserScanConstPtr& msg) {
    mScannerData = *msg;
    // Pre-processing
    generateDataMap();
#ifdef DEBUG_LIDAR_POPUP
    drawLidarData();
#endif
}

/* Thread */
void LidarGeneric::watchLidarThread() {
    mLidarSub =
        mNodeHandle.subscribe<sensor_msgs::LaserScan>(mCurrentLidarSource, 1000,
            boost::bind(&LidarGeneric::lidar_callback, this, _1));

    while (ros::ok()) {
        ros::spinOnce();
        mRate.sleep();
    }
}

void LidarGeneric::changeLidarSource(std::string lidarSource) {
    mCurrentLidarSource = lidarSource;
    mLidarSub.shutdown();
    mLidarSub =
        mNodeHandle.subscribe<sensor_msgs::LaserScan>(lidarSource, 1000,
            boost::bind(&LidarGeneric::lidar_callback, this, _1));
}

/* Accesser */
float LidarGeneric::getMaxAngle() {
    return mScannerData.angle_max;
}

float LidarGeneric::getMinAngle() {
    return mScannerData.angle_min;
}

float LidarGeneric::getMaxRange() {
    return LIDAR_FILTER_HIGH;
}

float LidarGeneric::getMinRnage() {
    return LIDAR_FILTER_LOW;
}

float LidarGeneric::getScanTime() {
    return mScannerData.scan_time;
}

float LidarGeneric::getTimeIncreament() {
    return mScannerData.time_increment;
}

float LidarGeneric::getAngleIncreament() {
    return mScannerData.angle_increment;
}

sensor_msgs::LaserScan LidarGeneric::getRawDataMap() {
    return mScannerData;
}

std::map<float, degreeSector> LidarGeneric::getSectorDataMap() {
    if (mSectorDataMap.size() <= 0) {
        ROS_WARN("Imcomplete Lidar Data Map !!!");
        // @todo should we try to regenerate OR populate with "safe" data set ?
    }
    return mSectorDataMap;
}

std::pair<float, float> LidarGeneric::getClosestSectorData() {
    float minRange = std::numeric_limits<float>::infinity();
    float minAngle = 0;
    for (auto it = mSectorDataMap.begin(); it != mSectorDataMap.end(); it++) {
        if ((it->second)[0] < minRange) {
            minRange = (it->second)[0];
            minAngle = it->first;
        }
    }
    return std::pair<float, float>(minAngle, minRange);
}

/* Processing Functions */
void LidarGeneric::generateDataMap() {
    mSectorDataMap.clear();
    // @todo Is this safe ?
    uint32_t count = (mScannerData.angle_max - mScannerData.angle_min) / mScannerData.angle_increment;
    for (uint32_t i = 0; i < count; i++) {
        if (i >= mScannerData.ranges.size()) {
            ROS_WARN("+LidarGeneric::generateDataMap: Missing Data, expect: %u actual: %zu",
                count, mScannerData.ranges.size());
            break;
        }
        float degree = GeneralUtility::radToDeg(getMinAngle() + getAngleIncreament() * i);
        degree = 0 - degree;  // Fix YDLidar's strange coordinate system
        degree = static_cast<int32_t>(degree + LIDAR_ORIENTATION_CW) % 360;
        if (!std::isinf(mScannerData.ranges[i]) &&
                mScannerData.ranges[i] >= LIDAR_FILTER_LOW &&
                mScannerData.ranges[i] <= LIDAR_FILTER_HIGH) {
            mSectorDataMap[degree].push_back(mScannerData.ranges[i]);
        }
    }
    generateDegreeSector();
}

void LidarGeneric::generateDegreeSector() {
    for (auto it = mSectorDataMap.begin(); it != mSectorDataMap.end(); it++) {
        if ((it->second).size() > 1) {
            // Use average for now
            float sum = 0.0;
            for (auto itVec = (it->second).begin(); itVec != (it->second).end(); ++itVec) {
                sum += *itVec;
            }
            (it->second)[0] = sum / (it->second).size();
        }
    }
}

/* Debug */
void LidarGeneric::printLidarInfo() {
    ROS_INFO("[LIDAR GENERIC] MaxRange: %f, MinRange: %f",
        getMaxRange(), getMinRnage());
    ROS_INFO("[LIDAR GENERIC] RAW MaxAngle: %f, MinAngle: %f, Increment: %f",
        getMaxAngle(), getMinAngle(), getAngleIncreament());
    ROS_INFO("[LIDAR GENERIC] time_increment: %f, scan_time: %f",
        getTimeIncreament(), getScanTime());
}

void drawLidarPoint(const cv::Mat &img, const cv::Point &center, float angle, float range, bool isLine = false) {
    range *= LIDAR_POPUP_SCALE;  // zoom up to draw better radar map
    double angleradians = angle * M_PI / 180.0f;
    double x = range * std::sin(angleradians);
    double y = range * std::cos(angleradians);
    x += center.x;
    y = center.y - y;
    if (isLine) {
        cv::line(img, center, cv::Point(x, y), cv::Scalar(0, 100, 100), 2);
    } else {
        cv::circle(img, cv::Point(x, y), 2, cv::Scalar(0, 0, 200));
    }
}

void LidarGeneric::drawLidarData() {
    int32_t winWidth = 800;
    int32_t winHeight = 800;
    cv::Point center(winWidth / 2, winHeight / 2);
    cv::Mat lidarDisk(winWidth, winHeight, CV_8UC3, cv::Scalar(0, 0, 0));
    // Draw center
    cv::circle(lidarDisk, center, 4, cv::Scalar(200, 0, 0), 2);
    // Draw lidar points
    for (auto it = mSectorDataMap.begin(); it != mSectorDataMap.end(); it++) {
        drawLidarPoint(lidarDisk, center, it->first, (it->second)[0]);
    }
    // Draw closest
    std::pair<float, float> minPoint = getClosestSectorData();
    drawLidarPoint(lidarDisk, center, minPoint.first, minPoint.second, true);

    cv::imshow(OPENCV_WINDOW_LIDAR, lidarDisk);
}

}  // namespace Lidar
