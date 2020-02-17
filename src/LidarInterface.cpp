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

#include <droneoa_ros/LidarInterface.hpp>
#include <droneoa_ros/Utils/GeneralUtils.hpp>
#include <droneoa_ros/PDN.hpp>

#include <opencv2/imgproc/imgproc.hpp>

static const char* OPENCV_WINDOW_LIDAR = "Lidar Debug window";

LidarInterface::LidarInterface() {
#ifdef DEBUG_LIDAR_POPUP
    cv::namedWindow(OPENCV_WINDOW_LIDAR);
#endif
}

LidarInterface::~LidarInterface() {
#ifdef DEBUG_LIDAR_POPUP
    cv::destroyWindow(OPENCV_WINDOW_LIDAR);
#endif
    delete thread_watch_lidar_;
    ROS_INFO("Destroy LidarInterface");
}

void LidarInterface::init(ros::NodeHandle nh, ros::Rate r) {
    n_ = nh;
    r_ = r;
    currentLidarSource_ = LIDAR_SOURCE_YDLIDAR;
    thread_watch_lidar_ = new boost::thread(boost::bind(&LidarInterface::watchLidarThread, this));
#ifdef DEBUG_LIDAR_POPUP
    cv::startWindowThread();  // DEBUG
    drawLidarData();
#endif
    ROS_INFO("[LIDAR] init");
}

/* Callback */
void LidarInterface::lidar_callback(const sensor_msgs::LaserScanConstPtr& msg) {
    scannerData_ = *msg;
    // Pre-processing
    generateDataMap();
#ifdef DEBUG_LIDAR_POPUP
    drawLidarData();
#endif
}

/* Thread */
void LidarInterface::watchLidarThread() {
    lidar_sub_ =
        n_.subscribe<sensor_msgs::LaserScan>(currentLidarSource_, 1000,
            boost::bind(&LidarInterface::lidar_callback, this, _1));

    while (ros::ok()) {
        ros::spinOnce();
        r_.sleep();
    }
}

void LidarInterface::changeLidarSource(std::string lidarSource) {
    currentLidarSource_ = lidarSource;
    lidar_sub_.shutdown();
    lidar_sub_ =
        n_.subscribe<sensor_msgs::LaserScan>(lidarSource, 1000,
            boost::bind(&LidarInterface::lidar_callback, this, _1));
}

/* Accesser */
float LidarInterface::getMaxAngle() {
    return scannerData_.angle_max;
}

float LidarInterface::getMinAngle() {
    return scannerData_.angle_min;
}

float LidarInterface::getMaxRange() {
    return LIDAR_FILTER_HIGH;
}

float LidarInterface::getMinRnage() {
    return LIDAR_FILTER_LOW;
}

std::map<float, degreeSector> LidarInterface::getScannerDataMap() {
    if (scannerDataMap_.size() <= 0) {
        ROS_WARN("Imcomplete Lidar Data Map !!!");
        // @todo should we try to regenerate OR populate with "safe" data set ?
    }
    return scannerDataMap_;
}

float LidarInterface::getIncreament() {
    return scannerData_.angle_increment;
}

std::pair<float, float> LidarInterface::getClosestSectorData() {
    float minRange = 100000;
    float minAngle = 0;
    for (auto it = scannerDataMap_.begin(); it != scannerDataMap_.end(); it++) {
        if ((it->second)[0] < minRange) {
            minRange = (it->second)[0];
            minAngle = it->first;
        }
    }
    return std::pair<float, float>(minAngle, minRange);
}

/* Processing Functions */
void LidarInterface::generateDataMap() {
    scannerDataMap_.clear();
    // @todo Is this safe ?
    unsigned int count = (scannerData_.angle_max - scannerData_.angle_min) / scannerData_.angle_increment;
    for (unsigned int i = 0; i < count; i++) {
        if (i >= scannerData_.ranges.size()) {
            ROS_WARN("+LidarInterface::generateDataMap: Missing Data, expect: %u actual: %zu",
                count, scannerData_.ranges.size());
            break;
        }
        float degree = GeneralUtility::radToDeg(getMinAngle() + getIncreament() * i);
        degree = 0 - degree;  // Fix YDLidar's strange coordinate system
        degree = static_cast<int>(degree + LIDAR_ORIENTATION_CW) % 360;
        if (!std::isinf(scannerData_.ranges[i]) &&
                scannerData_.ranges[i] >= LIDAR_FILTER_LOW &&
                scannerData_.ranges[i] <= LIDAR_FILTER_HIGH) {
            scannerDataMap_[degree].push_back(scannerData_.ranges[i]);
        }
    }
    generateDegreeSector();
}

void LidarInterface::generateDegreeSector() {
    for (auto it = scannerDataMap_.begin(); it != scannerDataMap_.end(); it++) {
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
void LidarInterface::printLidarInfo() {
    ROS_INFO("[LIDAR] MaxRange: %f, MinRange: %f",
        getMaxRange(), getMinRnage());
    ROS_INFO("[LIDAR] RAW MaxAngle: %f, MinAngle: %f, Increment: %f",
        getMaxAngle(), getMinAngle(), getIncreament());
    ROS_INFO("[LIDAR] time_increment: %f, scan_time: %f",
        scannerData_.time_increment, scannerData_.scan_time);
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

void LidarInterface::drawLidarData() {
    int winWidth = 800;
    int winHeight = 800;
    cv::Point center(winWidth / 2, winHeight / 2);
    cv::Mat lidarDisk(winWidth, winHeight, CV_8UC3, cv::Scalar(0, 0, 0));
    // Draw center
    cv::circle(lidarDisk, center, 4, cv::Scalar(200, 0, 0), 2);
    // Draw lidar points
    for (auto it = scannerDataMap_.begin(); it != scannerDataMap_.end(); it++) {
        drawLidarPoint(lidarDisk, center, it->first, (it->second)[0]);
    }
    // Draw closest
    std::pair<float, float> minPoint = getClosestSectorData();
    drawLidarPoint(lidarDisk, center, minPoint.first, minPoint.second, true);

    cv::imshow(OPENCV_WINDOW_LIDAR, lidarDisk);
}
