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
#include <droneoa_ros/Utils.hpp>
#include <droneoa_ros/PDN.hpp>

#include <opencv2/imgproc/imgproc.hpp>

static const char* OPENCV_WINDOW_LIDAR = "Lidar Debug window";

LidarInterface::LidarInterface() {
    cv::namedWindow(OPENCV_WINDOW_LIDAR);
}

LidarInterface::~LidarInterface() {
    cv::destroyWindow(OPENCV_WINDOW_LIDAR);
    delete thread_watch_lidar_;
}

void LidarInterface::init(ros::NodeHandle nh, ros::Rate r) {
    n_ = nh;
    r_ = r;
    thread_watch_lidar_ = new boost::thread(boost::bind(&LidarInterface::watchLidarThread, this));
#ifdef DEBUG_LIDAR_POPUP
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
    auto lidar_sub =
        n_.subscribe<sensor_msgs::LaserScan>("/scan", 1000,
            boost::bind(&LidarInterface::lidar_callback, this, _1));

    while (ros::ok()) {
        ros::spinOnce();
        r_.sleep();
    }
}

/* Accesser */
float LidarInterface::getMaxAngle() {
    return scannerData_.angle_max;
}

float LidarInterface::getMinAngle() {
    return scannerData_.angle_min;
}

float LidarInterface::getMaxRange() {
    return scannerData_.range_max;
}

float LidarInterface::getMinRnage() {
    return scannerData_.range_min;
}

std::map<float, degreeSector> LidarInterface::getScannerDataMap() {
    if (scannerDataMap_.size() <= 100) {
        ROS_WARN("Imcomplete Lidar Data Map !!!");
        // @todo should we try to regenerate OR populate with "safe" data set ?
    }
    return scannerDataMap_;
}

float LidarInterface::getIncreament() {
    return scannerData_.angle_increment;
}

std::pair<int, float> LidarInterface::getClosestSectorData() {
    // @todo
}

/* Processing Functions */
void LidarInterface::generateDataMap() {
    scannerDataMap_.clear();
    // @todo Is this safe ?
    unsigned int count = scannerData_.scan_time / scannerData_.time_increment;
    for (unsigned int i = 0; i < count; i++) {
        float degree = radToDeg(getMinAngle() + getIncreament() * i);
        degree = 0 - degree;  // Fix YDLidar's strange coordinate system
        degree = static_cast<int>(degree + LIDAR_ORIENTATION_CW) % 360;
        if (!std::isinf(scannerData_.ranges[i])) {
            scannerDataMap_[degree].push_back(scannerData_.ranges[i] * 100);
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
    ROS_INFO("[LIDAR] MaxAngle: %f, MinAngle: %f, Increment: %f",
        getMaxAngle(), getMinAngle(), getIncreament());
}

void drawLidarPoint(const cv::Mat &img, const cv::Point &center, float angle, float range) {
    double angleradians = angle * M_PI / 180.0f;
    double x = range * std::sin(angleradians);
    double y = range * std::cos(angleradians);
    x += center.x;
    y = center.y - y;
    cv::line(img, center, cv::Point(x, y), cv::Scalar(0, 0, 200), 2);
}

void LidarInterface::drawLidarData() {
    int winWidth = 800;
    int winHeight = 800;
    cv::Point center(winWidth / 2, winHeight / 2);
    cv::Mat lidarDisk(winWidth, winHeight, CV_8UC3, cv::Scalar(0, 0, 0));

    for (auto it = scannerDataMap_.begin(); it != scannerDataMap_.end(); it++) {
        drawLidarPoint(lidarDisk, center, it->first, (it->second)[0]);
    }

    cv::imshow(OPENCV_WINDOW_LIDAR, lidarDisk);
}
