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

LidarInterface::LidarInterface() {}

LidarInterface::~LidarInterface() {
    delete thread_watch_lidar_;
}

void LidarInterface::init(ros::NodeHandle nh, ros::Rate r) {
    n_ = nh;
    r_ = r;
    thread_watch_lidar_ = new boost::thread(boost::bind(&LidarInterface::watchLidarThread, this));
    ROS_INFO("[LIDAR] init");
}

/* Callback */
void LidarInterface::lidar_callback(const sensor_msgs::LaserScanConstPtr& msg) {
    scannerData_ = *msg;
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

std::vector<float> LidarInterface::getScannerDataVec() {
    return scannerData_.ranges;
}

float LidarInterface::getIncreament() {
    return scannerData_.angle_increment;
}
