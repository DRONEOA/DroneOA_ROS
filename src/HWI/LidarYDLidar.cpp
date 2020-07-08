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

#include <droneoa_ros/HWI/LidarYDLidar.hpp>

namespace Lidar {

LidarYDLidar::LidarYDLidar(ros::NodeHandle node, ros::Rate rate) : LidarGeneric(node, rate) {}

LidarYDLidar::~LidarYDLidar() {}

void LidarYDLidar::printLidarInfo() {
    ROS_INFO("[LIDAR YDLidar] MaxRange: %f, MinRange: %f",
        getMaxRange(), getMinRnage());
    ROS_INFO("[LIDAR YDLidar] RAW MaxAngle: %f, MinAngle: %f, Increment: %f",
        getMaxAngle(), getMinAngle(), getAngleIncreament());
    ROS_INFO("[LIDAR YDLidar] time_increment: %f, scan_time: %f",
        getTimeIncreament(), getScanTime());
}

void LidarYDLidar::initWatcherThread() {
    mCurrentLidarSource = LIDAR_SOURCE_YDLIDAR;
    mpThreadWatchLidar = new boost::thread(boost::bind(&LidarYDLidar::watchLidarThread, this));
    GUI::GUISubject::notifyGUIPopups();
    ROS_INFO("[LIDAR YDLidar] init");
}

}  // namespace Lidar
