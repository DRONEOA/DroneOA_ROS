/* Copyright (C) 2020 DroneOA Group - All Rights Reserved
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

#ifndef HWI_INTERFACE_LIDARINTERFACE_HPP_  // NOLINT
#define HWI_INTERFACE_LIDARINTERFACE_HPP_  // NOLINT

#include <string>
#include <utility>
#include <map>

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

namespace Lidar {

class LidarInterface {
 public:
    virtual ~LidarInterface() {}

    // Init
    virtual void initWatcherThread() = 0;

    // Basic Data
    virtual float getMaxAngle() = 0;
    virtual float getMinAngle() = 0;
    virtual float getMaxRange() = 0;
    virtual float getMinRnage() = 0;
    virtual float getScanTime() = 0;
    virtual float getTimeIncreament() = 0;
    virtual float getAngleIncreament() = 0;

    // Data Stream
    virtual sensor_msgs::LaserScan getRawDataMap() = 0;

    // Debug
    virtual void printLidarInfo() = 0;
};

}  // namespace Lidar

#endif  // HWI_INTERFACE_LIDARINTERFACE_HPP_  // NOLINT
