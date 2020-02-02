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

#ifndef INCLUDE_DRONEOA_ROS_OAUTILS_CAALGLIDAR_HPP_
#define INCLUDE_DRONEOA_ROS_OAUTILS_CAALGLIDAR_HPP_

#include <droneoa_ros/OAUtils/BaseAlg.hpp>
#include <droneoa_ros/LidarInterface.hpp>

// #define DEBUG_ALG_COLLISION_LIDAR

class CAAlgLidar : public BaseAlg {
    LidarInterface *lidar_;
    float lidarThreshold_;
    float lidarPossibility_;
 public:
    CAAlgLidar(CNCInterface *cnc, LidarInterface *lidar);
    ~CAAlgLidar() override;
    void init(LidarInterface *lidar);  // For restart
    bool collect() override;  // Collect required sensor data
    bool plan() override;  // Return false when get around is impossible
};

#endif  // INCLUDE_DRONEOA_ROS_OAUTILS_CAALGLIDAR_HPP_
