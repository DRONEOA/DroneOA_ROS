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

#ifndef INCLUDE_DRONEOA_ROS_COLLISIONAVOIDANCE_HPP_  // NOLINT
#define INCLUDE_DRONEOA_ROS_COLLISIONAVOIDANCE_HPP_  // NOLINT

#include <droneoa_ros/OAUtils/BaseAlg.hpp>
#include <droneoa_ros/LidarInterface.hpp>
#include <droneoa_ros/RSCInterface.hpp>

// #define DEBUG_ALG_COLLISION

class CollisionAvoidanceAlg : public BaseAlg {
    LidarInterface *lidar_;
    RSCInterface *rsc_;
 public:
    CollisionAvoidanceAlg(CNCInterface *cnc, LidarInterface *lidar, RSCInterface *rsc);
    ~CollisionAvoidanceAlg() override;
    void init(LidarInterface *lidar, RSCInterface *rsc);  // For restart
    bool collect() override;  // Collect required sensor data
    bool plan() override;  // Return false when get around is impossible

    std::vector<std::pair<CMD_QUEUE_TYPES, std::string>> getCommandQueue() override;
};

#endif