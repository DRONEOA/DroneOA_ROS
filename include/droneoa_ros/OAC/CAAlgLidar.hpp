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

#ifndef OAC_CAALGLIDAR_HPP_  // NOLINT
#define OAC_CAALGLIDAR_HPP_  // NOLINT

#include <droneoa_ros/OAC/BaseAlg.hpp>
#include <droneoa_ros/HWI/base/LidarGeneric.hpp>

namespace OAC {

class CAAlgLidar : public BaseAlg {
    Lidar::LidarGeneric *lidar_;
    float lidarThreshold_;
    float lidarPossibility_;
 public:
    CAAlgLidar(CNC::CNCInterface *cnc, Lidar::LidarGeneric *lidar);
    ~CAAlgLidar() override;
    void init(Lidar::LidarGeneric *lidar);  // For restart
    bool collect() override;  // Collect required sensor data
    bool plan() override;  // Return false when get around is impossible
};

}  // namespace OAC

#endif  // OAC_CAALGLIDAR_HPP_  // NOLINT
