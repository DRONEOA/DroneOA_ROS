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
 * Written by Bohan Shi <b34shi@edu.uwaterloo.ca>, Feb 2020
 */

#ifndef OAC_OAALGGLO_HPP_  // NOLINT
#define OAC_OAALGGLO_HPP_  // NOLINT

#include <droneoa_ros/OAC/BaseAlg.hpp>
#include <droneoa_ros/HWI/base/LidarGeneric.hpp>
#include <std_msgs/String.h>
#include <mavros_msgs/Trajectory.h>

namespace OAC {

class OAAlgGLO : public BaseAlg {
    Lidar::LidarGeneric *mpLidar;
    ros::NodeHandle n;
    ros::Publisher start3DGLO;
    ros::Subscriber getData;
    void pathCallback(const mavros_msgs::Trajectory& msg);
 public:
    OAAlgGLO(CNC::CNCInterface *cnc, Lidar::LidarGeneric *lidar, ros::NodeHandle n);
    ~OAAlgGLO() override;
    void init(Lidar::LidarGeneric *lidar);  // For restart
    bool collect() override;  // Collect required sensor data
    bool plan() override;  // Return false when get around is impossible
};

}  // namespace OAC

#endif  // OAC_OAALGGLO_HPP_  // NOLINT
