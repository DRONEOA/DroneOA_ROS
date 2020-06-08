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
 * Written by Xiao Zhou <x258zhou@uwaterloo.ca>, May. 2020
 * All Reference Attached
 */

#include <math.h>
#include <ros/ros.h>
#include <droneoa_ros/HWI/Utils/LidarUtils.hpp>

namespace Lidar {

void LidarUtility::pitchCorrection(geometry_msgs::Vector3 RPY, std::map<float, degreeSector> data) {
    for (auto it = data.begin(); it != data.end(); it++) {
        float angledX = it->second[0] * cos(it->first * M_PI/180);
        float angledY = it->second[0] * sin(it->first * M_PI/180);
        float x = angledX * cos(RPY.x);  // pitch
        float y = angledY * cos(RPY.y);  // roll
        it->second[0] = sqrt(x * x + y * y);
    }
}

}  // namespace Lidar

