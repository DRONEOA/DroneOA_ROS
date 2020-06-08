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
 * Written by Xiao Zhou <x258zhou@uwaterloo.ca>, May 2020.
 */

#ifndef HWI_UTILS_LIDARUTILS_HPP_  // NOLINT
#define HWI_UTILS_LIDARUTILS_HPP_  // NOLINT

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <map>
#include <droneoa_ros/HWI/base/LidarGeneric.hpp>

namespace Lidar {

class LidarUtility {
 public:
    static void pitchCorrection(geometry_msgs::Vector3 RPY, std::map<float, degreeSector> data);
};

}  // namespace Lidar

#endif  // HWI_UTILS_LIDARUTILS_HPP_  // NOLINT
