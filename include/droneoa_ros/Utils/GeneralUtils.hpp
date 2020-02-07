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
 * Written by Bohan Shi <b34shi@edu.uwaterloo.ca>, Feb. 2020
 */

#ifndef INCLUDE_DRONEOA_ROS_UTILS_GENERALUTILS_HPP_  // NOLINT
#define INCLUDE_DRONEOA_ROS_UTILS_GENERALUTILS_HPP_  // NOLINT

#include <string>
#include <vector>

class GeneralUtility {
 public:
    // Get deg angle from rad angle
    static float radToDeg(float rad);

    // Read config from file [float array]
    // Format: <key> <data1> <data2> ...
    static std::vector<float> getFloatDataFromConfig(std::string path, std::string keyName);


    // Check if target is in range [min, max] inclusive
    template<typename T>
    static bool inRange(T min, T max, T target) {
        return (target >= min && target <= max);
    }

    // Scale target in [rmin, rmax] to [tmin, tmax]
    template<typename T>
    static T scale(T target, T rmin, T rmax, T tmin, T tmax) {
        return ((target-rmin)*(tmax-tmin))/(rmax-rmin)+tmin;
    }
};



#endif  // INCLUDE_DRONEOA_ROS_UTILS_GENERALUTILS_HPP_  // NOLINT