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

#ifndef INCLUDE_DRONEOA_ROS_UTILS_CNCUTILS_HPP_  // NOLINT
#define INCLUDE_DRONEOA_ROS_UTILS_CNCUTILS_HPP_  // NOLINT

#include <string>
#include <vector>

#include <utility>
#include <droneoa_ros/GPSPoint.hpp>

class CNCUtility {
 public:
    // Get the GPD location dNorth, dEast from the original position
    static GPSPoint getLocationMeter(GPSPoint originLoc, float dNorth, float dEast);

    // Get distance between 2 GPS point in meter
    static float getDistanceMeter(GPSPoint point1, GPSPoint point2);

    // Get bearing in order to reach a location
    static float getBearing(GPSPoint point1, GPSPoint point2);

    // Get target dNorth, dEast with heading and distance
    static std::pair<float, float> getNorthEastDistanceFromHeading(float heading, float distance);

    // Chack target altitude, if exceed max allowed, set to max
    static float validAltitudeCMD(float targetAltitude);

    // Chack target speed, if exceed max allowed, set to max
    static float validSpeedCMD(float targetSpeed);
};

#endif  // INCLUDE_DRONEOA_ROS_UTILS_CNCUTILS_HPP_  // NOLINT
