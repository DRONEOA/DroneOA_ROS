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

#ifndef HWI_UTILS_CNCUTILS_HPP_  // NOLINT
#define HWI_UTILS_CNCUTILS_HPP_  // NOLINT

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <string>
#include <vector>
#include <utility>
#include <droneoa_ros/HWI/Utils/GPSPoint.hpp>

namespace CNC {

class CNCUtility {
 public:
    /**
     * @brief Get the GPD location dNorth, dEast from the original position
     * @param originLoc start GPS point
     * @param dNorth north axis distance
     * @param dEast east axis distance
     * @return stop location GPSPoint 
     */
    static GPSPoint getLocationMeter(GPSPoint originLoc, float dNorth, float dEast);

    /**
     * @brief Get distance between 2 GPS point in meter
     * @param point1 
     * @param point2 
     * @return float distance between 2 GPS points
     */
    static float getDistanceMeter(GPSPoint point1, GPSPoint point2);

    /**
     * @brief Get bearing in order to reach a location
     * @param point1 start point
     * @param point2 end point
     * @return bearing required to reach the end point (float)
     */
    static float getBearing(GPSPoint point1, GPSPoint point2);

    /**
     * @brief Get target dNorth, dEast with heading and distance
     * @param heading 
     * @param distance 
     * @return std::pair<float, float> North axis distance, East axis distance
     */
    static std::pair<float, float> getNorthEastDistanceFromHeading(float heading, float distance);

    /**
     * @brief Get relative dNorth, dEast with heading and distancegiven 2 GPS points
     * @param point1 start point
     * @param point2 end point 
     * @return std::pair<float, float> North axis distance, East axis distance
     */
    static std::pair<float, float> getNorthEastFromPoints(GPSPoint point1, GPSPoint point2);

    /**
     * @brief Chack target altitude, if exceed max allowed, set to max
     * @param targetAltitude 
     * @return float 
     */
    static float validAltitudeCMD(float targetAltitude);

    /**
     * @brief Chack target speed, if exceed max allowed, set to max
     * @param targetSpeed 
     * @return float 
     */
    static float validSpeedCMD(float targetSpeed);

    /**
     * @brief Convert quaternion to roll & pitch & yaw
     * @param quaternion 
     * @return geometry_msgs::Vector3 (rad)
     *         x: pitch
     *         y: roll
     *         z: yaw
     */
    static geometry_msgs::Vector3 quaternionToRPY(geometry_msgs::Quaternion quaternion);
};

}  // namespace CNC

#endif  // HWI_UTILS_CNCUTILS_HPP_  // NOLINT
