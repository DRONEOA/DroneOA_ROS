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
 * All Reference Attached
 */

#include <math.h>
#include <ros/ros.h>
#include <droneoa_ros/HWI/Utils/CNCUtils.hpp>
#include <droneoa_ros/PDN.hpp>
#include <droneoa_ros/Utils/DataPool.hpp>

namespace CNC {

GPSPoint CNCUtility::getLocationMeter(GPSPoint originLoc, float dNorth, float dEast) {
    // Reference: http://gis.stackexchange.com/questions/2951/algorithm-for-offsetting-a-latitude-longitude-by-some-amount-of-meters
    float earth_radius = 6378137.0;  // Radius of "spherical" earth
    // Coordinate offsets in radians
    float dLat = dNorth/earth_radius;
    float dLon = dEast/(earth_radius * cos(M_PI * originLoc.mX/180));
    // New position in decimal degrees
    float newlat = originLoc.mX + (dLat * 180/M_PI);
    float newlon = originLoc.mY + (dLon * 180/M_PI);

    return GPSPoint(newlat, newlon, originLoc.mZ);
}

float CNCUtility::getDistanceMeter(GPSPoint point1, GPSPoint point2) {
    // Reference: https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
    float dlat = point2.mX - point1.mX;
    float dlong = point2.mY - point1.mY;
    return sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5;
}

float CNCUtility::getBearing(GPSPoint point1, GPSPoint point2) {
    float off_x = point2.mY - point1.mY;
    float off_y = point2.mX - point1.mX;
    float bearing = 90.00 + atan2(-off_y, off_x) * 57.2957795;
    if (bearing < 0) {
        bearing += 360.00;
    }
    return bearing;
}

std::pair<float, float> CNCUtility::getNorthEastDistanceFromHeading(float heading, float distance) {
    float dNorth = distance * cos(heading * M_PI / 180.0f);
    float dEast = distance * sin(heading * M_PI / 180.0f);
    return std::pair<float, float>(dNorth, dEast);
}

std::pair<float, float> CNCUtility::getNorthEastFromPoints(GPSPoint point1, GPSPoint point2) {
    float bearing = getBearing(point1, point2);
    float distance = getDistanceMeter(point1, point2);
    double dNorth = distance * cos(bearing * M_PI / 180.0);
    double dEast = distance * sin(bearing * M_PI / 180.0);
    return std::pair<float, float>(dNorth, dEast);
}

float CNCUtility::validAltitudeCMD(float targetAltitude) {
    if (targetAltitude > VEHICLE_MAX_ALTITUDE_RELATIVE) {
        ROS_WARN("[CNC Utility] target altitude exceed max allowed altitude !");
        return VEHICLE_MAX_ALTITUDE_RELATIVE;
    }
    return targetAltitude;
}

float CNCUtility::validSpeedCMD(float targetSpeed) {
    DP::DataPool msDP;
    int32_t vehicleMaxSpeedHorizontal = boost::any_cast<int32_t>(
            msDP.getDataAsInt(DP::CONF_VEHICLE_MAX_SPEED_HORIZONTAL));
    if (targetSpeed > vehicleMaxSpeedHorizontal) {
        ROS_WARN("[CNC Utility] target speed exceed max allowed speed %d m/s!", vehicleMaxSpeedHorizontal);
        return vehicleMaxSpeedHorizontal;
    }
    return targetSpeed;
}

geometry_msgs::Vector3 CNCUtility::quaternionToRPY(geometry_msgs::Quaternion quaternion) {
    geometry_msgs::Vector3 RPY;
    double w = quaternion.w;
    double x = quaternion.x;
    double y = quaternion.y;
    double z = quaternion.z;
    RPY.x = 1.0 * (asin(-2 * x * z + 2 * w * y));  // pitch
    RPY.y = 1.0 * (atan2(2 * y * z + 2 * w * x, -2 * x * x - 2 * y * y + 1));  // roll
    RPY.z = 1.0 * (atan2(2 * (x * y + w * z), w * w + x * x - y * y - z * z));  // yaw
    return RPY;
}

}  // namespace CNC
