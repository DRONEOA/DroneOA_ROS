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

namespace CNC {

GPSPoint CNCUtility::getLocationMeter(GPSPoint originLoc, float dNorth, float dEast) {
    // Reference: http://gis.stackexchange.com/questions/2951/algorithm-for-offsetting-a-latitude-longitude-by-some-amount-of-meters
    float earth_radius = 6378137.0;  // Radius of "spherical" earth
    // Coordinate offsets in radians
    float dLat = dNorth/earth_radius;
    float dLon = dEast/(earth_radius * cos(M_PI * originLoc.latitude_/180));
    // New position in decimal degrees
    float newlat = originLoc.latitude_ + (dLat * 180/M_PI);
    float newlon = originLoc.longitude_ + (dLon * 180/M_PI);

    return GPSPoint(newlat, newlon, originLoc.altitude_);
}

float CNCUtility::getDistanceMeter(GPSPoint point1, GPSPoint point2) {
    // Reference: https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
    float dlat = point2.latitude_ - point1.latitude_;
    float dlong = point2.longitude_ - point1.longitude_;
    return sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5;
}

float CNCUtility::getBearing(GPSPoint point1, GPSPoint point2) {
    float off_x = point2.longitude_ - point1.longitude_;
    float off_y = point2.latitude_ - point1.latitude_;
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

float CNCUtility::validAltitudeCMD(float targetAltitude) {
    if (targetAltitude > VEHICLE_MAX_ALTITUDE_RELATIVE) {
        ROS_WARN("target altitude exceed max allowed altitude !");
        return VEHICLE_MAX_ALTITUDE_RELATIVE;
    }
    return targetAltitude;
}

float CNCUtility::validSpeedCMD(float targetSpeed) {
    if (targetSpeed > VEHICLE_MAX_SPEED_HORIZONTAL) {
        ROS_WARN("target speed exceed max allowed speed !");
        return VEHICLE_MAX_SPEED_HORIZONTAL;
    }
    return targetSpeed;
}

}  // namespace CNC