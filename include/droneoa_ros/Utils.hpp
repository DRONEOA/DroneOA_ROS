/* Copyright (C) DroneOA Group - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 * Written by Bohan Shi <b34shi@edu.uwaterloo.ca>, August 2019
 */

#ifndef INCLUDE_DRONEOA_ROS_UTILS_HPP_  // NOLINT
#define INCLUDE_DRONEOA_ROS_UTILS_HPP_  // NOLINT

#include <droneoa_ros/GPSPoint.hpp>

GPSPoint getLocationMeter(GPSPoint originLoc, float dNorth, float dEast);
float getDistanceMeter(GPSPoint point1, GPSPoint point2);
float getBearing(GPSPoint point1, GPSPoint point2);

#endif  // NOLINT
