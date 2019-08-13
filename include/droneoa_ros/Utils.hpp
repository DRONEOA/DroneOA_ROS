/* Copyright (C) DroneOA Group - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 * Written by Bohan Shi <b34shi@edu.uwaterloo.ca>, August 2019
 */

#ifndef INCLUDE_DRONEOA_ROS_UTILS_HPP_  // NOLINT
#define INCLUDE_DRONEOA_ROS_UTILS_HPP_  // NOLINT

#include <string>
#include <vector>

#include <utility>
#include <droneoa_ros/GPSPoint.hpp>

// Get the GPD location dNorth, dEast from the original position
GPSPoint getLocationMeter(GPSPoint originLoc, float dNorth, float dEast);

// Get distance between 2 GPS point in meter
float getDistanceMeter(GPSPoint point1, GPSPoint point2);

// Get bearing in order to reach a location
float getBearing(GPSPoint point1, GPSPoint point2);

// Get target dNorth, dEast with heading and distance
std::pair<float, float> getNorthEastDistanceFromHeading(float heading, float distance);

// Get deg angle from rad angle
float radToDeg(float rad);

// Read config from file [float array]
// Format: <key> <data1> <data2> ...
std::vector<float> getFloatDataFromConfig(std::string path, std::string keyName);

#endif  // NOLINT
