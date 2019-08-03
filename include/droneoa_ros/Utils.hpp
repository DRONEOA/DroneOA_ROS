#ifndef UTILS_OA_
#define UTILS_OA_

#include <droneoa_ros/GPSPoint.hpp>

GPSPoint getLocationMeter(GPSPoint originLoc, float dNorth, float dEast);

#endif