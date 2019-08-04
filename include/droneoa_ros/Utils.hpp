#ifndef UTILS_OA_
#define UTILS_OA_

#include <droneoa_ros/GPSPoint.hpp>

GPSPoint getLocationMeter(GPSPoint originLoc, float dNorth, float dEast);
float getDistanceMeter(GPSPoint point1, GPSPoint point2);
float getBearing(GPSPoint point1, GPSPoint point2);

#endif