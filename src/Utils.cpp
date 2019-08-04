#include <droneoa_ros/Utils.hpp>
#include <math.h>

GPSPoint getLocationMeter(GPSPoint originLoc, float dNorth, float dEast) {
    // Reference: http://gis.stackexchange.com/questions/2951/algorithm-for-offsetting-a-latitude-longitude-by-some-amount-of-meters
    float earth_radius = 6378137.0; // Radius of "spherical" earth
    // Coordinate offsets in radians
    float dLat = dNorth/earth_radius;
    float dLon = dEast/(earth_radius * cos(M_PI * originLoc.latitude_/180));
    // New position in decimal degrees
    float newlat = originLoc.latitude_ + (dLat * 180/M_PI);
    float newlon = originLoc.longitude_ + (dLon * 180/M_PI);

    return GPSPoint(newlat, newlon, originLoc.altitude_);
}

float getDistanceMeter(GPSPoint point1, GPSPoint point2) {
    // Reference: https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
    float dlat = point2.latitude_ - point1.latitude_;
    float dlong = point2.longitude_ - point1.longitude_;
    return sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5;
}

float getBearing(GPSPoint point1, GPSPoint point2) {
    float off_x = point2.longitude_ - point1.longitude_;
    float off_y = point2.latitude_ - point1.latitude_;
    float bearing = 90.00 + atan2(-off_y, off_x) * 57.2957795;
    if (bearing < 0) {
        bearing += 360.00;
    }
    return bearing;
}