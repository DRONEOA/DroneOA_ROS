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