#include <droneoa_ros/GPSPoint.hpp>

GPSPoint::GPSPoint() {
    latitude_ = 0;
    longitude_ = 0;
    altitude_ = 0;
}

GPSPoint::GPSPoint(float latitude, float longitude, float altitude) {
    latitude_ = latitude;
    longitude_ = longitude;
    altitude_ = altitude;
}