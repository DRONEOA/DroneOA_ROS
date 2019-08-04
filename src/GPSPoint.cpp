/* Copyright (C) DroneOA Group - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 * Written by Bohan Shi <b34shi@edu.uwaterloo.ca>, August 2019
 */

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
