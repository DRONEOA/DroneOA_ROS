/* Copyright (C) DroneOA Group - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 * Written by Bohan Shi <b34shi@edu.uwaterloo.ca>, August 2019
 */

#ifndef INCLUDE_DRONEOA_ROS_GPSPOINT_HPP_  // NOLINT
#define INCLUDE_DRONEOA_ROS_GPSPOINT_HPP_  // NOLINT

class GPSPoint {
 public:
    GPSPoint();
    GPSPoint(float latitude, float longitude, float altitude);
    virtual ~GPSPoint() = default;

    float latitude_;
    float longitude_;
    float altitude_;
};

#endif  // NOLINT
