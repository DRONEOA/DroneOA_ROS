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
 * Written by Bohan Shi <b34shi@edu.uwaterloo.ca>, August 2019
 */

#include <math.h>

#include <droneoa_ros/HWI/Utils/GPSPoint.hpp>
#include <droneoa_ros/PDN.hpp>

GPSPoint::GPSPoint() : Position3D() {}

GPSPoint::GPSPoint(float latitude, float longitude, float altitude) : Position3D(latitude, longitude, altitude) {}

bool GPSPoint::operator==(const Position3D& other) const {
    float dlat = this->mX - other.mX;
    float dlong = this->mY - other.mY;
    float dist = sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5;
    if (dist < GPS_COMPARE_DIFF_MAX) {
        return true;
    }
    return false;
}

std::string GPSPoint::AsString() const {
    std::string result = "GPSPoint: ";
    result += std::to_string(mX) + " ";
    result += std::to_string(mY) + " ";
    result += std::to_string(mZ);
    return result;
}
