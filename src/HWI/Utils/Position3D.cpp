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
 * Written by Bohan Shi <b34shi@edu.uwaterloo.ca>, July 2020
 */

#include <math.h>

#include <droneoa_ros/HWI/Utils/Position3D.hpp>
#include <droneoa_ros/PDN.hpp>

Position3D::Position3D() : mX(0), mY(0), mZ(0) {}

Position3D::Position3D(double x, double y, double z) : mX(x), mY(y), mZ(z) {}

Position3D::~Position3D() {}

float Position3D::getDistanceTo(const Position3D &other) const {
    float dx = mX - other.mX;
    float dy = mY - other.mY;
    float dz = mZ - other.mZ;
    return sqrt((dx*dx) + (dy*dy) + (dz*dz));
}

bool Position3D::operator==(const Position3D& other) const {
    return getDistanceTo(other) <= POS3D_COMPARE_DIFF_MAX;
}

std::string Position3D::AsString() const {
    std::string result = "LocalPoint: x: " + std::to_string(mX) + " y: " +
                std::to_string(mY) + " z: " + std::to_string(mZ);
        return result;
}
