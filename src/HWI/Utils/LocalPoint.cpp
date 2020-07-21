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

#include <droneoa_ros/HWI/Utils/LocalPoint.hpp>
#include <droneoa_ros/PDN.hpp>

LocalPoint::LocalPoint() : Position3D() {}

LocalPoint::LocalPoint(double x, double y, double z) : Position3D(x, y, z) {}

float LocalPoint::getDistBetweenPos3D(const LocalPoint &pos1, const LocalPoint& pos2) {
    float dx = pos1.mX - pos2.mX;
    float dy = pos1.mY - pos2.mY;
    float dz = pos1.mZ - pos2.mZ;
    return sqrt((dx*dx) + (dy*dy) + (dz*dz));
}

std::string LocalPoint::AsString() const {
    std::string result = "LocalPoint: x: " + std::to_string(mX) + " y: " +
                std::to_string(mY) + " z: " + std::to_string(mZ);
    return result;
}
