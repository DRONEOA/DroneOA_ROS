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

#ifndef HWI_POSITION3D_HPP_  // NOLINT
#define HWI_POSITION3D_HPP_  // NOLINT

#include <geometry_msgs/Vector3.h>
#include <string>
/**
 * @brief Reprsentation of any 3D point
 */

struct Position3D {
    Position3D();
    Position3D(double x, double y, double z);
    virtual ~Position3D();
    virtual std::string AsString() const;
    /**
     * @brief Compare whether 2 positions are equal.
     * @param other The other point
     * @return true if the distance between is less than or equal to POS3D_COMPARE_DIFF_MAX
     * @return false if the distance between is greater than POS3D_COMPARE_DIFF_MAX
     */
    virtual bool operator==(const Position3D& other) const;
    virtual float getDistanceTo(const Position3D &other) const;
    virtual geometry_msgs::Vector3 getGeoVec3();
    double mX;
    double mY;
    double mZ;
};

#endif  // HWI_POSITION3D_HPP_  // NOLINT
