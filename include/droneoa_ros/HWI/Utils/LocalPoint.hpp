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

#ifndef HWI_LOCALPOINT_HPP_  // NOLINT
#define HWI_LOCALPOINT_HPP_  // NOLINT

#include <string>
#include <droneoa_ros/HWI/Utils/Position3D.hpp>

/**
 * @brief Reprsentation of local ENU point
 */
struct LocalPoint : public Position3D {
    LocalPoint();
    LocalPoint(double x, double y, double z);
    std::string AsString() const override;
    static float getDistBetweenPos3D(const LocalPoint &pos1, const LocalPoint& pos2);
};

#endif  // HWI_LOCALPOINT_HPP_  // NOLINT
