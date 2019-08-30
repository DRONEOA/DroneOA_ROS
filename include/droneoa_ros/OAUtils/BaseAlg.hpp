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

#ifndef INCLUDE_DRONEOA_ROS_BASEALG_HPP_  // NOLINT
#define INCLUDE_DRONEOA_ROS_BASEALG_HPP_  // NOLINT

#include <droneoa_ros/CNCInterface.hpp>

class BaseAlg {
 public:
    BaseAlg();
    virtual ~BaseAlg();

    virtual void init(CNCInterface cnc);
    virtual void collect();  // Collect required sensor data
    virtual bool plan();  // Return false when get around is impossible

    virtual float getRelativeTurn();
    virtual float getNewSpeed();
    virtual GPSPoint getNextWaypointRelative();
    virtual GPSPoint getNextWaypointGlobal();
 private:
    CNCInterface cnc_;
};

#endif  // NOLINT