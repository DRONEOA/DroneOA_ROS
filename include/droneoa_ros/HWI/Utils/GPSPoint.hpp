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

#ifndef HWI_GPSPOINT_HPP_  // NOLINT
#define HWI_GPSPOINT_HPP_  // NOLINT

#include <string>
#include <droneoa_ros/HWI/Utils/Position3D.hpp>

/**
 * @brief Reprsentation of local GPS point
 */

class GPSPoint : public Position3D {
 public:
    GPSPoint();
    GPSPoint(float latitude, float longitude, float altitude);
    /**
     * @brief Compare whether 2 GPSPoint are equal.
     * @param other The other point
     * @return true if the distance between is less than or equal to GPS_COMPARE_DIFF_MAX
     * @return false if the distance between is greater than GPS_COMPARE_DIFF_MAX
     */
    bool operator==(const Position3D& other) const;
    std::string AsString() const override;
};

#endif  // HWI_GPSPOINT_HPP_  // NOLINT
