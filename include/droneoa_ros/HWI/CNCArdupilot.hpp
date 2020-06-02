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

#ifndef HWI_CNCARDUPILOT_HPP_  // NOLINT
#define HWI_CNCARDUPILOT_HPP_  // NOLINT

#include <string>

#include <ros/ros.h>
#include <mavros_msgs/Waypoint.h>
#include <mavros_msgs/WaypointPush.h>
#include <mavros_msgs/WaypointClear.h>

#include <droneoa_ros/HWI/base/CNCGeneric.hpp>

namespace CNC {

class CNCArdupilot : public CNCGeneric{
 public:
    CNCArdupilot(ros::NodeHandle node, ros::Rate rate);
    virtual ~CNCArdupilot();

    /***************************************************************************
     * Mission
     */
    /**
     * @brief Push New Waypoint List
     * @param x_lat
     * @param y_long
     * @param z_alt
     * @param isCurrent (2 = isCurrent Guided)
     * @param command (default NAV_WAYPOINT)
     * @return client send response
     */
    bool pushWaypoints(float x_lat, float y_long, float z_alt, uint8_t isCurrent = 2,
        uint16_t command = mavros_msgs::CommandCode::NAV_WAYPOINT);
    bool clearWaypoint();

    /***************************************************************************
     * User Simple Function
     */
    /**
     * @brief Goto Global Waypoint, 3D GPS Point
     * @param x_lat
     * @param y_long
     * @param z_alt in meter
     * @return client send response
     */
    bool gotoGlobal(float x_lat, float y_long, float z_alt) override;
    /**
     * @brief Goto Relative Waypoint (North+, East+)
     * @param x_lat North+ in meter
     * @param y_long East+ in meter
     * @param z_alt default 10 in meter
     * @param isAltDelta not used
     * @return client send response
     */
    bool gotoRelative(float x_lat, float y_long, float z_alt, bool isAltDelta = false) override;
    /**
     * @brief Goto Target Head
     * @param heading in degree
     * @param distance in meter
     * @param z_alt in meter
     * @return client send response
     */
    bool gotoHeading(float heading, float distance, float z_alt) override;

    /***************************************************************************
     * Public Helper
     */
    /**
     * @brief Check if a flight mode name is valid
     * @param modeName 
     * @return true if name is valid
     */
    bool checkFModeExist(std::string modeName) override;

    /***************************************************************************
     * Accessor
     */
    float getTargetAltitude();
    GPSPoint getTargetWaypoint() override;
    bool isGuided();
};

}  // namespace CNC

#endif  // HWI_CNCARDUPILOT_HPP_  // NOLINT
