/* Copyright (C) 2020 DroneOA Group - All Rights Reserved
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
 * Written by Bohan Shi <b34shi@uwaterloo.ca>, Dec 2020
 */

#include <droneoa_ros/Utils/DataPoolEntries.hpp>

namespace DP {

/*******************************************************************************
 * Type Mapping For DataPool Data Entries
 */
std::map<std::string, std::type_index> DP_TYPE_MAP = {
    //! @note Data Entries Start
    {DP_IS_ARMED, typeid(bool)},  // NOLINT
    {DP_IS_CONNECTED, typeid(bool)},  // NOLINT
    {DP_IS_GUIDED, typeid(bool)},  // NOLINT
    {DP_FLIGHT_MOD, typeid(std::string)},  // NOLINT
    {DP_SYS_STATUS, typeid(uint32_t)},  // NOLINT
    {DP_RELATIVE_ALTITUDE, typeid(float)},  // NOLINT
    {DP_HUD_ALTITUDE, typeid(float)},  // NOLINT
    {DP_BATTERY_VOLTAGE, typeid(float)},  // NOLINT
    {DP_HEADING, typeid(float)},  // NOLINT
    {DP_AIR_SPEED, typeid(float)},  // NOLINT
    {DP_GROUND_SPEED, typeid(float)},  // NOLINT
    {DP_CLIMB_Rate, typeid(float)},  // NOLINT
    {DP_THROTTLE, typeid(float)},  // NOLINT
    {DP_ORIENTATION_QUAT, typeid(geometry_msgs::Quaternion)},  // NOLINT
    {DP_ORIENTATION_RPY, typeid(geometry_msgs::Vector3)},  // NOLINT
    {DP_GPS_LOC, typeid(geometry_msgs::Vector3)},  // NOLINT
    {DP_GPS_HOME, typeid(geometry_msgs::Vector3)},  // NOLINT
    {DP_LOCAL_LOC, typeid(geometry_msgs::Vector3)},  // NOLINT
    {DP_CURR_SETPOINT_ENU_TARGET, typeid(geometry_msgs::Vector3)},  // NOLINT
    {DP_ACTIVE_OAC_LEVEL, typeid(int32_t)},  // NOLINT
};

/*******************************************************************************
 * Type Mapping For DataPool Config Entries
 */
std::map<std::string, std::type_index> CONF_TYPE_MAP = {
    //! @note Config Entries Start
    {CONF_SAFETY_GPS_FIX, typeid(bool)},  // NOLINT
    {CONF_VEHICLE_MAX_SPEED_HORIZONTAL, typeid(int32_t)},  // NOLINT
    {CONF_CHECK_CHANGES_FREQUENCY, typeid(float)}  // NOLINT
};

/*******************************************************************************
 * Default Value Setting For DataPool Entries
 */
std::map<std::string, boost::any> DP_DEFAULT_VALUE_MAP = {
    {CONF_SAFETY_GPS_FIX, (bool)true},  // NOLINT
    {DP_OAC_SWITCH, (bool)false},  // NOLINT
    {DP_ACTIVE_OAC_LEVEL, (int32_t)0},  // NOLINT
    {CONF_VEHICLE_MAX_SPEED_HORIZONTAL, (int32_t)12},  // NOLINT
    {CONF_CHECK_CHANGES_FREQUENCY, (float)0.1}  // NOLINT
};

}  // namespace DP
