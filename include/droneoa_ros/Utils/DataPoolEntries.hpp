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
 * Written by Bohan Shi <b34shi@uwaterloo.ca>, Oct 2020
 */

#ifndef UTILS_DATAPOOL_ENTRIES_  // NOLINT
#define UTILS_DATAPOOL_ENTRIES_  // NOLINT

#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>

#include <string>
#include <map>
#include <typeindex>
#include <boost/any.hpp>

namespace DP {

/*******************************************************************************
 * Record supported data types
 */
enum SUPPORT_TYPES {
    TYPE_BOOL,
    TYPE_STRING,
    TYPE_INT32,
    TYPE_UINT32,
    TYPE_FLOAT32,
    TYPE_GEO_MSGS_QUAT,
    TYPE_GEO_MSGS_VEC3
};

static const char* SUPPORT_TYPES_NAME[] {
    "BOOL_T",
    "STRING_T",
    "INT32_T",
    "UINT32_T",
    "FLOAT32_T",
    "GEO_QUAT_T",
    "GEO_VEC3_T"
};

/*******************************************************************************
 * Record supported entry types
 */
enum ENTRY_TYPES {
    NONE,
    DATA,
    CONFIG,
    ALL
};

/*******************************************************************************
 * Data Entries Definition
 */
// Basic Flight Info
constexpr char DP_IS_ARMED[] = "IsArmed";  /**< @brief Type: bool */

constexpr char DP_IS_CONNECTED[] = "IsConnected";  /**< @brief Type: bool */

constexpr char DP_IS_GUIDED[] = "IsGuided";  /**< @brief Type: bool */

constexpr char DP_FLIGHT_MOD[] = "FlightMod";  /**< @brief Type: string */

constexpr char DP_SYS_STATUS[] = "SystemStatus";  /**< @brief Type: uint8_t */

constexpr char DP_RELATIVE_ALTITUDE[] = "RelativeAltitude";  /**< @brief Type: float */

constexpr char DP_HUD_ALTITUDE[] = "HUDAltitude";  /**< @brief Type: float */

constexpr char DP_BATTERY_VOLTAGE[] = "BatteryVoltage";  /**< @brief Type: float */

constexpr char DP_HEADING[] = "Heading";  /**< @brief Type: float */

constexpr char DP_AIR_SPEED[] = "AirSpeed";  /**< @brief Type: float */

constexpr char DP_GROUND_SPEED[] = "GroundSpeed";  /**< @brief Type: float */

constexpr char DP_CLIMB_Rate[] = "ClimbRate";  /**< @brief Type: float */

constexpr char DP_THROTTLE[] = "Throttle";  /**< @brief Type: float */

constexpr char DP_ORIENTATION_QUAT[] = "OrientationQuaternion";  /**< @brief Type: geometry_msgs::Quaternion */

constexpr char DP_ORIENTATION_RPY[] = "OrientationRollPitchYaw";  /**< @brief Type: geometry_msgs::Vector3 */

// Location
constexpr char DP_GPS_LOC[] = "GPSLocal";  /**< @brief Type: geometry_msgs::Vector3 */

constexpr char DP_GPS_HOME[] = "GPSHome";  /**< @brief Type: geometry_msgs::Vector3 */

constexpr char DP_LOCAL_LOC[] = "LocalLocation";  /**< @brief Type: geometry_msgs::Vector3 */

constexpr char DP_CURR_SETPOINT_ENU_TARGET[] = "CurrentSetpointTarget";  /**< @brief Type: geometry_msgs::Vector3 */

// OAC
constexpr char DP_ACTIVE_OAC_LEVEL[] = "ActiveOACLevel";  /**< @brief Type: int32_t */

constexpr char DP_OAC_SWITCH[] = "OACSwitch";  /**< @brief Type: bool */

/*******************************************************************************
 * Config Entries Definition
 */
/**
 * @brief Whether to check GPS fix before initialize
 */
constexpr char CONF_SAFETY_GPS_FIX[] = "GPSFixSafetyCheck";  /**< @brief Type: bool */
constexpr char CONF_VEHICLE_MAX_SPEED_HORIZONTAL[] =
    "VehicleMaxSpeedHorizontal";  /**< @brief Type: int32_t (Recommending maximum frequency to be 1Hz) */
constexpr char CONF_CHECK_CHANGES_FREQUENCY[] = "ConfigCheckChangesFrequency";  /**< @brief Type: float */

/*******************************************************************************
 * Type Mapping For DataPool Data Entries
 *     Detail see: Utils/DataPoolEntries.cpp
 */
extern std::map<std::string, std::type_index> DP_TYPE_MAP;

/*******************************************************************************
 * Type Mapping For DataPool Config Entries
 *     Detail see: Utils/DataPoolEntries.cpp
 */
extern std::map<std::string, std::type_index> CONF_TYPE_MAP;

/*******************************************************************************
 * Default Value Setting For DataPool Entries
 *     Detail see: Utils/DataPoolEntries.cpp
 */
extern std::map<std::string, boost::any> DP_DEFAULT_VALUE_MAP;

}  // namespace DP

#endif  // UTILS_DATAPOOL_ENTRIES_  // NOLINT
