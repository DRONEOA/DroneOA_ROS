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

#include <string>

namespace DP {

/*******************************************************************************
 * Record supported data types
 */
enum SUPPORT_TYPES {
    TYPE_BOOL,
    TYPE_STRING,
    TYPE_UINT8,
    TYPE_INT16,
    TYPE_FLOAT,
    TYPE_POSITION3D,  // GPS & Local
    TYPE_GEO_MSGS_QUAT,
    TYPE_GEO_MSGS_VEC3
};

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
constexpr char DP_GPS_LOC[] = "GPSLocal";  /**< @brief Type: GPSPoint */

constexpr char DP_GPS_HOME[] = "GPSHome";  /**< @brief Type: GPSPoint */

constexpr char DP_LOCAL_LOC[] = "LocalLocation";  /**< @brief Type: LocalPoint */

constexpr char DP_CURR_SETPOINT_ENU_TARGET[] = "CurrentSetpointTarget";  /**< @brief Type: LocalPoint */

/*******************************************************************************
 * Config Entries Definition
 */
/**
 * @brief Whether to check GPS fix before initialize
 */
constexpr char CONF_SAFETY_GPS_FIX[] = "GPSFixSafetyCheck";  /**< @brief Type: Bbool */

}  // namespace DP

#endif  // UTILS_DATAPOOL_ENTRIES_  // NOLINT
