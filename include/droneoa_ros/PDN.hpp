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

/**
 * @file PDN.hpp
 * @author DroneOA (Bohan Shi)
 * @brief Pre-defined Names And Values
 * @version 1.0
 * @date 2019-08
 */

#ifndef INCLUDE_DRONEOA_ROS_PDN_HPP_  // NOLINT
#define INCLUDE_DRONEOA_ROS_PDN_HPP_  // NOLINT

#include <cstdint>

/**
 * @brief ROS Rate
 */
static const double GLOBAL_ROS_RATE = 20;

/**
 * @brief List of FCU Type
 */
static const char* FCU_PX4 = "PX4";
static const char* FCU_ARDUPILOT = "ARDUPILOT";
static const char* CURRENT_FCU_TYPE = FCU_ARDUPILOT;

/**
 * @brief List of flight modes Ardupilot
 */
static const char* FLT_MODE_OFFBOARDD = "OFFBOARD";

/**
 * @brief List of flight modes Ardupilot
 */
static const char* FLT_MODE_STABILIZE = "STABILIZE";
static const char* FLT_MODE_ACRO = "ACRO";
static const char* FLT_MODE_ALT_HOLD = "ALT_HOLD";
static const char* FLT_MODE_AUTO = "AUTO";
static const char* FLT_MODE_GUIDED = "GUIDED";
static const char* FLT_MODE_LOITER = "LOITER";
static const char* FLT_MODE_RTL = "RTL";
static const char* FLT_MODE_CIRCLE = "CIRCLE";
static const char* FLT_MODE_LAND = "LAND";
static const char* FLT_MODE_OF_LOITER = "OF_LOITER";
static const char* FLT_MODE_AUTOTUNE = "AUTOTUNE";
static const char* FLT_MODE_POSHOLD = "POSHOLD";
static const char* FLT_MODE_BRAKE = "BRAKE";
static const char* FLT_MODE_AVOID_ADSB = "AVOID_ADSB";
static const char* FLT_MODE_GUIDED_NOGPS = "GUIDED_NOGPS";

/**
 * @brief Whether to check GPS fix before initialize
 */
static const bool ENABLE_SAFETY_GPS = true;

/**
 * @brief True if want to use GPS global position for move relative and heading
 */
static const bool ENABLE_FORCE_GPS_ON_RELATIVE_MOVE = false;

// OA Controller
#define ENABLE_POINTCLOUD
static const double OAC_REFRESH_FREQ = 20;  /**< @brief unit: Hz suggest to be same as GLOBAL_ROS_RATE */
static const bool OAC_USE_SETPOINT_ENU = true;  /**< @brief true to use local ENU [Recommend]; false to use GPS */
static const bool OAC_CUMULATIVE_WAYPOINT = false;  /**< @brief if true, new waypoint will not overtake previous */

/**
 * @brief Enabled feature stage 1-3
 * 1: COllision Avoidance
 * 2: Obstacle Avoidance (Local Path Planning)
 * 3: Obstacle Avoidance (Global Path Planning)
 */
static const int32_t OAC_STAGE_SETTING = 3;

// Vehicle Data
static const int32_t VEHICLE_BOUNDBOX_WIDTH = 1000;  /**< @brief unit: mm */
static const int32_t VEHICLE_BOUNDBOX_LENGTH = 1000;  /**< @brief unit: mm */
static const int32_t VEHICLE_BOUNDBOX_HEIGHT = 600;  /**< @brief unit: mm */
static const int32_t VEHICLE_MAX_SPEED_HORIZONTAL = 12;  /**< @brief unit: m/s */
static const float VEHICLE_MAX_ALTITUDE_RELATIVE = 10;  /**< @brief unit: m */
static const float VEHICLE_MAX_ACCELEATION = 0.4f;  /**< @brief unit: m/s^2 */
static const float VEHICLE_MIN_SAFE_DISTANCE = 1.0f;  /**< @brief unit: m */

// Module Enable
static const bool ENABLE_RSC = true;  /**< @brief enable realsense camera */
static const bool ENABLE_LIDAR = true;  /**< @brief enable lidar scanner */
static const bool ENABLE_AI = true;  /**< @brief enable AI module */
static const bool ENABLE_NGN = true;  /**< @brief enable Non-GPS Navigation */
static const bool ENABLE_OCTOMAP = true;

// Lidar Setting
static const float LIDAR_ORIENTATION_CW = 180;  /**< @brief unit: degree */
static const float LIDAR_FILTER_LOW = 0.1;  /**< @brief unit: m */
static const float LIDAR_FILTER_HIGH = 10;  /**< @brief unit: m */
static const int32_t LIDAR_POPUP_SCALE = 50;
// Lidar sources
static const char* LIDAR_SOURCE_YDLIDAR = "/scan";
static const char* LIDAR_SOURCE_UE4 = "/sitl_lidar_test";

// Depth setting
static const float DEPTH_MAX_RANGE = 10.0f;  /**< @brief unit: m */
static const float DEPTH_MIN_RANGE = 0.1f;  /**< @brief unit: m */
// Depth sources
static const char* DEPTH_SOURCE_RSC = "/d435/depth/image_rect_raw";
static const char* DEPTH_SOURCE_UE4 = "/unreal_ros/image_depth";
static const char* PC_SOURCE_RSC = "/d435/depth/color/points";
static const char* PC_SOURCE_UE4 = "/depth_registered/points";

// Runner
static const int32_t RUNNER_TIMEOUT_LIMIT = 100000;  /**< @brief unit: msec */

// Max Error
static const double GPS_COMPARE_DIFF_MAX = 0.5;  /**< @brief max distance consider to be the same GPS point unit: m */
static const double ALT_COMPARE_DIFF_MAX = 0.1;  /**< @brief max distance consider to be the same Altitude unit: m */
static const double POS3D_COMPARE_DIFF_MAX = 0.2;  /**< @brief max diff consider to be the same position 3d unit: m */

// SITL
static const float UE4_SITL_SCALE = 1000.0f;

// RRT
// #define RRT_ENABLE_SMOOTHER
static const double RRT_MAX_PLANNING_DISTANCE = 10.0f;
static const double RRT_MAX_PLANNING_HEIGHT = VEHICLE_MAX_ALTITUDE_RELATIVE;
static const double RRT_MIN_PLANNING_HEIGHT = 0.0f;
static const float RRT_MIN_GOAL_HEIGHT = 1.0f;
static const float RRT_MAX_SPEED = 0.2f;
static const float OCTOMAP_RESOLUTION = 0.2f;

// Debug Feature Control
#define DEBUG_CNC_POPUP
// #define DEBUG_DEPTH_IMG_POPUP
// #define DEBUG_PCL_VIEWER  // Enable image popup first
// #define DEBUG_LIDAR_POPUP
// #define DEBUG_OAC
// #define DEBUG_ALG_COLLISION_LIDAR
// #define DEBUG_ALG_COLLISION_DEPTH
// #define DEBUG_ALG_OBSTACLE_FGM

#endif  // INCLUDE_DRONEOA_ROS_PDN_HPP_  // NOLINT
