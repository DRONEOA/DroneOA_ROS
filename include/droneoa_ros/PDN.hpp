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

// OA Controller
static const double OAC_REFRESH_FREQ = 20;  /**< @brief unit: Hz */

static const char* ALG_STR_BUG = "ALG_BUG";
static const char* ALG_STR_VFF = "ALG_VFF";
static const char* ALG_STR_FGM = "ALG_FGM";
static const char* ALG_STR_VISION = "ALG_VISION";
static const char* ALG_STR_COLLISION_LIDAR = "ALG_CA_LIDAR";
static const char* ALG_STR_COLLISION_DEPTH = "ALG_CA_DEPTH";
static const char* ALG_STR_AI = "ALG_AI";
static const char* ALG_STR_SLAM = "ALG_SLAM";

/**
 * @brief Enabled feature stage 1-3
 */
static const int32_t OAC_STAGE_SETTING = 1;

// Vehicle Data
static const int32_t VEHICLE_BOUNDBOX_WIDTH = 1200;  /**< @brief unit: mm */
static const int32_t VEHICLE_BOUNDBOX_LENGTH = 1200;  /**< @brief unit: mm */
static const int32_t VEHICLE_BOUNDBOX_HEIGHT = 1000;  /**< @brief unit: mm */
static const int32_t VEHICLE_MAX_SPEED_HORIZONTAL = 12;  /**< @brief unit: m/s */
static const float VEHICLE_MAX_ALTITUDE_RELATIVE = 10;  /**< @brief unit: m */
static const float VEHICLE_MAX_ACCELEATION = 0.4f;  /**< @brief unit: m/s^2 */
static const float VEHICLE_MIN_SAFE_DISTANCE = 1.0f;  /**< @brief unit: m */

// Module Enable
static const bool ENABLE_RSC = true;  /**< @brief enable realsense camera */
static const bool ENABLE_LIDAR = true;  /**< @brief enable lidar scanner */
static const bool ENABLE_AI = true;  /**< @brief enable AI module */
static const bool ENABLE_NGN = true;  /**< @brief enable Non-GPS Navigation */

// Lidar Setting
static const float LIDAR_ORIENTATION_CW = 180;  /**< @brief unit: degree */
static const float LIDAR_FILTER_LOW = 0.1;  /**< @brief unit: m */
static const float LIDAR_FILTER_HIGH = 10;  /**< @brief unit: m */

static const float UE4_SITL_SCALE = 1000.0f;

#endif  // INCLUDE_DRONEOA_ROS_PDN_HPP_  // NOLINT
