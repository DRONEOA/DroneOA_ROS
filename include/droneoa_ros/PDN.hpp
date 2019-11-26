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

#ifndef INCLUDE_DRONEOA_ROS_PDN_HPP_  // NOLINT
#define INCLUDE_DRONEOA_ROS_PDN_HPP_  // NOLINT

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

// SITL
#define UE4_SITL

// CNC
static const bool ENABLE_SAFETY_GPS = true;

// OA Controller
static const double OAC_REFRESH_FREQ = 20;  // Hz
enum CMD_QUEUE_TYPES {
    CMD_CHMOD = 0,  // param: mod name
    CMD_SET_MAX_VELOCITY  // param: float speed
};

enum DATA_QUEUE_TYPES {
    DATA_CONFIDENCE = 0,  // param: confidence
    DATA_ALG_NAME  // param: name
};

static const char* CMD_QUEUE_TYPES_NAME[] {
    "CMD_CHMOD",
    "CMD_SET_MAX_VELOCITY"
};

static const char* DATA_QUEUE_TYPES_NAME[] {
    "DATA_CONFIDENCE",
    "DATA_ALG_NAME"
};

static const char* ALG_STR_BUG = "ALG_BUG";
static const char* ALG_STR_VFF = "ALG_VFF";
static const char* ALG_STR_COLLISION_LIDAR = "ALG_CA_LIDAR";
static const char* ALG_STR_COLLISION_DEPTH = "ALG_CA_DEPTH";
static const char* ALG_STR_COLLISION_AI = "ALG_CA_AI";

static const int OAC_STAGE_SETTING = 1;  // Stage 1 - 3

// Vehicle Data
static const int VEHICLE_BOUNDBOX_WIDTH = 1200;  // mm
static const int VEHICLE_BOUNDBOX_LENGTH = 1200;  // mm
static const int VEHICLE_BOUNDBOX_HEIGHT = 1000;  // mm
static const int VEHICLE_MAX_SPEED_HORIZONTAL = 12;  // m/s
static const float VEHICLE_MAX_ALTITUDE_RELATIVE = 10;  // m
static const float VEHICLE_MAX_ACCELEATION = 0.4f;  // m/s^2 Placeholder, Need to measure
static const float VEHICLE_MIN_SAFE_DISTANCE = 0.5f;

// Module Enable
static const bool ENABLE_RSC = true;
static const bool ENABLE_LIDAR = true;
static const bool ENABLE_AI = true;
static const bool ENABLE_NGN = true;

// Lidar Setting
static const float LIDAR_ORIENTATION_CW = 180;
static const float LIDAR_FILTER_LOW = 0.1;  // in m
static const float LIDAR_FILTER_HIGH = 10;  // in m

#endif  // NOLINT
