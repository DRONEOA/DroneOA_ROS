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
 * Written by Bohan Shi <b34shi@edu.uwaterloo.ca>, January 2020
 */

#ifndef INCLUDE_DRONEOA_ROS_OAC_COMMAND_HPP_  // NOLINT
#define INCLUDE_DRONEOA_ROS_OAC_COMMAND_HPP_  // NOLINT

#include <utility>
#include <vector>
#include <string>
#include <droneoa_ros/CNCInterface.hpp>

/*
 * Command
 */

enum CMD_QUEUE_TYPES {
    CMD_CHMOD = 0,  // @param: mod name
    CMD_SET_MAX_VELOCITY,  // @param: float speed
    CMD_DELAY_MSEC  // @param: uint32 time in ms
};

static const char* CMD_QUEUE_TYPES_NAME[] {
    "CMD_CHMOD",
    "CMD_SET_MAX_VELOCITY",
    "CMD_DELAY_MSEC"
};

typedef std::pair<CMD_QUEUE_TYPES, std::string> CommandLine;
typedef std::vector<CommandLine> CommandQueue;

/*
 * Data
 */

enum DATA_QUEUE_TYPES {
    DATA_CONFIDENCE = 0,  // param: confidence
    DATA_ALG_NAME  // param: name
};

static const char* DATA_QUEUE_TYPES_NAME[] {
    "DATA_CONFIDENCE",
    "DATA_ALG_NAME"
};

typedef std::pair<DATA_QUEUE_TYPES, std::string> DataLine;
typedef std::vector<DataLine> DataQueue;

/*
 * Generic Parser
 */

/*
 * Parse each single line of command
 * - Input: command line (std::pair<CMD_QUEUE_TYPES, std::string>)
 * - Output: whether operation is succesuful
 */
bool parseCMD(CNCInterface *cnc, const CommandLine& cmdline);

#endif  // INCLUDE_DRONEOA_ROS_OAC_COMMAND_HPP_  // NOLINT
