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

/**
 * @file Command.hpp
 * @author DroneOA (Bohan Shi)
 * @brief Define Command Related Types And Generic Parsers
 * @version 1.0
 * @date 2020-02
 */

#ifndef OAC_COMMAND_HPP_  // NOLINT
#define OAC_COMMAND_HPP_  // NOLINT

#include <utility>
#include <vector>
#include <string>
#include <droneoa_ros/HWI/interface/CNCInterface.hpp>
#include <droneoa_ros/PDN.hpp>

namespace Command {

constexpr char CommandDataDelimiter = ' ';

/**
 * @brief ENUM of supported COMMAND types
 */
enum CMD_QUEUE_TYPES : uint32_t {
    CMD_CHMOD = 0,  /*!< DATA: mod name */
    CMD_ARM,  /*!< DATA: none */
    CMD_TAKEOFF,  /*!< DATA: float altitude */
    CMD_LAND,  /*!< DATA: none */
    CMD_SET_MAX_VELOCITY,  /*!< DATA: float speed */
    CMD_SET_YAW,  /*!< DATA: float heading */
    CMD_DELAY_MSEC,  /*!< DATA: uint32 time in ms */
    CMD_GOTO_RELATIVE,  /*!< DATA: float North axis dist, float East axis dist, float Altitude */
    CMD_GOTO_GLOBAL,  /*!< DATA: float Latitude, float Longitude, float Altitude */
    CMD_GOTO_HEADING,  /*!< DATA: float heading, float distance, float Altitude */
    CMD_CLIMB,  /*!< DATA: float Delta Altitude */
    CMD_DESCEND,  /*!< DATA: float Delta Altitude */
    CMD_UNTIL,  /*!< DATA: mode (arrwp: arrive at way point; clrwp: clear all waypoints; More to be added) */
};

/**
 * @brief ENUM of supported COMMAND types's name
 */
static const char* CMD_QUEUE_TYPES_NAME[] {
    "CMD_CHMOD",
    "CMD_ARM",
    "CMD_TAKEOFF",
    "CMD_LAND",
    "CMD_SET_MAX_VELOCITY",
    "CMD_SET_YAW",
    "CMD_DELAY_MSEC",
    "CMD_GOTO_RELATIVE",
    "CMD_GOTO_GLOBAL",
    "CMD_GOTO_HEADING",
    "CMD_CLIMB",
    "CMD_DESCEND",
    "CMD_UNTIL",
};

/**
 * @brief Represent a line of command
 * pair.first is the type of the command
 * pair.second is the data requied by the command
 */
typedef std::pair<CMD_QUEUE_TYPES, std::string> CommandLine;
/**
 * @brief Represent a queue of command in time order
 * a vector of commandlines
 */
typedef std::vector<CommandLine> CommandQueue;

/**
 * @brief ENUM of supported DATA types
 */
enum DATA_QUEUE_TYPES : uint32_t {
    DATA_CONFIDENCE = 0,  /*!< DATA: confidence */
    DATA_ALG_NAME,  /*!< DATA: algorithm name */
};

/**
 * @brief ENUM of supported DATA types's name
 */
static const char* DATA_QUEUE_TYPES_NAME[] {
    "DATA_CONFIDENCE",
    "DATA_ALG_NAME"
};

/**
 * @brief Represent a line of data
 * pair.first is the type of the data
 * pair.second is the data content
 */
typedef std::pair<DATA_QUEUE_TYPES, std::string> DataLine;
/**
 * @brief Represent a queue of data
 * a vector of datalines
 */
typedef std::vector<DataLine> DataQueue;

std::vector<std::string> getDataListFromString(std::string data);

/**
 * @brief Generic Parser for a single line of command
 * @param cnc the pointer to the shared command and control module
 * @param cmdline the source CommandLine
 * @return whether the operation is successful
 */
bool parseCMD(CNC::CNCInterface *cnc, const CommandLine& cmdline, bool isFromOAC = false);

}  // namespace Command

#endif  // OAC_COMMAND_HPP_  // NOLINT
