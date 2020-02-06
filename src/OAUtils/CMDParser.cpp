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
 * Written by Bohan Shi <b34shi@edu.uwaterloo.ca>, November 2019
 */

#include "droneoa_ros/OAUtils/CMDParser.hpp"

CMDParser::CMDParser(CNCInterface *cnc) : cnc_(cnc) {}

/*
 * Parse each single line of command
 * - Input: command line (std::pair<CMD_QUEUE_TYPES, std::string>)
 * - Output: whether operation is succesuful
 */
bool CMDParser::parseCMD(const CommandLine& cmdline) {
    try {
        switch (cmdline.first) {
            case CMD_QUEUE_TYPES::CMD_CHMOD:
            {
                if (cnc_->getMode() == cmdline.second) return true;
                return cnc_->setMode(cmdline.second);
            }
            case CMD_QUEUE_TYPES::CMD_SET_MAX_VELOCITY:
            {
                float targetSpeed = std::stof(cmdline.second);
                targetSpeed = targetSpeed >= 0 ? targetSpeed : 0;
                return cnc_->setMaxSpeed(1, targetSpeed, 0);
            }
            default:
                throw 1;
        }
    } catch (...) {
        ROS_ERROR("[CMD PARSER] Invalid Command or arguments: %s, %s", CMD_QUEUE_TYPES_NAME[cmdline.first],
                cmdline.second.c_str());
        return false;
    }
}

/*
 * Parse a queue of commands in order
 * - Input: command queue
 * - Output: whether operation is succesuful
 * - TODO: support timed commands (currently all commands are sent with no delay or complete check)
 */
bool CMDParser::parseCMDQueue(const CommandQueue& cmdqueue) {
    for (auto cmdline : cmdqueue) {
        if (!parseCMD(cmdline)) {
            ROS_ERROR("[CMD PARSER] Queue Parser Terminated With ERROR !!!");
            return false;
        }
    }
    return true;
}
