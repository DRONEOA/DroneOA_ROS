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

#include <ros/ros.h>
#include <droneoa_ros/OAC/CMDParser.hpp>
#include <droneoa_ros/OAC/OAC.hpp>

namespace OAC {

CMDParser::CMDParser(CNC::CNCInterface *cnc, CMDRunner *runner) : mpCNC(cnc), cmdRunner_(runner) {}

CMDParser::~CMDParser() {
    ROS_INFO("Destroy CMDParser");
}

/*
 * Parse a queue of commands in order
 * - Input: command queue
 * - Output: whether operation is succesuful
 */
bool CMDParser::parseCMDQueue(const Command::CommandQueue& cmdqueue, bool isFromOAC) {
    if (cmdqueue.size() == 0) {
        return true;
    }
    bool isInstant = true;
    for (auto tmpLine : cmdqueue) {
        if (tmpLine.first == Command::CMD_QUEUE_TYPES::CMD_DELAY_MSEC) {
            isInstant = false;
            break;
        }
        if (tmpLine.first == Command::CMD_QUEUE_TYPES::CMD_UNTIL) {
            isInstant = false;
            break;
        }
        //! @todo Merge continuous move wp command to single move wp list command
    }
    std::cout << "parseCMDQueue isInstant: " << isInstant << std::endl;
    if (isInstant) {
        for (auto cmdline : cmdqueue) {
            if (!Command::parseCMD(mpCNC, cmdline, isFromOAC)) {
                ROS_ERROR("[CMD PARSER] Queue Parser Terminated With ERROR !!!");
                return false;
            }
        }
    } else {
        if (!isFromOAC && ACTIVE_OAC_LEVEL > 1) {
            ROS_ERROR("This Command Queue Is NOT Supported When Obstacle Avoidance Is ON !!!");
            return false;
        }
        //! @todo(shibohan) should we wait for previous queue to finish in some cases?
        cmdRunner_->setupRunner(cmdqueue);
    }
    return true;
}

}  // namespace OAC
