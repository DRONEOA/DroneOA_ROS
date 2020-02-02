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

CMDParser::CMDParser(CNCInterface *cnc, CMDRunner *runner) : cnc_(cnc), cmdRunner_(runner) {}

CMDParser::~CMDParser() {
    if (cmdRunner_) {
        delete cmdRunner_;
    }
}

/*
 * Parse a queue of commands in order
 * - Input: command queue
 * - Output: whether operation is succesuful
 * - TODO: support timed commands (currently all commands are sent with no delay or complete check)
 */
bool CMDParser::parseCMDQueue(const CommandQueue& cmdqueue, bool isInstant) {
    if (isInstant) {
        for (auto cmdline : cmdqueue) {
            if (!parseCMD(cnc_, cmdline)) {
                ROS_ERROR("[CMD PARSER] Queue Parser Terminated With ERROR !!!");
                return false;
            }
        }
    } else {
        //! @todo(shibohan) should we wait for previous queue to finish in some cases?
        cmdRunner_->setupRunner(cmdqueue);
    }
    return true;
}
