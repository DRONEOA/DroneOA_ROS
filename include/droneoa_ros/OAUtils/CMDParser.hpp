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

#ifndef INCLUDE_DRONEOA_ROS_CMDPARSER_HPP_  // NOLINT
#define INCLUDE_DRONEOA_ROS_CMDPARSER_HPP_  // NOLINT

#include <utility>
#include <vector>
#include <string>
#include "droneoa_ros/CNCInterface.hpp"

class CMDParser {
    typedef std::pair<CMD_QUEUE_TYPES, std::string> CommandLine;
    typedef std::vector<CommandLine> CommandQueue;

    CNCInterface *cnc_;
    bool parseCMD(const CommandLine& cmdline);
 public:
    explicit CMDParser(CNCInterface *cnc);
    bool parseCMDQueue(const CommandQueue& cmdqueue);
};

#endif  // NOLINT
