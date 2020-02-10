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

#ifndef INCLUDE_DRONEOA_ROS_OAC_CMDPARSER_HPP_  // NOLINT
#define INCLUDE_DRONEOA_ROS_OAC_CMDPARSER_HPP_  // NOLINT

#include <utility>
#include <vector>
#include <string>
#include <droneoa_ros/CNCInterface.hpp>
#include <droneoa_ros/OAC/Command.hpp>
#include <droneoa_ros/OAC/CMDRunner.hpp>

class CMDParser {
    CNCInterface *cnc_;
    CMDRunner *cmdRunner_;

 public:
    explicit CMDParser(CNCInterface *cnc, CMDRunner *runner);
    virtual ~CMDParser();
    /**
     * @brief Parse and execute the command queue
     * @param cmdqueue a queue of command
     * @param isInstant true if all command can be executed at the same time, false will use the runner
     * @return whether the operation is successful
     */
    bool parseCMDQueue(const CommandQueue& cmdqueue, bool isInstant = true);
};

#endif  // INCLUDE_DRONEOA_ROS_OAC_CMDPARSER_HPP_  // NOLINT
