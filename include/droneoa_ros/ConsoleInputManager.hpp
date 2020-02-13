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
 * Written by Bohan Shi <b34shi@edu.uwaterloo.ca>, Feb. 2020
 */

#ifndef INCLUDE_DRONEOA_ROS_CONSOLEINPUTMANAGER_HPP_  // NOLINT
#define INCLUDE_DRONEOA_ROS_CONSOLEINPUTMANAGER_HPP_  // NOLINT

#include <vector>
#include <string>
#include <utility>

#include <droneoa_ros/CNCInterface.hpp>
#include <droneoa_ros/RSCInterface.hpp>
#include <droneoa_ros/OAController.hpp>
#include <droneoa_ros/LidarInterface.hpp>

static const char ConsoleDelimiter = ' ';

class ConsoleInputManager {
 public:
    explicit ConsoleInputManager(bool* masterSwitch);
    bool init(CNCInterface* cnc, RSCInterface *rsc, OAController *oac, LidarInterface *lidar);
    bool parseAndExecuteConsole(std::string cmd);

 private:
    std::pair<std::string, std::vector<std::string>> currentCommand_;
    bool* masterSwitch_;

    // Pointer to controller
    CNCInterface *cnc_;
    RSCInterface *rsc_;
    OAController *oac_;
    LidarInterface *lidar_;

    // Private Handlers
    bool splitModuleCommand(std::string cmd);
    bool commandDispatch();

    // Module Handlers
    bool handleCNCCommands();
    bool handleOACCommands();
    bool handleRSCCommands();
    bool handleLIDARCommands();
    bool handleQuickCommands();

    // Helper
    void printFormatHelper();
    void printModuleHelper();
    void printCNCHelper();
    void printRSCHelper();
    void printOACHelper();
    void printLIDARHelper();
    void printQuickHelper();
};

#endif  // INCLUDE_DRONEOA_ROS_CONSOLEINPUTMANAGER_HPP_  // NOLINT
