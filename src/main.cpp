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

#include <signal.h>
#include <ros/ros.h>
#include <readline/readline.h>
#include <readline/history.h>

#include <cstdlib>
#include <iomanip>
#include <sstream>

#include <droneoa_ros/CNCInterface.hpp>
#include <droneoa_ros/RSCInterface.hpp>
#include <droneoa_ros/LidarInterface.hpp>
#include <droneoa_ros/OAController.hpp>
#include <droneoa_ros/Utils/CNCUtils.hpp>
#include <droneoa_ros/ConsoleInputManager.hpp>

void sysSignalhandler(int signum) {
    ROS_WARN("Caught signal %d", signum);
    // Terminate program
    exit(signum);
}

int main(int argc, char **argv) {
    // Setup Refresh Rate
    int32_t rate = 10;

    ros::init(argc, argv, "mavros_takeoff");
    ros::NodeHandle n;

    ros::Rate r(rate);

    // Register exit signal
    signal(SIGINT, sysSignalhandler);

    // Interface Instance
    CNCInterface cnc;
    RSCInterface rsc;
    LidarInterface lidar;
    cnc.init(n, r);
    if (ENABLE_RSC) {
        rsc.init(n, r);
    }
    if (ENABLE_LIDAR) {
        lidar.init(n, r);
    }

    // Create CMD Queue Runner
    CMDRunner runner(&cnc);

    // Create OA Controller
    OAController oac(&cnc, &lidar, &rsc, &runner, ros::Rate(OAC_REFRESH_FREQ));

    // Process Console Commands
    char *commandIn;
    bool masterSW = true;
    ConsoleInputManager consoleInputManager(&masterSW);
    consoleInputManager.init(&cnc, &rsc, &oac, &lidar);
    while ((commandIn = readline("")) != nullptr) {
        if (*commandIn) {
            add_history(commandIn);
            std::string sCommandIn(commandIn);
            consoleInputManager.parseAndExecuteConsole(sCommandIn);
        }
        free(commandIn);
        if (!masterSW) {  // Quit Signal
            break;
        }
    }

    while (n.ok() && masterSW) {
      ros::spinOnce();
      r.sleep();
    }

    return 0;
}
