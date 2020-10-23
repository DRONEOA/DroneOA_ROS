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
 */

#include <signal.h>

#include <ros/ros.h>
#include <readline/readline.h>
#include <readline/history.h>

#include <cstdlib>
#include <iomanip>
#include <sstream>

#include <droneoa_ros/HWI/CNCArdupilot.hpp>
#include <droneoa_ros/HWI/LidarYDLidar.hpp>
#include <droneoa_ros/HWI/RSC.hpp>
#include <droneoa_ros/OAC/OAC.hpp>
#include <droneoa_ros/HWI/ConsoleInputManager.hpp>
#include <droneoa_ros/GUI/Debug/LidarPopup.hpp>
#include <droneoa_ros/GUI/Debug/RSCPopup.hpp>
#include <droneoa_ros/GUI/Debug/CNCPopup.hpp>
#include <droneoa_ros/GUI/Release/WebGUIServer.hpp>

void sysSignalhandler(int signum) {
    ROS_WARN("Caught signal %d", signum);
    // Terminate program
    ros::shutdown();
    exit(signum);
}

int main(int argc, char **argv) {
    // Setup Refresh Rate
    ros::init(argc, argv, "droneoa");
    ros::NodeHandle node;
    ros::Rate rate(GLOBAL_ROS_RATE);

    // Register exit signal
    signal(SIGINT, sysSignalhandler);

    // HWI Components
    CNC::CNCArdupilot cnc(node, rate);
    Lidar::LidarYDLidar lidar(node, rate);
    Depth::RSC rsc(node, rate);

    // OAC Components
    OAC::CMDRunner runner(&cnc);
    OAC::OAController oac(&cnc, &lidar, &rsc, &runner, ros::Rate(OAC_REFRESH_FREQ));

    // GUIs
    GUI::WebGUIServer webGUI = GUI::WebGUIServer("Default Session", &cnc);
    #ifdef DEBUG_CNC_POPUP
    GUI::CNCPopup cp = GUI::CNCPopup("CNC Debug Popup", &cnc);
    #endif
    #ifdef DEBUG_LIDAR_POPUP
    GUI::LidarPopup lp = GUI::LidarPopup("Lidar Debug Popup", &lidar, &cnc);
    #endif
    #ifdef DEBUG_DEPTH_IMG_POPUP
        #ifdef DEBUG_PCL_VIEWER
        GUI::RSCPopup dp = GUI::RSCPopup("RSC Debug Popup", &rsc, true);
        #else
        GUI::RSCPopup dp = GUI::RSCPopup("RSC Debug Popup", &rsc, false);
        #endif
    #endif

    // Console IO
    char *commandIn;
    bool masterSW = true;
    IO::ConsoleInputManager consoleInputManager(&masterSW);
    consoleInputManager.init(&cnc, &rsc, &oac, &lidar, &runner);  // @todo Or seperate runner ?
    while ((commandIn = readline("")) != nullptr) {
        if (*commandIn) {
            add_history(commandIn);
            std::string sCommandIn(commandIn);
            consoleInputManager.parseAndExecuteConsole(sCommandIn);
        }
        free(commandIn);
        if (!masterSW) {  // Quit Signal
            ros::shutdown();
            break;
        }
    }

    //! @todo Do we need this
    while (node.ok()) {
      ros::spinOnce();
      rate.sleep();
    }

    return 0;
}
