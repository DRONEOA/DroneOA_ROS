/* Copyright (C) 2020 DroneOA Group - All Rights Reserved
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
 * Written by Bohan Shi <b34shi@uwaterloo.ca>, Nov 2020
 */

#include <ros/ros.h>
#include <readline/readline.h>
#include <readline/history.h>

#include <droneoa_ros/ConsoleService/ConsoleService.hpp>

int main(int argc, char **argv) {
    ros::init(argc, argv, "droneoa_ros_console_server");
    ros::NodeHandle n;
    ros::Rate r(GLOBAL_ROS_RATE);
    ConsoleService::ConsoleService consoleServiceHandler;
    ROS_INFO("[Console Service] Ready to process console inputs");

    char* buf = nullptr;
    while (n.ok() && ((buf = readline("")) != nullptr)) {
        if (strlen(buf) > 0) {
            add_history(buf);
        }
        consoleServiceHandler.handleConsoleInput(std::string(buf));
        free(buf);
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}
