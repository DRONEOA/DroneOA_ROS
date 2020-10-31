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
#include <ros/package.h>
#include <iostream>
#include <string>

#include <droneoa_ros/PackageManager/PMCommandParser.hpp>

/********************************************************
 * Method
 *      Both main node and PM accept PM inputs
 *          main node forward to PM
 *      Note: PM need a list of node
 *      Note: need a config for each addon, like, whether to start with main
 *      PS: we may want main node to broadcast all unacceped console inputs
 *      PS: make main node module help command only, like when type help. Maybe only notify, not main node command
 *      PS: other node free to pick up if matched, no need to inform main node
 */

int main(int argc, char **argv) {
    // Set up ROS.
    ros::init(argc, argv, "talker");
    ros::NodeHandle n;
    ros::Rate r(10);

    std::string line;
    PM::CommandParser mPM;
    // Main loop.
    while (n.ok() && std::getline(std::cin, line)) {
        mPM.parseInput(line);
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}
