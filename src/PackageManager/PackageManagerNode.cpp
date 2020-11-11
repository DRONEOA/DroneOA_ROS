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
#include <droneoa_ros/PDN.hpp>

/*******************************************************************************
 * @brief  Package Manager Node.
 *     Package Manager (PM) is used to manage installed packages and perform
 *     package related operations, including: install, update, uninstall,
 *     launch and more.
 */

int main(int argc, char **argv) {
    // Set up ROS.
    ros::init(argc, argv, "droneoa_ros_package_manager");
    ros::NodeHandle n;
    ros::Rate r(GLOBAL_ROS_RATE);

    PM::CommandParser mPM;

    while (n.ok()) {
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}
