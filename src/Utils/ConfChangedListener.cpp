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
 * Written by Clair Hu <clair.hu.1@uwaterloo.ca>, Dec 2020
 */

#include <droneoa_ros/Utils/ConfChangedListener.hpp>
#include <ros/ros.h>
#include <fstream>
#include <droneoa_ros/Utils/JsonUtils.hpp>

ConfChangedListener::ConfChangedListener(std::string filePath) {
    thread_watch_command_ = new boost::thread(boost::bind(&ConfChangedListener::watchConfFileThread, this));
    jsonUtils = new JsonUtils(filePath);
}

ConfChangedListener::~ConfChangedListener() {
    if (jsonUtils) {
        delete(jsonUtils);
    }
    if (thread_watch_command_) {
        delete(thread_watch_command_);
    }
}

void ConfChangedListener::conf_callback() {
    ROS_INFO("watch conf file thread while loop");
    for (auto param : jsonUtils->readJsonToMap()) {
        dataPool.setConfig(param.first, param.second);
        std::cout << param.first << ": " << param.second << std::endl;
    }
}

void ConfChangedListener::watchConfFileThread() {
    auto rate = ros::Rate(0.1);
    while (ros::ok) {
        conf_callback();
        rate.sleep();
    }
}
