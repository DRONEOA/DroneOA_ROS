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

#ifndef CONSOLE_SERVICE_  // NOLINT
#define CONSOLE_SERVICE_  // NOLINT

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <droneoa_ros/CheckGetNewInput.h>

#include <string>

#include <boost/thread.hpp>

#include <droneoa_ros/Utils/GeneralUtils.hpp>
#include <droneoa_ros/PDN.hpp>

namespace ConsoleService {

class ConsoleService {
    ros::NodeHandle mNodeHandle;
    std::string mModuleName;
    std::string mNewInput;
    bool mMsgExpired = false;
    ros::ServiceServer mService;
    ros::Publisher mNewInputFlagPub;
    // Input topic
    boost::thread* thread_watch_topic_input_ = nullptr;
    void watchTopicInputThread();
    void input_callback(const std_msgs::String::ConstPtr& msg);
    // Helper
    bool matchingModuleName(std::string inputModuleName, std::string desireModuleNames);
    std::string getModuleName(std::string input);
    bool handleGetInputRequest(droneoa_ros::CheckGetNewInput::Request  &req,  // NOLINT
            droneoa_ros::CheckGetNewInput::Response &res);  // NOLINT
    void changeLoggerLevel(std::string input);

 public:
    ConsoleService();
    ~ConsoleService();
    void handleConsoleInput(std::string input);
};

}  // namespace ConsoleService

#endif  // NOLINT
