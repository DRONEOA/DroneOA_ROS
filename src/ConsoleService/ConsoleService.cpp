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

#include <ros/console.h>
#include <droneoa_ros/ConsoleService/ConsoleService.hpp>

namespace ConsoleService {

void ConsoleService::watchTopicInputThread() {
    auto rate = ros::Rate(10);
    auto node = boost::make_shared<ros::NodeHandle>();
    auto input_sub =
        node->subscribe<std_msgs::String>(INPUT_THROUGH_TOPIC_TOPIC_NAME, 1,
            boost::bind(&ConsoleService::input_callback, this, _1));

    while (ros::ok()) {
        ros::spinOnce();
        rate.sleep();
    }
}

void ConsoleService::input_callback(const std_msgs::String::ConstPtr& msg) {
    std_msgs::String inputCMD = *msg;
    handleConsoleInput(inputCMD.data);
}

bool ConsoleService::matchingModuleName(std::string inputModuleName, std::string desireModuleNames) {
    ROS_DEBUG("[Console Service] inputModuleName: [%s] desireModuleNames: [%s]",
            inputModuleName.c_str(), desireModuleNames.c_str());
    // Match any module name
    if (desireModuleNames == ANY_ACCEPTED_MODULE_NAMES) {
        return true;
    }
    // Match specific module name(s)
    std::string desireModuleName;
    std::istringstream tokenStream(desireModuleNames);
    while (std::getline(tokenStream, desireModuleName, CONSOLE_DELIMITER)) {
        if (desireModuleName == inputModuleName) {
            return true;
        }
    }
    return false;
}

std::string ConsoleService::getModuleName(std::string input) {
    std::string name;
    std::istringstream tokenStream(input);
    if (mNewInput.empty()) {
        return "";
    }
    std::getline(tokenStream, name, CONSOLE_DELIMITER);
    return name;
}

bool ConsoleService::handleGetInputRequest(droneoa_ros::CheckGetNewInput::Request  &req,
        droneoa_ros::CheckGetNewInput::Response &res) {
    if (!mMsgExpired && matchingModuleName(mModuleName, req.module_name)) {
        res.msg = mNewInput;
        ROS_DEBUG("[Console Service] Found match, response: [%s]", res.msg.c_str());
        // Any matcher, help matcher and quit matcher won't make the input message expire
        if (mModuleName != ANY_ACCEPTED_MODULE_NAMES &&
            mModuleName != VERBOSE_ACCEPTED_MODULE_NAMES &&
                res.msg != HELP_ACCEPTED_MODULE_NAMES &&
                res.msg != QUIT_ACCEPTED_MODULE_NAMES) {
            ROS_DEBUG("[Console Service] Mark As Expired");
            mMsgExpired = true;
        }
        return true;
    }
    return false;
}

void ConsoleService::changeLoggerLevel(std::string input) {
    if (input == "verbose debug") {
        GeneralUtility::setVerbosityLevel(GeneralUtility::E_VERBOSITY::DEBUG);
    } else {
        GeneralUtility::setVerbosityLevel(GeneralUtility::E_VERBOSITY::INFO);
    }
}

void ConsoleService::handleConsoleInput(std::string input) {
    mNewInput = input;
    // Pre-process input
    GeneralUtility::removeSpaces(&mNewInput);
    GeneralUtility::toLowerCaseStr(&mNewInput);
    mModuleName = getModuleName(mNewInput);
    ROS_DEBUG("[Console Service] new input: %s; module name: %s", mNewInput.c_str(), mModuleName.c_str());
    // Change verbose command
    if (mModuleName == "verbose") {
        changeLoggerLevel(mNewInput);
    }
    // Print verbose help
    if (mModuleName == "help") {
        ROS_WARN("Change Log Verbosity:");
        ROS_WARN("    verbose debug/info:     Default: INFO");
    }
    // Publish flag to indicate new input
    mMsgExpired = false;
    std_msgs::String newInputNotification;
    newInputNotification.data = "New Input";
    mNewInputFlagPub.publish(newInputNotification);
}

ConsoleService::ConsoleService() {
    mService = mNodeHandle.advertiseService(INPUT_MSG_REQUEST_SERVICE_NAME,
            &ConsoleService::handleGetInputRequest, this);
    mNewInputFlagPub = mNodeHandle.advertise<std_msgs::String> (NEW_INPUT_FLAG_TOPIC_NAME, 1000);
    thread_watch_topic_input_ = new boost::thread(boost::bind(&ConsoleService::watchTopicInputThread, this));
}

ConsoleService::~ConsoleService() {
    if (thread_watch_topic_input_) {
        delete thread_watch_topic_input_;
    }
}

}  // namespace ConsoleService
