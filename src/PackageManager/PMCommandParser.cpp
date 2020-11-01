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

#include <sstream>
#include <iostream>

#include <droneoa_ros/PackageManager/PMCommandParser.hpp>
#include <droneoa_ros/Utils/GeneralUtils.hpp>
#include <droneoa_ros/PDN.hpp>

namespace PM {

PackageRecord::PackageRecord() : name(""), startWithMainNode(false) {}

PackageRecord::PackageRecord(std::string pktName) : name(pktName), startWithMainNode(false) {}

std::string PackageRecord::toString() {
    std::string result = name + " [" + branch + "]";
    return result;
}


CommandParser::CommandParser() {
    DRONEOA_PATH = ros::package::getPath("droneoa_ros");
    readListFromFile();
    thread_watch_command_ = new boost::thread(boost::bind(&CommandParser::watchCommandThread, this));
}

CommandParser::~CommandParser() {
    if (thread_watch_command_) delete thread_watch_command_;
    writeListToFile();
}

void CommandParser::command_callback(const std_msgs::String::ConstPtr& msg) {
    std_msgs::String inputCMD = *msg;
    if (!parseInput(inputCMD.data)) {
        ROS_WARN("[PM] Command from unhandled_inputs: %s. Ignored", inputCMD.data.c_str());
    }
}

void CommandParser::watchCommandThread() {
    auto rate = ros::Rate(10);
    auto node = boost::make_shared<ros::NodeHandle>();
    auto cmd_sub =
        node->subscribe<std_msgs::String>("droneoa/unhandled_inputs", 1,
            boost::bind(&CommandParser::command_callback, this, _1));

    while (ros::ok()) {
        ros::spinOnce();
        rate.sleep();
    }
}

bool CommandParser::parseInput(std::string input) {
    std::vector<std::string> result;
    result.clear();
    std::string token;
    std::istringstream tokenStream(input);

    while (std::getline(tokenStream, token, CONSOLE_DELIMITER)) {
        GeneralUtility::toLowerCaseStr(&token);
        result.push_back(token);
    }

    return parseInput(result);
}

bool CommandParser::parseInput(std::vector<std::string> tokens) {
    // Check if command if for Package Manger Module
    if (tokens.size() == 0) {
        return false;
    }
    if (tokens[0] != "pm") {
        return false;
    }
    tokens.erase(tokens.begin());
    // Check Command Type NOT Specified / Help
    if (tokens.size() == 0 || tokens[0] == "help") {
        printHelp();
        return true;
    }
    // Check Command Type
    std::string cmdType = tokens[0];
    tokens.erase(tokens.begin());
    if (cmdType == "install") {
        install(tokens);
    } else if (cmdType == "update") {
        update(tokens);
    } else if (cmdType == "uninstall") {
        uninstall(tokens);
    } else if (cmdType == "list") {
        list(tokens);
    } else if (cmdType == "launch") {
        launch(tokens);
    } else if (cmdType == "shutdown") {
        shutdown(tokens);
    } else {
        ROS_WARN("Unknown PM Command !!!");
        printHelp();
    }
    return true;
}

void CommandParser::install(std::vector<std::string> tokens) {
    //! @todo specific branch
    // Check arguments complete
    if (tokens.size() < 2) {
        ROS_ERROR("[PM][INSTALL] Missing Package Name Or URL !!!");
        return;
    }
    // Check if package name already exist
    if (mPackageList.find(tokens[0]) != mPackageList.end()) {
        ROS_ERROR("[PM][INSTALL] Package Name [%s] Already Exist !!!", tokens[0].c_str());
        return;
    }
    // Run install script (clone & pkg's install script)
    std::string cmd = "bash " + DRONEOA_PATH + "/scripts/PackageManager/install.sh ";
    cmd = cmd + tokens[0] + " " + tokens[1];
    int ret = system(cmd.c_str());
    if (WEXITSTATUS(ret) != 0) {
        ROS_ERROR("[PM][INSTALL] CLONE and/or INITIAL PACKAGE SETUP failed !!!");
        return;
    }
    // Build packages
    if (!rebuild()) {
        ROS_ERROR("[PM][INSTALL] Rebuild failed !!!");
        uninstall(tokens);  // Restore file structure
        return;
    }
    // Add package to installed package list
    mPackageList[tokens[0]] = PackageRecord(tokens[0]);
    mPackageList[tokens[0]].url = tokens[1];
    writeListToFile();
    ROS_INFO("[PM][INSTALL] Package: %s is Successfully Installed.", tokens[0].c_str());
}

void CommandParser::update(std::vector<std::string> tokens) {
    //! @todo update branch? e.g. update xxx v0.2
    //! @todo update all command
    // Check arguments complete
    if (tokens.size() < 1) {
        ROS_ERROR("[PM][UPDATE] Missing Package Name !!!");
        return;
    }
    // Update latest by pull
    std::string cmd = "bash " + DRONEOA_PATH + "/scripts/PackageManager/update.sh ";
    cmd = cmd + tokens[0];
    int ret = system(cmd.c_str());
    if (WEXITSTATUS(ret) != 0) {
        ROS_ERROR("[PM][UPDATE] Update pull latest failed !!!");
        return;
    }
    // Build packages
    if (!rebuild()) {
        ROS_ERROR("[PM][UPDATE] Rebuild failed !!!");
        return;
    }
    ROS_INFO("[PM][UPDATE] Package: %s is Successfully Updated.", tokens[0].c_str());
}

void CommandParser::uninstall(std::vector<std::string> tokens) {
    //! @todo update branch? e.g. update xxx v0.2
    // Check arguments complete
    if (tokens.size() < 1) {
        ROS_ERROR("[PM][UNINSTALL] Missing Package Name !!!");
        return;
    }
    // Check if package name exist
    if (mPackageList.find(tokens[0]) == mPackageList.end()) {
        ROS_ERROR("[PM][UNINSTALL] Package Name [%s] Not Exist !!!", tokens[0].c_str());
        return;
    }
    // Update latest by pull
    std::string cmd = "bash " + DRONEOA_PATH + "/scripts/PackageManager/uninstall.sh ";
    cmd = cmd + tokens[0];
    int ret = system(cmd.c_str());
    if (WEXITSTATUS(ret) != 0) {
        ROS_ERROR("[PM][UNINSTALL] Uninstall deletion failed !!!");
        return;
    }
    // Build packages
    if (!rebuild()) {
        ROS_ERROR("[PM][UNINSTALL] Rebuild failed !!!");
    } else {
        ROS_INFO("[PM][UNINSTALL] Package: %s is Successfully Uninstalled.", tokens[0].c_str());
    }
    // Remove From Installed List
    mPackageList.erase(mPackageList.find(tokens[0]));
}

void CommandParser::list(std::vector<std::string> tokens) {
    // Check arguments complete
    if (tokens.size() < 1) {
        ROS_ERROR("[PM][LIST] Missing Package Name !!!");
        return;
    }
    if (tokens[0] == "running") {
        ROS_INFO("[PM][LIST] Listing Running Nodes");
        system("rosnode list");
    } else if (tokens[0] == "installed") {
        ROS_INFO("[PM][LIST] Listing Installed Nodes");
        for (auto pkg : mPackageList) {
            ROS_INFO("    > %s", pkg.second.toString().c_str());
        }
    } else {
        ROS_ERROR("[PM][LIST] Unknown Listing Type (running, installed)");
    }
}

bool CommandParser::rebuild() {
    std::string cmd = "bash " + DRONEOA_PATH + "/scripts/PackageManager/rebuild.sh";
    int ret = system(cmd.c_str());
    std::cout << "Return: " << WEXITSTATUS(ret) << std::endl;
    if (ret != 0) {
        ROS_ERROR("[PM][REBUILD] Rebuild failed !!!");
        return false;
    }
    ROS_INFO("[PM][REBUILD] Rebuild Operation is Successful.");
    return true;
}

void CommandParser::launch(std::vector<std::string> tokens) {
    std::string cmd;
    if (tokens.size() > 0 && tokens[0] != "main") {
        // Check if package name exist
        if (mPackageList.find(tokens[0]) == mPackageList.end()) {
            ROS_ERROR("[PM][LAUNCH] Package Name [%s] Not Exist !!!", tokens[0].c_str());
            return;
        }
        cmd = "bash " + DRONEOA_PATH + "/scripts/PackageManager/launch.sh " + tokens[0];
    } else {
        //! @todo launch all startWithMainNode = true nodes (background)
        // Main Node Always Launch Last, As it accept console inputs
        cmd = "roslaunch " + DRONEOA_PATH + "/launch/step2.launch";
    }
    int ret = system(cmd.c_str());
    if (WEXITSTATUS(ret) != 0) {
        ROS_ERROR("[PM][LAUNCH] Launch failed !!!");
        return;
    }
}

void CommandParser::shutdown(std::vector<std::string> tokens) {
    // Check arguments complete
    if (tokens.size() < 1) {
        ROS_ERROR("[PM][SHUTDOWN] Missing Package !!!");
        return;
    }
    std::string nodeName = tokens[0];
    std::string cmd;
    if (nodeName == "all") {
        cmd = "rosnode kill --all";
    } else if (nodeName == "droneoa_ros" || tokens[0] == "main" || tokens[0] == "droneoa") {
        nodeName = "droneoa_ros";
        cmd = "bash " + DRONEOA_PATH + "/scripts/PackageManager/kill.sh " + nodeName;
    } else {
        cmd = "bash " + DRONEOA_PATH + "/scripts/PackageManager/shutdownOtherNode.sh " + nodeName;
    }
    int ret = system(cmd.c_str());
    if (WEXITSTATUS(ret) != 0) {
        ROS_ERROR("[PM][SHUTDOWN] Shutdown Node: %s Failed", nodeName.c_str());
        return;
    }
    ROS_INFO("[PM][SHUTDOWN] Shutdown Node: %s is Successful.", nodeName.c_str());
}

void CommandParser::safeShutdownMainNode() {
    //! @todo Send "cnc quit" via topic
    //! @todo Should Main Node quit on its own before went to PM (need pickup but forward commands)?
    //          or PM close main?
}

void CommandParser::printHelp() {
    ROS_WARN("Package Manager Commands: [required] <optional>");
    ROS_WARN("    install   [name of package] [repo url]");
    ROS_WARN("    update    [name of package]");
    ROS_WARN("    uninstall [name of package]");
    ROS_WARN("    list      [installed / running]");
    ROS_WARN("    launch    WIP: default launch main node");
    ROS_WARN("    shutdown  [node name / all]");
    //! @todo help message
}

bool CommandParser::writeListToFile() {
    //! @todo json utility
}

bool CommandParser::readListFromFile() {
    //! @todo json utility
}


}  // namespace PM
