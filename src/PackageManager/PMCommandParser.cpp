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
}

CommandParser::~CommandParser() {
    writeListToFile();
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
        launch();
    } else if (cmdType == "shutdown") {
        shutdown(false);
    } else {
        ROS_WARN("Unknown PM Command !!!");
        printHelp();
    }
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
    std::string cmd = DRONEOA_PATH + "/scripts/PackageManager/install.sh ";
    cmd = cmd + tokens[0] + " " + tokens[1];
    int ret = system(cmd.c_str());
    if (WEXITSTATUS(ret) != 0) {
        ROS_ERROR("[PM][INSTALL] CLONE and/or INITIAL PACKAGE SETUP failed !!!");
    }
    // Build packages
    if (!rebuild()) {
        ROS_ERROR("[PM][INSTALL] Rebuild failed !!!");
    }
    // Add package to installed package list
    mPackageList[tokens[0]] = PackageRecord(tokens[0]);
    mPackageList[tokens[0]].url = tokens[1];
    writeListToFile();
}

void CommandParser::update(std::vector<std::string> tokens) {
    //! @todo update branch? e.g. update xxx v0.2
    //! @todo update all command
    // Check arguments complete
    if (tokens.size() < 1) {
        ROS_ERROR("[PM][UPDATE] Missing Package Name Or URL !!!");
        return;
    }
    // Update latest by pull
    std::string cmd = DRONEOA_PATH + "/scripts/PackageManager/update.sh ";
    cmd = cmd + tokens[0];
    int ret = system(cmd.c_str());
    if (WEXITSTATUS(ret) != 0) {
        ROS_ERROR("[PM][UPDATE] Update pull latest failed !!!");
    }
    // Build packages
    if (!rebuild()) {
        ROS_ERROR("[PM][UPDATE] Rebuild failed !!!");
    }
}

void CommandParser::uninstall(std::vector<std::string> tokens) {
    //! @todo update branch? e.g. update xxx v0.2
    // Check arguments complete
    if (tokens.size() < 1) {
        ROS_ERROR("[PM][UNINSTALL] Missing Package Name Or URL !!!");
        return;
    }
    // Check if package name exist
    if (mPackageList.find(tokens[0]) == mPackageList.end()) {
        ROS_ERROR("[PM][UNINSTALL] Package Name [%s] Not Exist !!!", tokens[0].c_str());
        return;
    }
    // Update latest by pull
    std::string cmd = DRONEOA_PATH + "/scripts/PackageManager/uninstall.sh ";
    cmd = cmd + tokens[0];
    int ret = system(cmd.c_str());
    if (WEXITSTATUS(ret) != 0) {
        ROS_ERROR("[PM][UNINSTALL] Uninstall deletion failed !!!");
    }
    // Build packages
    if (!rebuild()) {
        ROS_ERROR("[PM][UNINSTALL] Rebuild failed !!!");
    }
}

void CommandParser::list(std::vector<std::string> tokens) {
    for (auto pkg : mPackageList) {
        ROS_INFO("> %s", pkg.second.toString().c_str());
    }
}

bool CommandParser::rebuild() {
    std::string cmd = DRONEOA_PATH + "/scripts/PackageManager/rebuild.sh";
    int ret = system(cmd.c_str());
    std::cout << "Return: " << WEXITSTATUS(ret) << std::endl;
    if (ret != 0) {
        ROS_ERROR("[PM][REBUILD] Rebuild failed !!!");
        return false;
    }
    return true;
}

void CommandParser::launch() {
    //! @todo launch specific node
    //! @todo launch all startWithMainNode = true nodes
    std::string cmd = "roslaunch " + DRONEOA_PATH + "/launch/step2.launch";
    system(cmd.c_str());
}

void CommandParser::shutdown(bool killAllNodes) {
    //! @todo selective shutdown
    if (killAllNodes) {
        system("rosnode kill --all");
    } else {
        system("rosnode kill droneoa_ros");
        ros::shutdown();
    }
}

void CommandParser::printHelp() {
    ROS_WARN("Package Manager Commands: [required] <optional>");
    ROS_WARN("    install   [name of package] [repo url]");
    ROS_WARN("    update    [name of package]");
    ROS_WARN("    uninstall [name of package]");
    //! @todo help message
}

bool CommandParser::writeListToFile() {
    //! @todo json utility
}

bool CommandParser::readListFromFile() {
    //! @todo json utility
}


}  // namespace PM