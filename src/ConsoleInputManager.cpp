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
 * Written by Bohan Shi <b34shi@edu.uwaterloo.ca>, Feb. 2020
 */

#include <sstream>
#include <algorithm>
#include <cctype>
#include <string>
#include <droneoa_ros/ConsoleInputManager.hpp>
#include <droneoa_ros/CNCInterface.hpp>

/* Helper Function */
char asciitolower(char in) {
    if (in <= 'Z' && in >= 'A')
        return in - ('Z' - 'z');
    return in;
}

void toLowerCaseStr(std::string* input) {
    std::transform(input->begin(), input->end(), input->begin(), asciitolower);
}

ConsoleInputManager::ConsoleInputManager(bool* masterSwitch) : masterSwitch_(masterSwitch) {}

bool ConsoleInputManager::parseAndExecuteConsole(std::string cmd) {
    //! @todo(shibohan) Detect composed commands, use runner in this case
    if (!splitModuleCommand(cmd)) {
        ROS_WARN("Command Missing Module Name");
        printFormatHelper();
        return false;
    }
    return commandDispatch();
}

bool ConsoleInputManager::splitModuleCommand(std::string cmd) {
    currentCommand_.first = "INVALID";
    currentCommand_.second.clear();
    std::string token;
    std::istringstream tokenStream(cmd);

    while (std::getline(tokenStream, token, ConsoleDelimiter)) {
        if (currentCommand_.first == "INVALID") {
            currentCommand_.first = token;
            continue;
        }
        toLowerCaseStr(&token);
        currentCommand_.second.push_back(token);
    }

    if (currentCommand_.first == "INVALID" || currentCommand_.first == "") {
        return false;
    }
    return true;
}

bool ConsoleInputManager::commandDispatch() {
    if (currentCommand_.first == "CNC") {
        return handleCNCCommands();
    } else if (currentCommand_.first == "OAC") {
        return handleOACCommands();
    } else {
        ROS_WARN("Unknown Module Name: %s", currentCommand_.first.c_str());
        printModuleHelper();
        return false;
    }
}

bool ConsoleInputManager::handleCNCCommands() {
    try {
        std::string cmdType = currentCommand_.second.at(0);
        if (cmdType == "arm") {
            ROS_WARN("ARM::");
        } else if (cmdType == "takeoff") {
            ROS_WARN("TAKEOFF::");
        } else if (cmdType == "quit") {
            ROS_WARN("QUIT::");
            *masterSwitch_ = false;
        }
    } catch (...) {
        ROS_WARN("ERROR happened while processing CNC command");
    }
    return true;
}

bool ConsoleInputManager::handleOACCommands() {
    return true;
}

void ConsoleInputManager::printFormatHelper() {
    ROS_WARN("Command Format:  no < > in actually command");
    ROS_WARN("    <Module Name> <arg1> <arg2> ...");
}

void ConsoleInputManager::printModuleHelper() {
    ROS_WARN("Module Names:");
    ROS_WARN("    CNC:    Command And Control Module");
    ROS_WARN("    OAC:    Obstacle Avoidance Algorithm Controller");
}
