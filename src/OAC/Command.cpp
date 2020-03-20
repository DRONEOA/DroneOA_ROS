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
 * Written by Bohan Shi <b34shi@edu.uwaterloo.ca>, January 2020
 */

#include <droneoa_ros/OAC/Command.hpp>
#include <droneoa_ros/Utils/GeneralUtils.hpp>

std::vector<std::string> getDataListFromString(std::string data) {
    std::string token;
    std::istringstream tokenStream(data);
    std::vector<std::string> dataList;

    while (std::getline(tokenStream, token, CommandDataDelimiter)) {
        GeneralUtility::toLowerCaseStr(&token);
        dataList.push_back(token);
    }

    return dataList;
}

bool parseCMD(CNCInterface *cnc, const CommandLine& cmdline) {
    try {
        switch (cmdline.first) {
            case CMD_QUEUE_TYPES::CMD_CHMOD:
            {
                if (cnc->getMode() == cmdline.second) return true;
                return cnc->setMode(cmdline.second);
            }
            case CMD_QUEUE_TYPES::CMD_SET_MAX_VELOCITY:
            {
                float targetSpeed = std::stof(cmdline.second);
                targetSpeed = targetSpeed >= 0 ? targetSpeed : 0;
                return cnc->setMaxSpeed(1, targetSpeed, 0);
            }
            case CMD_QUEUE_TYPES::CMD_GOTO_RELATIVE:
            {
                std::vector<std::string> dataList = getDataListFromString(cmdline.second);
                if (dataList.size() != 3) throw 1;
                float northAxis = std::stof(dataList.at(0));
                float eastAxis = std::stof(dataList.at(1));
                float alt = std::stof(dataList.at(2));
                return cnc->gotoRelative(northAxis, eastAxis, alt);
            }
            case CMD_QUEUE_TYPES::CMD_GOTO_GLOBAL:
            {
                std::vector<std::string> dataList = getDataListFromString(cmdline.second);
                if (dataList.size() != 3) throw 1;
                float latPos = std::stof(dataList.at(0));
                float longPos = std::stof(dataList.at(1));
                float alt = std::stof(dataList.at(2));
                return cnc->gotoGlobal(latPos, longPos, alt);
            }
            default:
                throw 1;
        }
    } catch (...) {
        ROS_ERROR("[CMD PARSER] Invalid Command or arguments: %s, %s", CMD_QUEUE_TYPES_NAME[cmdline.first],
                cmdline.second.c_str());
        return false;
    }
}
