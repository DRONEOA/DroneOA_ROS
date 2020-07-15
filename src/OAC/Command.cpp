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

#include <ros/ros.h>

#include <sstream>

#include <droneoa_ros/OAC/Command.hpp>
#include <droneoa_ros/Utils/GeneralUtils.hpp>
#include <droneoa_ros/HWI/Utils/CNCUtils.hpp>
#include <droneoa_ros/HWI/CNCArdupilot.hpp>

namespace Command {

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

bool parseCMD(CNC::CNCInterface *cnc, const CommandLine& cmdline, bool isFromOAC) {
    try {
        switch (cmdline.first) {
            case CMD_QUEUE_TYPES::CMD_CHMOD:
            {
                if (cnc->getMode() == cmdline.second) return true;
                return cnc->setMode(cmdline.second);
            }
            case CMD_QUEUE_TYPES::CMD_ARM:
            {
                if (CURRENT_FCU_TYPE == FCU_ARDUPILOT) {
                    cnc->setMode(FLT_MODE_GUIDED);
                } else if (CURRENT_FCU_TYPE == FCU_PX4) {
                    cnc->setMode(FLT_MODE_OFFBOARDD);
                }
                return cnc->armVehicle();
            }
            case CMD_QUEUE_TYPES::CMD_TAKEOFF:
            {
                if (!cnc->isArmed()) {
                    ROS_WARN("VEHICLE NOT ARMED !!!");
                    return false;
                }
                float targetAltitude = std::stof(cmdline.second);
                return cnc->takeoff(targetAltitude);
            }
            case CMD_QUEUE_TYPES::CMD_LAND:
            {
                return cnc->land(1);
            }
            case CMD_QUEUE_TYPES::CMD_SET_MAX_VELOCITY:
            {
                float targetSpeed = std::stof(cmdline.second);
                targetSpeed = targetSpeed >= 0 ? targetSpeed : 0;
                return cnc->setMaxSpeed(1, targetSpeed, 0);
            }
            case CMD_QUEUE_TYPES::CMD_SET_YAW:
            {
                float targetYaw = std::stof(cmdline.second);
                return cnc->setYaw(targetYaw, false, isFromOAC);
            }
            case CMD_QUEUE_TYPES::CMD_GOTO_RELATIVE:
            {
                std::vector<std::string> dataList = getDataListFromString(cmdline.second);
                if (dataList.size() > 3 || dataList.size() < 2) throw 1;
                float northAxis = std::stof(dataList.at(0));
                float eastAxis = std::stof(dataList.at(1));
                float alt = cnc->getRelativeAltitude();
                if (dataList.size() == 3) {
                    alt = std::stof(dataList.at(2));
                }
                return cnc->gotoRelative(northAxis, eastAxis, alt, false, isFromOAC);
            }
            case CMD_QUEUE_TYPES::CMD_GOTO_GLOBAL_ENU:
            {
                ROS_WARN("CMD_GOTO_GLOBAL_ENU: %s", cmdline.second.c_str());
                std::vector<std::string> dataList = getDataListFromString(cmdline.second);
                if (dataList.size() > 3 || dataList.size() < 2) throw 1;
                float x = std::stof(dataList.at(0));
                float y = std::stof(dataList.at(1));
                float z = cnc->getRelativeAltitude();
                if (dataList.size() == 3) {
                    z = std::stof(dataList.at(2));
                }
                return cnc->pushLocalENUWaypoint(LocalPoint(x, y, z), isFromOAC);
            }
            case CMD_QUEUE_TYPES::CMD_GOTO_GLOBAL_GPS:
            {
                std::vector<std::string> dataList = getDataListFromString(cmdline.second);
                if (dataList.size() > 3 || dataList.size() < 2) throw 1;
                float latPos = std::stof(dataList.at(0));
                float longPos = std::stof(dataList.at(1));
                float alt = cnc->getRelativeAltitude();
                if (dataList.size() == 3) {
                    alt = std::stof(dataList.at(2));
                }
                return cnc->gotoGlobal(latPos, longPos, alt, isFromOAC);
            }
            case CMD_QUEUE_TYPES::CMD_GOTO_HEADING:
            {
                std::vector<std::string> dataList = getDataListFromString(cmdline.second);
                if (dataList.size() > 3 || dataList.size() < 2) throw 1;
                float heading = std::stof(dataList.at(0));
                float dist = std::stof(dataList.at(1));
                float alt = cnc->getRelativeAltitude();
                if (dataList.size() == 3) {
                    alt = std::stof(dataList.at(2));
                }
                if (!cnc->setYaw(heading, false, isFromOAC)) {
                    return false;
                }
                return cnc->gotoHeading(heading, dist, alt, isFromOAC);
            }
            case CMD_QUEUE_TYPES::CMD_CLIMB:
            {
                float deltaAlt = std::stof(cmdline.second);
                if (deltaAlt < 0.0f) {
                    throw 1;
                }
                float heading = cnc->getHUDData().heading;
                float dist = 0.0f;
                float alt = cnc->getRelativeAltitude();
                //! @TODO To prevent slight heading change, try magnetic compass?
                return cnc->gotoHeading(heading, dist, deltaAlt+alt, isFromOAC);
            }
            case CMD_QUEUE_TYPES::CMD_DESCEND:
            {
                float deltaAlt = std::stof(cmdline.second);
                if (deltaAlt < 0.0f) {
                    throw 1;
                }
                float heading = cnc->getHUDData().heading;
                float dist = 0.0f;
                float alt = cnc->getRelativeAltitude();
                //! @TODO To prevent slight heading change, try magnetic compass?
                return cnc->gotoHeading(heading, dist, deltaAlt-alt, isFromOAC);
            }
            case CMD_QUEUE_TYPES::CMD_PUSH_MISSION_QUEUE:
            {
                std::istringstream iss(cmdline.second);
                std::vector<std::string> results((std::istream_iterator<std::string>(iss)),
                        std::istream_iterator<std::string>());
                //! @todo this depends on type of FCU
                std::vector<GPSPoint> wps;
                for (auto gps : results) {
                    std::istringstream singleWPss(gps);
                    std::vector<float> resultWPBreak;
                    while (singleWPss.good()) {
                        std::string singleWPStr;
                        getline(singleWPss, singleWPStr, ',');
                        resultWPBreak.push_back(std::stof(singleWPStr));
                    }
                    if (resultWPBreak.size() != 3) throw 1;
                    wps.push_back(GPSPoint(resultWPBreak.at(0), resultWPBreak.at(1), resultWPBreak.at(2)));
                }
                cnc->clearFCUWaypoint();
                return cnc->pushGlobalMission(wps, true);
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

}  // namespace Command
