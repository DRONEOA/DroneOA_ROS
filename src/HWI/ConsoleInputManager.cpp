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
#include <string>
#include <droneoa_ros/HWI/ConsoleInputManager.hpp>
#include <droneoa_ros/HWI/Utils/CNCUtils.hpp>
#include <droneoa_ros/Utils/GeneralUtils.hpp>

namespace IO {

ConsoleInputManager::ConsoleInputManager(bool* masterSwitch) : masterSwitch_(masterSwitch),
        cnc_(nullptr), rsc_(nullptr), oac_(nullptr), lidar_(nullptr) {}

ConsoleInputManager::~ConsoleInputManager() {
    if (thread_watch_command_) delete thread_watch_command_;
}

bool ConsoleInputManager::init(CNC::CNCInterface* cnc, Depth::RSC *rsc, OAC::OAController *oac,
        Lidar::LidarGeneric *lidar, SLAM::OctomapClient *octomapClient) {
    cnc_ = cnc;
    rsc_ = rsc;
    oac_ = oac;
    lidar_ = lidar;
    octomapClient_ = octomapClient;
    if (!(cnc_ && rsc_ && oac_ && lidar_ && octomapClient_)) {
        return false;
    }
    thread_watch_command_ = new boost::thread(boost::bind(&ConsoleInputManager::watchCommandThread, this));
    return true;
}

void ConsoleInputManager::command_callback(const std_msgs::String::ConstPtr& msg) {
    std_msgs::String inputCMD = *msg;
    parseAndExecuteConsole(inputCMD.data);
}

void ConsoleInputManager::watchCommandThread() {
    auto rate = ros::Rate(OAC_REFRESH_FREQ);
    auto node = boost::make_shared<ros::NodeHandle>();
    auto gpsFix_sub =
        node->subscribe<std_msgs::String>("droneoa/input_command", 1,
            boost::bind(&ConsoleInputManager::command_callback, this, _1));

    while (ros::ok()) {
        ros::spinOnce();
        rate.sleep();
    }
}

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
        GeneralUtility::toLowerCaseStr(&token);
        if (currentCommand_.first == "INVALID") {
            currentCommand_.first = token;
            continue;
        }
        currentCommand_.second.push_back(token);
    }

    if (currentCommand_.first == "INVALID" || currentCommand_.first == "") {
        return false;
    }
    return true;
}

bool ConsoleInputManager::commandDispatch() {
    if (currentCommand_.first == "cnc") {
        return handleCNCCommands();
    } else if (currentCommand_.first == "oac") {
        return handleOACCommands();
    } else if (currentCommand_.first == "rsc") {
        if (!ENABLE_RSC) {
            ROS_WARN("This module [RSC] is not enabled !!!");
            return false;
        }
        return handleRSCCommands();
    } else if (currentCommand_.first == "lidar") {
        if (!ENABLE_LIDAR) {
            ROS_WARN("This module [LIDAR] is not enabled !!!");
            return false;
        }
        return handleLIDARCommands();
    } else if (currentCommand_.first == "map") {
        //! @todo possible to also support 2d lidar
        if (!ENABLE_RSC) {
            ROS_WARN("This module requires [RSC] !!!");
            return false;
        }
        return handleMapCommands();
    } else if (currentCommand_.first == "!") {
        return handleQuickCommands();
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
            ROS_WARN("::ARM::");
            cnc_->setMode(FLT_MODE_GUIDED);
            cnc_->armVehicle();
        } else if (cmdType == "takeoff") {
            ROS_WARN("::TAKEOFF::");
            if (!cnc_->isArmed()) {
                ROS_WARN("VEHICLE NOT ARMED !!!");
                return false;
            }
            float targetAltitude = std::stof(currentCommand_.second.at(1));
            cnc_->takeoff(targetAltitude);
        } else if (cmdType == "chmod") {
            std::string currentMode = cnc_->getMode();
            std::string newMode = currentCommand_.second.at(1);
            if (!cnc_->checkFModeExist(newMode)) {
                ROS_WARN("Flight Mode Does Not Exist !!!");
                return false;
            }
            GeneralUtility::toUpperCaseStr(&newMode);
            ROS_WARN("::ChangeMode %s -> %s::", currentMode.c_str(), newMode.c_str());
            cnc_->setMode(newMode);
        } else if (cmdType == "land") {
            ROS_WARN("::LAND::");
            cnc_->land(1);
        } else if (cmdType == "rtl") {
            ROS_WARN("::RTL::");
            cnc_->setMode(FLT_MODE_RTL);
        } else if (cmdType == "velocity") {
            float vel = std::stof(currentCommand_.second.at(1));
            ROS_WARN("::SET MAX VELOCITY -> %f::", vel);
            if (vel == 0.0) {
                ROS_WARN("Invalid Speed Setting");
                return false;
            }
            cnc_->setMaxSpeed(1, vel, 0);
        } else if (cmdType == "yaw") {
            float yawAngle = std::stof(currentCommand_.second.at(1));
            ROS_WARN("::SET YAW -> %f::", yawAngle);
            cnc_->setYaw(yawAngle);
            if (currentCommand_.second.size() >= 3) {
                float dist = std::stof(currentCommand_.second.at(2));
                float alt = cnc_->getRelativeAltitude();
                if (currentCommand_.second.size() >= 4) {
                    alt = std::stof(currentCommand_.second.at(3));
                }
                ROS_WARN("::GOTO YAW -> yaw:%f dist:%f alt:%f::", yawAngle, dist, alt);
                cnc_->gotoHeading(yawAngle, dist, alt);
            }
        } else if (cmdType == "info") {
            GPSPoint tmpGPSPoint = cnc_->getCurrentGPSPoint();
            ROS_INFO(">>>>>>>>>> INFO START <<<<<<<<<<");
            ROS_INFO("[DISPLAY] gps: %f %f", tmpGPSPoint.latitude_, tmpGPSPoint.longitude_);
            ROS_INFO("[DISPLAY] altitude: %f", cnc_->getRelativeAltitude());
            ROS_INFO("[DISPLAY] mode: %s", cnc_->getMode().c_str());
            ROS_INFO("[DISPLAY] voltage: %f", cnc_->getBatteryVoltage());
            ROS_INFO("[DISPLAY] orientation: %f, %f, %f", cnc_->getIMUData().orientation.x,
                                                        cnc_->getIMUData().orientation.y,
                                                        cnc_->getIMUData().orientation.z);
            ROS_INFO("[HUD] heading: %d", cnc_->getHUDData().heading);
            ROS_INFO("[HUD] airspeed: %f", cnc_->getHUDData().airspeed);
            ROS_INFO("[HUD] groundspeed: %f", cnc_->getHUDData().groundspeed);
            ROS_INFO("[HUD] altitude: %f", cnc_->getHUDData().altitude);
            ROS_INFO("[HUD] climb: %f", cnc_->getHUDData().climb);
            ROS_INFO("[HUD] throttle: %f", cnc_->getHUDData().throttle);
            ROS_INFO(">>>>>>>>>> INFO END <<<<<<<<<<");
        } else if (cmdType == "quit") {
            ROS_WARN("::QUIT::");
            oac_->masterSwitch(false);
            *masterSwitch_ = false;
        } else {
            ROS_WARN("Unknown CNC command");
            printCNCHelper();
        }
    } catch (...) {
        ROS_WARN("ERROR happened while processing CNC command");
        printCNCHelper();
    }
    return true;
}

bool ConsoleInputManager::handleOACCommands() {
    try {
        std::string cmdType = currentCommand_.second.at(0);
        if (cmdType == "on") {
            ROS_WARN("::OAC->ON::");
            oac_->masterSwitch(true);
        } else if (cmdType == "off") {
            ROS_WARN("::OAC->OFF::");
            oac_->masterSwitch(false);
        } else if (cmdType == "info") {
            ROS_INFO(">>>>>>>>>> INFO START <<<<<<<<<<");
            ROS_INFO("[OAC] status: %s", oac_->getStatus().c_str());
            ROS_INFO(">>>>>>>>>> INFO END <<<<<<<<<<");
        } else {
            ROS_WARN("Unknown OAC command");
            printOACHelper();
        }
    } catch (...) {
        ROS_WARN("ERROR happened while processing OAC command");
        printOACHelper();
    }
    return true;
}

//! @note Keep this handler small !!!
bool ConsoleInputManager::handleQuickCommands() {
    try {
        std::string cmdType = currentCommand_.second.at(0);
        if (cmdType == "w") {
            ROS_WARN("::GO NORTH::");
            float dist = std::stof(currentCommand_.second.at(1));
            float alt = cnc_->getRelativeAltitude();
            if (currentCommand_.second.size() >= 3) {
                alt = std::stof(currentCommand_.second.at(2));
            }
            cnc_->gotoRelative(dist, 0, alt);
            cnc_->setYaw(CNC::CNCUtility::getBearing(cnc_->getCurrentGPSPoint(), cnc_->getTargetWaypoint()));
        } else if (cmdType == "s") {
            ROS_WARN("::GO SOUTH::");
            float dist = std::stof(currentCommand_.second.at(1));
            float alt = cnc_->getRelativeAltitude();
            if (currentCommand_.second.size() >= 3) {
                alt = std::stof(currentCommand_.second.at(2));
            }
            cnc_->gotoRelative(-dist, 0, alt);
            cnc_->setYaw(CNC::CNCUtility::getBearing(cnc_->getCurrentGPSPoint(), cnc_->getTargetWaypoint()));
        } else if (cmdType == "a") {
            ROS_WARN("::GO WEST::");
            float dist = std::stof(currentCommand_.second.at(1));
            float alt = cnc_->getRelativeAltitude();
            if (currentCommand_.second.size() >= 3) {
                alt = std::stof(currentCommand_.second.at(2));
            }
            cnc_->gotoRelative(0, -dist, alt);
            cnc_->setYaw(CNC::CNCUtility::getBearing(cnc_->getCurrentGPSPoint(), cnc_->getTargetWaypoint()));
        } else if (cmdType == "d") {
            ROS_WARN("::GO EAST::");
            float dist = std::stof(currentCommand_.second.at(1));
            float alt = cnc_->getRelativeAltitude();
            if (currentCommand_.second.size() >= 3) {
                alt = std::stof(currentCommand_.second.at(2));
            }
            cnc_->gotoRelative(0, dist, alt);
            cnc_->setYaw(CNC::CNCUtility::getBearing(cnc_->getCurrentGPSPoint(), cnc_->getTargetWaypoint()));
        } else {
            ROS_WARN("Unknown Quick command");
            printQuickHelper();
        }
    } catch (...) {
        ROS_WARN("ERROR happened while processing Quick command");
        printQuickHelper();
    }
    return true;
}

bool ConsoleInputManager::handleRSCCommands() {
    try {
        std::string cmdType = currentCommand_.second.at(0);
        if (cmdType == "info") {
            ROS_INFO(">>>>>>>>>> INFO START <<<<<<<<<<");
            rsc_->printImgInfo();
            ROS_INFO(">>>>>>>>>> INFO END <<<<<<<<<<");
        } else if (cmdType == "chsrc") {
            if (currentCommand_.second.size() >= 2) {
                std::string srcName = currentCommand_.second.at(1);
                if (srcName == "rsc") {
                    ROS_WARN("::RSC SRC -> RSC::");
                    rsc_->changeDepthSource(DEPTH_SOURCE_RSC);
                    rsc_->changePC2Source(PC_SOURCE_RSC);
                } else if (srcName == "ue4") {
                    ROS_WARN("::RSC SRC -> UE4::");
                    rsc_->changeDepthSource(DEPTH_SOURCE_UE4);
                    rsc_->changePC2Source(PC_SOURCE_UE4);
                } else {
                    ROS_WARN("Unknown New Source Name");
                    ROS_WARN("    rsc:      use real realsense camera data");
                    ROS_WARN("    ue4:      use simulated data from UE4 SIM");
                }
            } else {
                ROS_WARN("Unknown New Source Name");
                ROS_WARN("    rsc:      use real realsense camera data");
                ROS_WARN("    ue4:      use simulated data from UE4 SIM");
            }
        } else if (cmdType == "range") {
            std::string cancel = currentCommand_.second.at(1);
            if (currentCommand_.second.size() == 3) {
                float max = std::stof(currentCommand_.second.at(1));
                float min = std::stof(currentCommand_.second.at(2));
                if (max < min || min < 0) {
                    throw 1;
                }
                rsc_->setRange(min, max);
                rsc_->setRangeSwitch(true);
            } else if (currentCommand_.second.size() == 2 && cancel == "cancel") {
                rsc_->setRangeSwitch(false);
            } else {
                ROS_WARN("Unknown Range Operation");
                ROS_WARN("    [max(mm)] [min(mm)]       Set range");
                ROS_WARN("    cancel                    Cancel range");
            }
        } else {
            ROS_WARN("Unknown RSC command");
            printRSCHelper();
        }
    } catch (...) {
        ROS_WARN("ERROR happened while processing RSC command");
        printRSCHelper();
    }
    return true;
}

bool ConsoleInputManager::handleLIDARCommands() {
    try {
        std::string cmdType = currentCommand_.second.at(0);
        if (cmdType == "info") {
            ROS_INFO(">>>>>>>>>> INFO START <<<<<<<<<<");
            lidar_->printLidarInfo();
            ROS_INFO(">>>>>>>>>> INFO END <<<<<<<<<<");
        } else if (cmdType == "chsrc") {
            if (currentCommand_.second.size() >= 2) {
                std::string srcName = currentCommand_.second.at(1);
                if (srcName == "ydlidar") {
                    ROS_WARN("::LIDAR SRC -> YDLIDAR::");
                    lidar_->changeLidarSource(LIDAR_SOURCE_YDLIDAR);
                } else if (srcName == "ue4") {
                    ROS_WARN("::LIDAR SRC -> UE4::");
                    lidar_->changeLidarSource(LIDAR_SOURCE_UE4);
                } else {
                    ROS_WARN("Unknown New Source Name");
                    ROS_WARN("    ydlidar:  use real YDLidar data");
                    ROS_WARN("    ue4:      use simulated data from UE4 SIM");
                }
            } else {
                ROS_WARN("Missing New Source Name");
                ROS_WARN("    ydlidar:  use real YDLidar data");
                ROS_WARN("    ue4:      use simulated data from UE4 SIM");
            }
        } else {
            ROS_WARN("Unknown LIDAR command");
            printLIDARHelper();
        }
    } catch (...) {
        ROS_WARN("ERROR happened while processing LIDAR command");
        printLIDARHelper();
    }
    return true;
}

bool ConsoleInputManager::handleMapCommands() {
    try {
        std::string cmdType = currentCommand_.second.at(0);
        if (cmdType == "info") {
            //! @todo
            octomapClient_->printDebugInfo();
            ROS_WARN("TODO SLAM");
        } else {
            ROS_WARN("Unknown MAP command");
        }
    } catch (...) {
        ROS_WARN("ERROR happened while processing MAP command");
    }
    return true;
}

void ConsoleInputManager::printCNCHelper() {
    ROS_WARN("CNC Commands: [required] <optional>");
    ROS_WARN("    arm:                                  Arm the vehicle motor");
    ROS_WARN("    takeoff [altitude]:                   Takeoff");
    ROS_WARN("    chmod [mode name]:                    Change flight mode");
    ROS_WARN("    rtl:                                  Return to land");
    ROS_WARN("    velocity [Speed]:                     Set max velocity");
    ROS_WARN("    yaw [Heading] <Distance> <Altitude>:  Change yaw / go to heading");
    ROS_WARN("    info:                                 Print information");
    ROS_WARN("    quit:                                 Quit the node");
}

void ConsoleInputManager::printRSCHelper() {
    ROS_WARN("RSC Commands: [required] <optional>");
    ROS_WARN("    info:                                 Print information");
    ROS_WARN("    chsrc:                                Change data source");
    ROS_WARN("    range [max(mm)] [min(mm)]             Set range");
    ROS_WARN("    range cancel                          Cancel range");
}

void ConsoleInputManager::printOACHelper() {
    ROS_WARN("OAC Commands: [required] <optional>");
    ROS_WARN("    on:           Resume OAC");
    ROS_WARN("    off:          Pause OAC");
    ROS_WARN("    info:         Print information");
}

void ConsoleInputManager::printLIDARHelper() {
    ROS_WARN("LIDAR Commands: [required] <optional>");
    ROS_WARN("    info:                                 Print information");
    ROS_WARN("    chsrc:                                Change data source");
}

void ConsoleInputManager::printQuickHelper() {
    ROS_WARN("Quick Commands: [required] <optional>");
    ROS_WARN("    w [Distance] <Altitude>:         Go to North");
    ROS_WARN("    s [Distance] <Altitude>:         Go to South");
    ROS_WARN("    a [Distance] <Altitude>:         Go to West");
    ROS_WARN("    d [Distance] <Altitude>:         Go to East");
}

void ConsoleInputManager::printFormatHelper() {
    ROS_WARN("Command Format:  no < > in actually command");
    ROS_WARN("    <Module Name> <arg1> <arg2> ...");
}

void ConsoleInputManager::printModuleHelper() {
    ROS_WARN("Module Names:");
    ROS_WARN("    CNC:    Command And Control Module");
    ROS_WARN("    OAC:    Obstacle Avoidance Algorithm Controller");
    ROS_WARN("    RSC:    Realsense Camera HS Interface");
    ROS_WARN("    LIDAR:  Lidar Sensor HS Interface");
    ROS_WARN("    !:      Quick Commands");
}

}  // namespace IO
