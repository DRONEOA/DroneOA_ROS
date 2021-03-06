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

#include <droneoa_ros/CheckGetNewInput.h>
#include <sstream>
#include <string>
#include <droneoa_ros/HWI/ConsoleInputManager.hpp>
#include <droneoa_ros/HWI/Utils/CNCUtils.hpp>
#include <droneoa_ros/Utils/GeneralUtils.hpp>

namespace IO {

ConsoleInputManager::ConsoleInputManager(bool* masterSwitch) : masterSwitch_(masterSwitch),
        mpCNC(nullptr), mpRSC(nullptr), oac_(nullptr), mpLidar(nullptr), mIsBuildingQueue(false) {}

ConsoleInputManager::~ConsoleInputManager() {
    if (thread_watch_command_) delete thread_watch_command_;
}

bool ConsoleInputManager::init(CNC::CNCInterface* cnc, Depth::RSC *rsc, OAC::OAController *oac,
        Lidar::LidarGeneric *lidar, OAC::CMDRunner *runner) {
    mpCNC = cnc;
    mpRSC = rsc;
    oac_ = oac;
    mpLidar = lidar;
    thread_watch_command_ = new boost::thread(boost::bind(&ConsoleInputManager::watchCommandThread, this));
    mpParser = new OAC::CMDParser(cnc, runner);
    return thread_watch_command_ ? true : false;
}

void ConsoleInputManager::command_callback(const std_msgs::String::ConstPtr& msg) {
    std_msgs::String inputCMD = *msg;
    ros::ServiceClient client = mNodeHandle.serviceClient<droneoa_ros::CheckGetNewInput>(
            INPUT_MSG_REQUEST_SERVICE_NAME);
    droneoa_ros::CheckGetNewInput srv;
    srv.request.module_name = MAIN_NODE_ACCEPTED_MODULE_NAMES;
    if (client.call(srv)) {
        ROS_DEBUG("[MainNode] Input MATCH: %s", srv.response.msg.c_str());
        if (!parseAndExecuteConsole(srv.response.msg)) {
            ROS_WARN("[MainNode] Command from console service: %s. Ignored", srv.response.msg.c_str());
        }
    } else {
        ROS_DEBUG("[MainNode] New input: No match or Expired");
    }
}

void ConsoleInputManager::watchCommandThread() {
    auto rate = ros::Rate(OAC_REFRESH_FREQ);
    auto node = boost::make_shared<ros::NodeHandle>();
    auto cmd_sub =
        node->subscribe<std_msgs::String>(NEW_INPUT_FLAG_TOPIC_NAME, 1,
            boost::bind(&ConsoleInputManager::command_callback, this, _1));

    while (ros::ok()) {
        ros::spinOnce();
        rate.sleep();
    }
}

bool ConsoleInputManager::parseAndExecuteConsole(std::string cmd) {
    GeneralUtility::removeSpaces(&cmd);
    if (cmd.empty()) {
        printModuleHelper();
        return true;
    }
    if (!splitModuleCommand(cmd)) {
        ROS_WARN("[MainNode] Missing Module Name");
        printModuleHelper();
        return true;  // Forwarded, may still be accepted by other nodes
    }
    mGeneratedCMDQueue.clear();
    if (buildCommandQueue()) {
        return mpParser->parseCMDQueue(mGeneratedCMDQueue);
    }
    return false;
}

bool ConsoleInputManager::splitModuleCommand(std::string cmd) {
    currentCommand_.first = "INVALID";
    currentCommand_.second.clear();
    std::string token;
    std::istringstream tokenStream(cmd);

    while (std::getline(tokenStream, token, CONSOLE_DELIMITER)) {
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

bool ConsoleInputManager::buildCommandQueue() {
    if (currentCommand_.first == "cnc") {
        return buildCNCCommands();
    } else if (currentCommand_.first == "oac") {
        return buildOACCommands();
    } else if (currentCommand_.first == "rsc") {
        if (!ENABLE_RSC) {
            ROS_WARN("[MainNode] RSC is not enabled !!!");
            return false;
        }
        return buildRSCCommands();
    } else if (currentCommand_.first == "lidar") {
        if (!ENABLE_LIDAR) {
            ROS_WARN("[MainNode] LIDAR is not enabled !!!");
            return false;
        }
        return buildLIDARCommands();
    } else if (currentCommand_.first == "dp") {
        return buildDPCommands();
    } else if (currentCommand_.first == "!") {
        return buildQuickCommands();
    } else if (currentCommand_.first == "start") {
        return buildQueueCommands();
    } else if (currentCommand_.first == "cancel") {
        mGeneratedCMDQueue.push_back({Command::CMD_QUEUE_TYPES::CMD_CANCEL_QUEUE, ""});
        return true;
    } else if (currentCommand_.first == "delay") {
        if (mIsBuildingQueue && currentCommand_.second.size() > 0) {
            mGeneratedCMDQueue.push_back({Command::CMD_QUEUE_TYPES::CMD_DELAY_MSEC, currentCommand_.second.at(0)});
            return true;
        }
        mIsBuildingQueue = false;
        ROS_ERROR("[MainNode] Delay is only valid in command queue: delay [time in ms]");
        return false;
    } else if (currentCommand_.first == "until") {
        if (mIsBuildingQueue && currentCommand_.second.size() > 0) {
            std::string dataStr = "";
            for (auto tmp : currentCommand_.second) {
                dataStr = dataStr + tmp + " ";
            }
            mGeneratedCMDQueue.push_back({Command::CMD_QUEUE_TYPES::CMD_UNTIL, dataStr});
            //! @todo will support more commands in the future (e.g. velocity greater than "velgt [num]")
            return true;
        }
        mIsBuildingQueue = false;
        ROS_ERROR("[MainNode] Until is only valid in command queue: until [mode]");
        return false;
    } else if (currentCommand_.first == "then") {
        ROS_WARN("[MainNode] Redundent THEN Detected - ignore");
        return true;
    } else if (currentCommand_.first == HELP_ACCEPTED_MODULE_NAMES) {
        printModuleHelper();
        return true;
    } else if (currentCommand_.first == QUIT_ACCEPTED_MODULE_NAMES) {
        ROS_INFO("::QUIT::");
        oac_->masterSwitch(false);
        *masterSwitch_ = false;
        return true;
    } else if (currentCommand_.first == VERBOSE_ACCEPTED_MODULE_NAMES) {
        if (currentCommand_.second.size() > 0) {
            GeneralUtility::setVerbosityLevel(currentCommand_.second[0]);
            return true;
        }
        ROS_ERROR("[PM] New verbosity level unspecified");
        return false;
    } else {
        ROS_WARN("[MainNode] Unknown Module Name: %s", currentCommand_.first.c_str());
        return true;  // Forwarded, may still be accepted by other nodes
    }
}

bool ConsoleInputManager::buildQueueCommands() {
    std::vector<std::string> inputCmd = currentCommand_.second;
    if (inputCmd.size() == 0) {
        ROS_WARN("[MainNode] Empty Command Queue");
        printQueueHelper();
        return false;
    }
    uint8_t stepCount = 0;  // 0: module name; >=1:entries;
    currentCommand_.second.clear();
    mIsBuildingQueue = true;
    for (std::string entry : inputCmd) {
        // THEN, END, UNTIL indicate end of previous command
        //   Parse the queue then clean the currentCommand_ (which is used by single command bulider)
        if (entry == "then" || entry == "end" || entry == "until") {
            stepCount = 0;
            if (!buildCommandQueue()) {
                printQueueHelper();
                mIsBuildingQueue = false;
                return false;
            }
            currentCommand_.first = entry;
            currentCommand_.second.clear();
            // Until is also first entry of a command, so do not skip
            if (entry != "until") continue;
        }
        // First entry of command indicate target module name
        if (stepCount == 0) {
            currentCommand_.first = entry;
            stepCount++;
            continue;
        }
        // Store parameters of the command
        currentCommand_.second.push_back(entry);
    }
    mIsBuildingQueue = false;
    ROS_INFO("[MainNode] QUEUE BUILD DONE");
    return true;
}

bool ConsoleInputManager::buildCNCCommands() {
    try {
        std::string cmdType = currentCommand_.second.at(0);
        if (cmdType == "arm") {
            ROS_INFO("::ARM::");
            mGeneratedCMDQueue.push_back({Command::CMD_QUEUE_TYPES::CMD_ARM, FLT_MODE_GUIDED});
        } else if (cmdType == "takeoff") {
            ROS_INFO("::TAKEOFF::");
            mGeneratedCMDQueue.push_back({Command::CMD_QUEUE_TYPES::CMD_TAKEOFF, currentCommand_.second.at(1)});
        } else if (cmdType == "chmod") {
            std::string currentMode = mpCNC->getMode();
            std::string newMode = currentCommand_.second.at(1);
            if (!mpCNC->checkFModeExist(newMode)) {
                ROS_ERROR("::ChangeMode:: Flight Mode Does Not Exist !!!");
                return false;
            }
            GeneralUtility::toUpperCaseStr(&newMode);
            ROS_INFO("::ChangeMode %s -> %s::", currentMode.c_str(), newMode.c_str());
            mGeneratedCMDQueue.push_back({Command::CMD_QUEUE_TYPES::CMD_CHMOD, currentCommand_.second.at(1)});
        } else if (cmdType == "land") {
            ROS_INFO("::LAND::");
            // @todo accept Cancel altitude
            mGeneratedCMDQueue.push_back({Command::CMD_QUEUE_TYPES::CMD_LAND, ""});
        } else if (cmdType == "rtl") {
            ROS_INFO("::RTL::");
            mGeneratedCMDQueue.push_back({Command::CMD_QUEUE_TYPES::CMD_CHMOD, FLT_MODE_RTL});
        } else if (cmdType == "velocity") {
            float vel = std::stof(currentCommand_.second.at(1));
            ROS_INFO("::SET MAX VELOCITY -> %f::", vel);
            if (vel <= 0.0) {
                ROS_ERROR("::SET MAX VELOCITY:: Invalid Speed Setting");
                return false;
            }
            mGeneratedCMDQueue.push_back({Command::CMD_QUEUE_TYPES::CMD_SET_MAX_VELOCITY, std::to_string(vel)});
        } else if (cmdType == "yaw") {
            float yawAngle = std::stof(currentCommand_.second.at(1));
            ROS_INFO("::SET YAW -> %f::", yawAngle);
            mGeneratedCMDQueue.push_back({Command::CMD_QUEUE_TYPES::CMD_SET_YAW, std::to_string(yawAngle)});
            if (currentCommand_.second.size() >= 3) {
                float dist = std::stof(currentCommand_.second.at(2));
                float alt = mpCNC->getRelativeAltitude();
                std::string dataStr = std::to_string(yawAngle) + " " + std::to_string(dist);
                if (currentCommand_.second.size() >= 4) {
                    alt = std::stof(currentCommand_.second.at(3));
                    dataStr = dataStr + " " + std::to_string(alt);
                }
                ROS_INFO("::GOTO YAW -> yaw:%f dist:%f alt:%f::", yawAngle, dist, alt);
                mGeneratedCMDQueue.push_back({Command::CMD_QUEUE_TYPES::CMD_GOTO_HEADING, dataStr});
            }
        } else if (cmdType == "climb") {
            float deltaAlt = std::stof(currentCommand_.second.at(1));
            if (deltaAlt < 0.0f) {
                throw 1;
            }
            ROS_INFO("::CLIMB -> -%f::", deltaAlt);
            mGeneratedCMDQueue.push_back({Command::CMD_QUEUE_TYPES::CMD_CLIMB, std::to_string(deltaAlt)});
        } else if (cmdType == "descent") {
            float deltaAlt = std::stof(currentCommand_.second.at(1));
            if (deltaAlt < 0.0f) {
                throw 1;
            }
            ROS_INFO("::DECENT -> %f::", deltaAlt);
            mGeneratedCMDQueue.push_back({Command::CMD_QUEUE_TYPES::CMD_DESCEND, std::to_string(deltaAlt)});
        } else if (cmdType == "info") {
            GPSPoint tmpGPSPoint = mpCNC->getCurrentGPSPoint();
            LocalPoint localPose = mpCNC->getLocalPosition();
            geometry_msgs::Vector3 RPY = CNC::CNCUtility::quaternionToRPY(mpCNC->getIMUData().orientation);
            ROS_INFO(">>>>>>>>>> INFO START <<<<<<<<<<");
            ROS_INFO("[DISPLAY] gps: %f %f %f", tmpGPSPoint.mX, tmpGPSPoint.mY, tmpGPSPoint.mZ);
            ROS_INFO("[DISPLAY] local: %f %f %f", localPose.mX, localPose.mY, localPose.mZ);
            ROS_INFO("[DISPLAY] altitude: %f", mpCNC->getRelativeAltitude());
            ROS_INFO("[DISPLAY] mode: %s", mpCNC->getMode().c_str());
            ROS_INFO("[DISPLAY] voltage: %f", mpCNC->getBatteryVoltage());
            ROS_INFO("[DISPLAY] orientation: P:%f, R:%f, Y%f", RPY.x*57.3,
                                                        RPY.y*57.3,
                                                        RPY.z*57.3);
            ROS_INFO("[HUD] heading: %d", mpCNC->getHUDData().heading);
            ROS_INFO("[HUD] airspeed: %f", mpCNC->getHUDData().airspeed);
            ROS_INFO("[HUD] groundspeed: %f", mpCNC->getHUDData().groundspeed);
            ROS_INFO("[HUD] altitude: %f", mpCNC->getHUDData().altitude);
            ROS_INFO("[HUD] climb: %f", mpCNC->getHUDData().climb);
            ROS_INFO("[HUD] throttle: %f", mpCNC->getHUDData().throttle);
            ROS_INFO(">>>>>>>>>> INFO END <<<<<<<<<<");
        } else if (cmdType == "quit") {
            ROS_INFO("::QUIT::");
            oac_->masterSwitch(false);
            *masterSwitch_ = false;
        } else {
            ROS_ERROR("[MainNode] Unknown CNC command: %s", cmdType.c_str());
            printCNCHelper();
        }
    } catch (...) {
        ROS_ERROR("[MainNode] ERROR happened while processing CNC command");
        printCNCHelper();
    }
    return true;
}

bool ConsoleInputManager::buildOACCommands() {
    try {
        std::string cmdType = currentCommand_.second.at(0);
        if (cmdType == "on") {
            ROS_INFO("::OAC->ON::");
            mGeneratedCMDQueue.push_back({Command::CMD_QUEUE_TYPES::CMD_OAC_SWITCH, "1"});
        } else if (cmdType == "off") {
            ROS_INFO("::OAC->OFF::");
            mGeneratedCMDQueue.push_back({Command::CMD_QUEUE_TYPES::CMD_OAC_SWITCH, "0"});
        } else if (cmdType == "info") {
            ROS_INFO(">>>>>>>>>> INFO START <<<<<<<<<<");
            ROS_INFO("[OAC] status: %s", oac_->getStatus().c_str());
            ROS_INFO(">>>>>>>>>> INFO END <<<<<<<<<<");
        } else {
            ROS_ERROR("[MainNode] Unknown OAC command");
            printOACHelper();
        }
    } catch (...) {
        ROS_ERROR("[MainNode] ERROR happened while processing OAC command");
        printOACHelper();
    }
    return true;
}

//! @note Keep this handler small !!!
bool ConsoleInputManager::buildQuickCommands() {
    try {
        std::string cmdType = currentCommand_.second.at(0);
        if (cmdType == "w") {
            ROS_INFO("::GO NORTH::");
            float dist = std::stof(currentCommand_.second.at(1));
            float alt = mpCNC->getRelativeAltitude();
            std::string dataStr = std::to_string(dist) + " 0";
            if (currentCommand_.second.size() >= 3) {
                alt = std::stof(currentCommand_.second.at(2));
                dataStr = dataStr + " " + std::to_string(alt);
            }
            mGeneratedCMDQueue.push_back({Command::CMD_QUEUE_TYPES::CMD_GOTO_RELATIVE, dataStr});
            mGeneratedCMDQueue.push_back({Command::CMD_QUEUE_TYPES::CMD_SET_YAW, "0"});
        } else if (cmdType == "s") {
            ROS_INFO("::GO SOUTH::");
            float dist = std::stof(currentCommand_.second.at(1));
            float alt = mpCNC->getRelativeAltitude();
            std::string dataStr = std::to_string(-dist) + " 0";
            if (currentCommand_.second.size() >= 3) {
                alt = std::stof(currentCommand_.second.at(2));
                dataStr = dataStr + " " + std::to_string(alt);
            }
            mGeneratedCMDQueue.push_back({Command::CMD_QUEUE_TYPES::CMD_GOTO_RELATIVE, dataStr});
            mGeneratedCMDQueue.push_back({Command::CMD_QUEUE_TYPES::CMD_SET_YAW, "180"});
        } else if (cmdType == "a") {
            ROS_INFO("::GO WEST::");
            float dist = std::stof(currentCommand_.second.at(1));
            float alt = mpCNC->getRelativeAltitude();
            std::string dataStr = "0 " + std::to_string(-dist);
            if (currentCommand_.second.size() >= 3) {
                alt = std::stof(currentCommand_.second.at(2));
                dataStr = dataStr + " " + std::to_string(alt);
            }
            mGeneratedCMDQueue.push_back({Command::CMD_QUEUE_TYPES::CMD_GOTO_RELATIVE, dataStr});
            mGeneratedCMDQueue.push_back({Command::CMD_QUEUE_TYPES::CMD_SET_YAW,  "270"});
        } else if (cmdType == "d") {
            ROS_INFO("::GO EAST::");
            float dist = std::stof(currentCommand_.second.at(1));
            float alt = mpCNC->getRelativeAltitude();
            std::string dataStr = "0 " + std::to_string(dist);
            if (currentCommand_.second.size() >= 3) {
                alt = std::stof(currentCommand_.second.at(2));
                dataStr = dataStr + " " + std::to_string(alt);
            }
            mGeneratedCMDQueue.push_back({Command::CMD_QUEUE_TYPES::CMD_GOTO_RELATIVE, dataStr});
            mGeneratedCMDQueue.push_back({Command::CMD_QUEUE_TYPES::CMD_SET_YAW, "90"});
        } else if (cmdType == "t") {
            ROS_INFO("::CUSTOM TEST CMD::");
            /* ! @node Add your custom command here, trigger with "! t" command */
            mpCNC->pushLocalENUWaypoint(LocalPoint(0, 0, 3));
        } else {
            ROS_ERROR("[MainNode] Unknown Quick command");
            printQuickHelper();
        }
    } catch (...) {
        ROS_ERROR("[MainNode] ERROR happened while processing Quick command");
        printQuickHelper();
    }
    return true;
}

bool ConsoleInputManager::buildRSCCommands() {
    try {
        std::string cmdType = currentCommand_.second.at(0);
        if (cmdType == "info") {
            ROS_INFO(">>>>>>>>>> INFO START <<<<<<<<<<");
            mpRSC->printImgInfo();
            ROS_INFO(">>>>>>>>>> INFO END <<<<<<<<<<");
        } else if (cmdType == "chsrc") {
            if (currentCommand_.second.size() >= 2) {
                std::string srcName = currentCommand_.second.at(1);
                if (srcName == "rsc") {
                    ROS_INFO("::RSC SRC -> RSC::");
                    mpRSC->changeDepthSource(DEPTH_SOURCE_RSC);
                    mpRSC->changePC2Source(PC_SOURCE_RSC);
                } else if (srcName == "ue4") {
                    ROS_INFO("::RSC SRC -> UE4::");
                    mpRSC->changeDepthSource(DEPTH_SOURCE_UE4);
                    mpRSC->changePC2Source(PC_SOURCE_UE4);
                } else {
                    ROS_WARN("::RSC SRC:: Unknown New Source Name");
                    ROS_WARN("    rsc:      use real realsense camera data");
                    ROS_WARN("    ue4:      use simulated data from UE4 SIM");
                }
            } else {
                ROS_WARN("::RSC SRC:: Unknown New Source Name");
                ROS_WARN("    rsc:      use real realsense camera data");
                ROS_WARN("    ue4:      use simulated data from UE4 SIM");
            }
        } else if (cmdType == "range") {
            std::string cancel = currentCommand_.second.at(1);
            if (currentCommand_.second.size() == 3) {
                ROS_INFO("::RSC Range Set::");
                float max = std::stof(currentCommand_.second.at(1));
                float min = std::stof(currentCommand_.second.at(2));
                if (max < min || min < 0) {
                    throw 1;
                }
                mpRSC->setRange(min, max);
                mpRSC->setRangeSwitch(true);
            } else if (currentCommand_.second.size() == 2 && cancel == "cancel") {
                ROS_INFO("::RSC Range Cancel::");
                mpRSC->setRangeSwitch(false);
            } else {
                ROS_WARN("::RSC Range:: Unknown Range Operation");
                ROS_WARN("    [max(mm)] [min(mm)]       Set range");
                ROS_WARN("    cancel                    Cancel range");
            }
        } else {
            ROS_ERROR("[MainNode] Unknown RSC command");
            printRSCHelper();
        }
    } catch (...) {
        ROS_ERROR("[MainNode] ERROR happened while processing RSC command");
        printRSCHelper();
    }
    return true;
}

bool ConsoleInputManager::buildLIDARCommands() {
    try {
        std::string cmdType = currentCommand_.second.at(0);
        if (cmdType == "info") {
            ROS_INFO(">>>>>>>>>> INFO START <<<<<<<<<<");
            mpLidar->printLidarInfo();
            ROS_INFO(">>>>>>>>>> INFO END <<<<<<<<<<");
        } else if (cmdType == "chsrc") {
            if (currentCommand_.second.size() >= 2) {
                std::string srcName = currentCommand_.second.at(1);
                if (srcName == "ydlidar") {
                    ROS_INFO("::LIDAR SRC -> YDLIDAR::");
                    mpLidar->changeLidarSource(LIDAR_SOURCE_YDLIDAR);
                } else if (srcName == "ue4") {
                    ROS_INFO("::LIDAR SRC -> UE4::");
                    mpLidar->changeLidarSource(LIDAR_SOURCE_UE4);
                } else {
                    ROS_WARN("::LIDAR SRC:: Unknown New Source Name");
                    ROS_WARN("    ydlidar:  use real YDLidar data");
                    ROS_WARN("    ue4:      use simulated data from UE4 SIM");
                }
            } else {
                ROS_WARN("::LIDAR SRC:: Missing New Source Name");
                ROS_WARN("    ydlidar:  use real YDLidar data");
                ROS_WARN("    ue4:      use simulated data from UE4 SIM");
            }
        } else {
            ROS_ERROR("[MainNode] Unknown LIDAR command");
            printLIDARHelper();
        }
    } catch (...) {
        ROS_ERROR("[MainNode] ERROR happened while processing LIDAR command");
        printLIDARHelper();
    }
    return true;
}

bool ConsoleInputManager::buildDPCommands() {
    try {
        std::string cmdType = currentCommand_.second.at(0);
        if (cmdType == "list") {
            msDP.printAllEntry();
        } else if (cmdType == "print") {
            std::string srcName = currentCommand_.second.at(1);
            if (srcName == "all") {
                msDP.printAllEntryWithData();
                return true;
            }
            ROS_INFO("[DP] Data at: %s = %s", srcName.c_str(), msDP.getDataAsString(srcName).c_str());
        } else {
            ROS_ERROR("[MainNode] Unknown DataPool command");
            printDPHelper();
        }
    } catch (...) {
        ROS_ERROR("[MainNode] ERROR happened while processing DataPool command");
        printDPHelper();
    }
    return true;
}

void ConsoleInputManager::printCNCHelper() {
    ROS_WARN("CNC Commands: [required] <optional>");
    ROS_WARN("    arm:                                  Arm the vehicle motor");
    ROS_WARN("    takeoff [altitude]:                   Takeoff");
    ROS_WARN("    land:                                 Land at current location");
    ROS_WARN("    chmod [mode name]:                    Change flight mode");
    ROS_WARN("    rtl:                                  Return to land");
    ROS_WARN("    velocity [Speed]:                     Set max velocity");
    ROS_WARN("    yaw [Heading] <Distance> <Altitude>:  Change yaw / go to heading");
    ROS_WARN("    climb [Altitude]:                     Climb based on relative altitude");
    ROS_WARN("    descent [Altitude]:                   Descent based on relative altitude");
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

void ConsoleInputManager::printDPHelper() {
    ROS_WARN("DataPool Commands: [required] <optional>");
    ROS_WARN("    list:                                 List existing datapool entries");
    ROS_WARN("    print [entry name / ALL]:             Print value of given entry or all");
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
    ROS_WARN("Main Node Module Names:");
    ROS_WARN("    CNC:            Command And Control Module");
    ROS_WARN("    OAC:            Obstacle Avoidance Algorithm Controller");
    ROS_WARN("    RSC:            Realsense Camera HS Interface");
    ROS_WARN("    LIDAR:          Lidar Sensor HS Interface");
    ROS_WARN("    DP:             DataPool Tools");
    ROS_WARN("    !:              Quick Commands");
    ROS_WARN("    [Package Name]: Installed Extra Packages");
    ROS_WARN("Queue Commands:");
    ROS_WARN("    START [CMD] THEN [CMD] TEHN [CMD] ... END");
}

void ConsoleInputManager::printQueueHelper() {
    ROS_WARN("Command Queue Helper:");
    ROS_WARN("Usage: START [CMD] THEN [CMD] TEHN [CMD] ... END");
    ROS_WARN("Supported Commands [CMD]:");
    ROS_WARN("    CNC:    all expect: quit");
    ROS_WARN("    OAC:    on/off switch");
    ROS_WARN("    RSC:    none WIP");
    ROS_WARN("    Lidar:  none WIP");
    ROS_WARN("    !:      all");
    ROS_WARN("    UNTIL:  alt(gt lt eq), clrwp, arrwp");
    ROS_WARN("    DELAY:  time in msec");
}

}  // namespace IO
