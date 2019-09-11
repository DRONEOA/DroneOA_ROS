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
 * Written by Bohan Shi <b34shi@edu.uwaterloo.ca>, August 2019
 */

#include <ros/ros.h>

#include <cstdlib>
#include <iomanip>
#include <sstream>

#include <droneoa_ros/CNCInterface.hpp>
#include <droneoa_ros/RSCInterface.hpp>
#include <droneoa_ros/LidarInterface.hpp>
#include <droneoa_ros/Utils.hpp>

float getFloatCmdArg(std::stringstream& ss) {
    float result = 0.0;
    if (!ss.eof()) {
        std::string tmp;
        ss >> tmp;
        try {
            result = std::stof(tmp);
        } catch (...) {
            result = 0.0;
        }
    }
    return result;
}

int main(int argc, char **argv) {
    // Setup Refresh Rate
    int rate = 10;

    ros::init(argc, argv, "mavros_takeoff");
    ros::NodeHandle n;

    ros::Rate r(rate);

    // Interface Instance
    CNCInterface cnc;
    RSCInterface rsc;
    LidarInterface lidar;
    cnc.init(n, r);
    if (ENABLE_RSC) {
        rsc.init(n, r);
    }
    if (ENABLE_LIDAR) {
        lidar.init(n, r);
    }

    // Command Paser
    // Only For Testing
    // @todo Do we need a command line based direct control interface and a command queue in final product ?
    std::string commandIn;
    while (std::getline(std::cin, commandIn)) {
        std::stringstream ss;
        ss << commandIn;
        std::string cmdType;
        ss >> cmdType;

        if (cmdType == "quit") {
            break;
        } else if (cmdType == "arm") {
            cnc.setMode(FLT_MODE_GUIDED);
            cnc.armVehicle();
        } else if (cmdType == "takeoff") {
            if (!cnc.isArmed()) {
                std::cerr << "VEHICLE NOT ARMED !!!" << std::endl;
                continue;
            }
            cnc.takeoff(getFloatCmdArg(ss));
            sleep(10);
        } else if (cmdType == "w") {
            float dist = getFloatCmdArg(ss);
            float alt = getFloatCmdArg(ss);
            if (alt == 0.0) {
                alt = cnc.getTargetAltitude();
            }
            cnc.gotoRelative(dist, 0, alt);
        } else if (cmdType == "s") {
            float dist = getFloatCmdArg(ss);
            float alt = getFloatCmdArg(ss);
            if (alt == 0.0) {
                alt = cnc.getTargetAltitude();
            }
            cnc.gotoRelative(-dist, 0, alt);
        } else if (cmdType == "a") {
            float dist = getFloatCmdArg(ss);
            float alt = getFloatCmdArg(ss);
            if (alt == 0.0) {
                alt = cnc.getTargetAltitude();
            }
            cnc.gotoRelative(0, -dist, alt);
        } else if (cmdType == "d") {
            float dist = getFloatCmdArg(ss);
            float alt = getFloatCmdArg(ss);
            if (alt == 0.0) {
                alt = cnc.getTargetAltitude();
            }
            cnc.gotoRelative(0, dist, alt);
        } else if (cmdType == "rtl") {
            cnc.setMode(FLT_MODE_RTL);
        } else if (cmdType == "land") {
            cnc.land(1);
        } else if (cmdType == "info") {
            GPSPoint tmpGPSPoint = cnc.getCurrentGPSPoint();
            std::cout << ">>>>>>>>>> INFO START <<<<<<<<<<" << std::endl;
            std::cout << "[DISPLAY] gps: " << std::fixed << std::setprecision(6) << tmpGPSPoint.latitude_<< " "
                                           << tmpGPSPoint.longitude_<< " " << std::endl;
            std::cout << "[DISPLAY] altitude: " << cnc.getRelativeAltitude() << std::endl;
            std::cout << "[DISPLAY] mode: " << cnc.getMode() << std::endl;
            std::cout << "[DISPLAY] voltage: " << cnc.getBatteryVoltage() << std::endl;
            std::cout << "[DISPLAY] orientation: " << cnc.getIMUData().orientation.x << ", "
                                                   << cnc.getIMUData().orientation.y << ", "
                                                   << cnc.getIMUData().orientation.z << std::endl;
            std::cout << "[HUD] heading: " << cnc.getHUDData().heading << std::endl;
            std::cout << "[HUD] airspeed: " << cnc.getHUDData().airspeed << std::endl;
            std::cout << "[HUD] groundspeed: " << cnc.getHUDData().groundspeed << std::endl;
            std::cout << "[HUD] altitude: " << cnc.getHUDData().altitude << std::endl;
            std::cout << "[HUD] climb: " << cnc.getHUDData().climb << std::endl;
            std::cout << "[HUD] throttle: " << cnc.getHUDData().throttle << std::endl;
            if (ENABLE_RSC) {
                rsc.printImgInfo();
            }
            if (ENABLE_LIDAR) {
                lidar.printLidarInfo();
            }
            std::cout << ">>>>>>>>>> INFO END  <<<<<<<<<<" << std::endl;
        } else if (cmdType == "yaw") {
            float yawAngle = getFloatCmdArg(ss);
            float dist = getFloatCmdArg(ss);
            if (dist == 0.0) {
                dist = 1000;
            }
            float alt = getFloatCmdArg(ss);
            if (alt == 0.0) {
                alt = cnc.getTargetAltitude();
            }
            cnc.gotoHeading(yawAngle, dist, alt);
            cnc.setYaw(getBearing(cnc.getCurrentGPSPoint(), cnc.getTargetWaypoint()));
        } else if (cmdType == "velocity") {
            float vel = getFloatCmdArg(ss);
            if (vel == 0.0) {
                std::cerr << "Invalid Speed Setting" << std::endl;
                continue;
            }
            cnc.setMaxSpeed(1, vel, 0);
        } else if (commandIn.front() == 'r') {
            // @todo refactor this command to: rsc range [max] [min]
            commandIn = commandIn.substr(1);
            if (commandIn.front() == 'c') {
                rsc.setRangeSwitch(false);
            } else {
                size_t mid = commandIn.find('-');
                int min = std::stoi(commandIn.substr(0, mid), 0);
                int max = std::stoi(commandIn.substr(mid+1), 0);
                rsc.setRange(min, max);
                rsc.setRangeSwitch(true);
            }
        } else {
            std::cout << ">>>>>>>>>> HELP START <<<<<<<<<<" << std::endl;
            std::cout << "+ COMMAND HELP LIST" << std::endl;
            std::cout << "+ Argument [] is required, <> is optional" << std::endl;
            std::cout << "- quit                       - quit" << std::endl;
            std::cout << "- w [dist] <alt>             - move north" << std::endl;
            std::cout << "- s [dist] <alt>             - move south" << std::endl;
            std::cout << "- a [dist] <alt>             - move west" << std::endl;
            std::cout << "- d [dist] <alt>             - move east" << std::endl;
            std::cout << "- rtl                        - return to home" << std::endl;
            std::cout << "- land                       - land" << std::endl;
            std::cout << "- takeoff [alt]              - takeoff to altitude" << std::endl;
            std::cout << "- arm                        - arm motor" << std::endl;
            std::cout << "- info                       - print information" << std::endl;
            std::cout << "- yaw [heading] <dist> <alt> - fly heading" << std::endl;
            std::cout << "- velocity [speed]           - set max speed" << std::endl;
            std::cout << "- r[min]-[max]               - set range" << std::endl;
            std::cout << "- rc                         - cancel range" << std::endl;
            std::cout << ">>>>>>>>>> HELP END  <<<<<<<<<<" << std::endl;
        }
    }

    while (n.ok()) {
      ros::spinOnce();
      r.sleep();
    }

    return 0;
}
