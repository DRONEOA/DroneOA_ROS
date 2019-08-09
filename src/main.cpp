/* Copyright (C) DroneOA Group - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 * Written by Bohan Shi <b34shi@edu.uwaterloo.ca>, August 2019
 */

#include <ros/ros.h>

#include <cstdlib>
#include <iomanip>

#include <droneoa_ros/CNCInterface.hpp>
#include <droneoa_ros/CNCInterfacePX4.hpp>
#include <droneoa_ros/Utils.hpp>

int main(int argc, char **argv) {
    // Setup Refresh Rate
    int rate = 10;

    ros::init(argc, argv, "mavros_takeoff");
    ros::NodeHandle n;

    ros::Rate r(rate);

    // Test PX4
    CNCInterfacePX4 cnc;
    cnc.init(n, r);

    bool ret = false;
    ret = cnc.armVehicle();
    if (ret) {
        ret = cnc.takeoff(2);
    }

    sleep(10);
    cnc.setMode(FLT_MODE_GUIDED_HOLD);
    sleep(10);

    if (ret) {
        ret = cnc.land();
    } else {
        ROS_ERROR("MASTER ERROR: Fail To Land !!!");
    }

    ROS_INFO("Mission Complete");

    // // Test Ardupilot
    // // Interface Instance
    // CNCInterface cnc;
    // cnc.init(n, r);

    // std::string commandIn;
    // while (std::cin >> commandIn) {
    //     if (commandIn == "q") {
    //         break;
    //     } else if (commandIn == "arm") {
    //         cnc.armVehicle();
    //     } else if (commandIn == "takeoff") {
    //         if (!cnc.isArmed()) {
    //             ROS_ERROR("VEHICLE NOT ARMED !!!");
    //             continue;
    //         }
    //         cnc.takeoff(10);
    //         sleep(10);
    //     } else if (commandIn == "w") {
    //         cnc.gotoRelative(100, 0, 10);
    //     } else if (commandIn == "s") {
    //         cnc.gotoRelative(-100, 0, 10);
    //     } else if (commandIn == "a") {
    //         cnc.gotoRelative(0, -100, 10);
    //     } else if (commandIn == "d") {
    //         cnc.gotoRelative(0, 100, 10);
    //     } else if (commandIn == "rtl") {
    //         cnc.setMode(FLT_MODE_RTL);
    //     } else if (commandIn == "land") {
    //         cnc.land(10);
    //     } else if (commandIn == "info") {
    //         GPSPoint tmpGPSPoint = cnc.getCurrentGPSPoint();
    //         std::cout << "[DISPLAY] gps: " << std::fixed << std::setprecision(6) << tmpGPSPoint.latitude_<< " "
    //             << tmpGPSPoint.longitude_<< " " << std::endl;
    //         std::cout << "[DISPLAY] altitude: " << cnc.getRelativeAltitude() << std::endl;
    //         std::cout << "[DISPLAY] mode: " << cnc.getMode() << std::endl;
    //         std::cout << "[DISPLAY] voltage: " << cnc.getBatteryVoltage() << std::endl;
    //     } else if (commandIn.front() == 'y') {
    //         if (commandIn.size() == 1) {
    //             commandIn = "y0";
    //         }
    //         cnc.gotoHeading(std::stoi(commandIn.substr(1)), 1000, 10);
    //         cnc.setYaw(getBearing(cnc.getCurrentGPSPoint(), cnc.getTargetWaypoint()));
    //     } else {
    //         std::cout << "+ COMMAND HELP LIST" << std::endl;
    //         std::cout << "- q         - quit" << std::endl;
    //         std::cout << "- w         - move north" << std::endl;
    //         std::cout << "- s         - move south" << std::endl;
    //         std::cout << "- a         - move west" << std::endl;
    //         std::cout << "- d         - move east" << std::endl;
    //         std::cout << "- rtl       - return to home" << std::endl;
    //         std::cout << "- land      - land" << std::endl;
    //         std::cout << "- takeoff   - takeoff" << std::endl;
    //         std::cout << "- arm       - arm motor" << std::endl;
    //         std::cout << "- info      - print information" << std::endl;
    //         std::cout << "- y[number] - fly heading" << std::endl;
    //     }
    // }

    ros::shutdown();

    return ret;
}
