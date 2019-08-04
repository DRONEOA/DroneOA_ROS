/* Copyright (C) DroneOA Group - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 * Written by Bohan Shi <b34shi@edu.uwaterloo.ca>, August 2019
 */

#include <ros/ros.h>

#include <cstdlib>
#include <iomanip>

#include <droneoa_ros/CNCInterface.hpp>
#include <droneoa_ros/PDN.hpp>
#include <droneoa_ros/Utils.hpp>

int main(int argc, char **argv) {
    // Setup Refresh Rate
    int rate = 10;

    ros::init(argc, argv, "mavros_takeoff");
    ros::NodeHandle n;

    ros::Rate r(rate);

    // Interface Instance
    CNCInterface cnc;
    cnc.init(n, r);

    cnc.setMode(FLT_MODE_GUIDED);

    cnc.armVehicle();

    cnc.takeoff(10);

    sleep(10);
    std::string commandIn;
    while (std::cin >> commandIn) {
        if (commandIn == "q") {
            break;
        } else if (commandIn == "w") {
            cnc.gotoRelative(100, 0, 10);
        } else if (commandIn == "s") {
            cnc.gotoRelative(-100, 0, 10);
        } else if (commandIn == "a") {
            cnc.gotoRelative(0, -100, 10);
        } else if (commandIn == "d") {
            cnc.gotoRelative(0, 100, 10);
        } else if (commandIn == "r") {
            cnc.setMode(FLT_MODE_RTL);
        } else if (commandIn == "l") {
            cnc.land(10);
        } else if (commandIn == "f") {
            GPSPoint tmpGPSPoint = cnc.getCurrentGPSPoint();
            std::cout << "[DISPLAY] gps: " << std::fixed << std::setprecision(6) << tmpGPSPoint.latitude_<< " "
                << tmpGPSPoint.longitude_<< " " << std::endl;
            std::cout << "[DISPLAY] mode: " << cnc.getMode() << std::endl;
        } else if (commandIn == "y") {
            cnc.setYaw(270);
        } else if (commandIn == "y0") {
            cnc.setYaw(getBearing(cnc.getCurrentGPSPoint(), cnc.getTargetWaypoint()));
        }
    }

    while (n.ok()) {
      ros::spinOnce();
      r.sleep();
    }

    return 0;
}
