#include <cstdlib>
#include <iomanip>

#include <ros/ros.h>

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

    ////////////////////////////////////////////
    /////////////  GUIDED MODE  ////////////////
    ////////////////////////////////////////////
    cnc.setMode(FLT_MODE_GUIDED);

    ////////////////////////////////////////////
    /////////////////  ARM  ////////////////////
    ////////////////////////////////////////////
    cnc.armVehicle();

    ////////////////////////////////////////////
    ///////////////  TAKEOFF  //////////////////
    ////////////////////////////////////////////
    cnc.takeoff(10);

    ////////////////////////////////////////////
    ///////////////  DO STUFF  /////////////////
    ////////////////////////////////////////////
    sleep(10);
    std::string commandIn;
    while (std::cin >> commandIn){
        if (commandIn == "q") {
            break;
        } else if (commandIn == "w") {
            GPSPoint tmpPoint = getLocationMeter(cnc.getCurrentGPSPoint(), 100, 0);
            cnc.gotoGlobal(tmpPoint.latitude_, tmpPoint.longitude_, 10);
        } else if (commandIn == "s") {
            GPSPoint tmpPoint = getLocationMeter(cnc.getCurrentGPSPoint(), -100, 0);
            cnc.gotoGlobal(tmpPoint.latitude_, tmpPoint.longitude_, 10);
        } else if (commandIn == "a") {
            GPSPoint tmpPoint = getLocationMeter(cnc.getCurrentGPSPoint(), 0, 100);
            cnc.gotoGlobal(tmpPoint.latitude_, tmpPoint.longitude_, 10);
        } else if (commandIn == "d") {
            GPSPoint tmpPoint = getLocationMeter(cnc.getCurrentGPSPoint(), 0, -100);
            cnc.gotoGlobal(tmpPoint.latitude_, tmpPoint.longitude_, 10);
        } else if (commandIn == "r") {
            cnc.setMode(FLT_MODE_RTL);
        } else if (commandIn == "l") {
            cnc.land(10);
        } else if (commandIn == "d") {
            GPSPoint tmpGPSPoint = cnc.getCurrentGPSPoint();
            std::cout << "[DISPLAY] gps: " << std::fixed << std::setprecision(6) << tmpGPSPoint.latitude_<< " " << tmpGPSPoint.longitude_<< " " << std::endl;
            std::cout << "[DISPLAY] mode: " << cnc.getMode() << std::endl;
        }
    }

    while (n.ok())
    {
      ros::spinOnce();
      r.sleep();
    }

    return 0;
}