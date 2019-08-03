#include <cstdlib>

#include <ros/ros.h>

#include <droneoa_ros/CNCInterface.hpp>
#include <droneoa_ros/PDN.hpp>

bool gotoGlobal(CNCInterface cnc, float x_lat, float y_long, float z_alt) {
  // TODO: check Guided mode
  bool rel = false;
  rel = cnc.clearWaypoint();
  rel = cnc.pushWaypoints(x_lat, y_long, z_alt);
  return rel;
}

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
    std::string line;
    while (std::getline(std::cin, line)){
        if (line == "q") {
            break;
        } else if (line == "a") {
            gotoGlobal(cnc, 37.619595, -122.377170, 15);
        } else if (line == "b") {
            gotoGlobal(cnc, 37.620153, -122.375994, 15);
        } else if (line == "r") {
            cnc.setMode(FLT_MODE_RTL);
        } else if (line == "l") {
            cnc.land(10);
        }
    }

    while (n.ok())
    {
      ros::spinOnce();
      r.sleep();
    }

    return 0;
}