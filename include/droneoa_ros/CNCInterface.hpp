#ifndef CNC_INTERFACE_
#define CNC_INTERFACE_

#include <string>

#include <ros/ros.h>

class CNCInterface {
public:
    CNCInterface();
    virtual ~CNCInterface();
    void init(ros::NodeHandle nh);

    // Mode Control
    bool setMode(std::string modeName);

    // Safety
    bool armVehicle();

    // Guided Flight Control
    bool takeoff(int targetAltitude);
    bool land(int fromAltitude);

    // Navigation
    bool setHome(float targetLatitude, float targetLongitude, float targetAltitude);

private:
    ros::NodeHandle n;
};

#endif