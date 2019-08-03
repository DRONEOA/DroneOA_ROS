#ifndef CNC_INTERFACE_
#define CNC_INTERFACE_

#include <string>
#include <vector>
#include <boost/bind.hpp>
#include <boost/thread/thread.hpp>

#include <ros/ros.h>
#include <mavros_msgs/Waypoint.h>
#include <mavros_msgs/WaypointPush.h>
#include <mavros_msgs/WaypointClear.h>
#include <mavros_msgs/CommandCode.h>
#include <mavros_msgs/State.h>

class CNCInterface {
public:
    CNCInterface();
    virtual ~CNCInterface();
    void init(ros::NodeHandle nh, ros::Rate r);

    // Mode Control
    bool setMode(std::string modeName);

    // Safety
    bool armVehicle();

    // Guided Flight Control
    bool takeoff(int targetAltitude);
    bool land(int fromAltitude);

    // Navigation
    bool setHome(float targetLatitude, float targetLongitude, float targetAltitude);

    // Mission
    bool pushWaypoints(float x_lat, float y_long, float z_alt, uint8_t isCurrent = 2, uint16_t command = mavros_msgs::CommandCode::NAV_WAYPOINT);
    bool clearWaypoint();

    // Callback
    void state_callback(const mavros_msgs::State::ConstPtr& msg);

    // Status
    bool isConnected();
    bool isArmed();
    std::string getMode();

private:
    ros::NodeHandle n;
    ros::Rate r_ = ros::Rate(10.0);
    mavros_msgs::State current_state;
    std::vector<mavros_msgs::Waypoint> waypointVec;

    boost::thread* thread_watch_state_ = nullptr;
    void watchStateThread();
};

#endif