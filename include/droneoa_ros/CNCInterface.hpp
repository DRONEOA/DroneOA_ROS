/* Copyright (C) DroneOA Group - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 * Written by Bohan Shi <b34shi@edu.uwaterloo.ca>, August 2019
 */

#ifndef INCLUDE_DRONEOA_ROS_CNCINTERFACE_HPP_  // NOLINT
#define INCLUDE_DRONEOA_ROS_CNCINTERFACE_HPP_  // NOLINT

#include <ros/ros.h>
#include <mavros_msgs/Waypoint.h>
#include <mavros_msgs/WaypointPush.h>
#include <mavros_msgs/WaypointClear.h>
#include <mavros_msgs/CommandCode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/Altitude.h>
#include <sensor_msgs/NavSatFix.h>

#include <string>
#include <vector>
#include <boost/bind.hpp>
#include <boost/thread/thread.hpp>

#include <droneoa_ros/GPSPoint.hpp>

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
    bool setYaw(float targetYaw, bool isRelative = false);

    // Navigation
    bool setHome(float targetLatitude, float targetLongitude, float targetAltitude);

    // Mission
    bool pushWaypoints(float x_lat, float y_long, float z_alt, uint8_t isCurrent = 2,
        uint16_t command = mavros_msgs::CommandCode::NAV_WAYPOINT);
    bool clearWaypoint();

    // Callback
    void state_callback(const mavros_msgs::State::ConstPtr& msg);
    void gpsFix_callback(const sensor_msgs::NavSatFixConstPtr& msg);

    // Status
    /* State */
    bool isConnected();
    bool isArmed();
    bool isGuided();
    std::string getMode();
    uint8_t getSysStatus();
    /* GPS Fix */
    GPSPoint getCurrentGPSPoint();
    /* Target */
    float getTargetAltitude();
    GPSPoint getTargetWaypoint();

    // User Simple Function
    bool gotoGlobal(float x_lat, float y_long, float z_alt);
    bool gotoRelative(float x_lat, float y_long, float z_alt, bool isAltDelta = false);
    bool gotoHeading(float heading, float distance, float z_alt);

 private:
    ros::NodeHandle n;
    ros::Rate r_ = ros::Rate(10.0);

    float targetAltitude_ = 0;
    GPSPoint recentWaypoint_;

    mavros_msgs::State current_state_;
    sensor_msgs::NavSatFix current_gps_fix_;
    boost::thread* thread_watch_state_ = nullptr;
    boost::thread* thread_watch_GPSFix_ = nullptr;
    void watchStateThread();
    void watchGPSFixThread();
};

#endif  // NOLINT
