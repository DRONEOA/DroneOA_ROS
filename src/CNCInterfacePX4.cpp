/* Copyright (C) DroneOA Group - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 * Written by Bohan Shi <b34shi@edu.uwaterloo.ca>, August 2019
 */

#include <droneoa_ros/CNCInterfacePX4.hpp>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/PositionTarget.h>

CNCInterfacePX4::CNCInterfacePX4() {
    home_.geo.latitude = home_.geo.longitude = home_.geo.altitude = NAN;
}

CNCInterfacePX4::~CNCInterfacePX4() {
    delete thread_watch_alt_;
}

// Init PX4 Command and Control Interface
// - Input: ros::NodeHandle; ros::Rate
void CNCInterfacePX4::init(ros::NodeHandle nh, ros::Rate r) {
    nh_ = nh;
    rate_ = r;
    getHomeGeoPoint();
    thread_watch_alt_ = new boost::thread(boost::bind(&CNCInterfacePX4::watchAltitudeThread, this));
}

/* Mode Control */
bool CNCInterfacePX4::setMode(std::string modeName) {
    ros::ServiceClient cl = nh_.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    mavros_msgs::SetMode srv_setMode;
    srv_setMode.request.base_mode = 0;
    srv_setMode.request.custom_mode = modeName;
    if (cl.call(srv_setMode)) {
        ROS_INFO("setmode send ok %d value:", srv_setMode.response.mode_sent);
        sleep(1);
        return true;
    } else {
        ROS_ERROR("Failed SetMode");
        return false;
    }
}

/* Safety Feature */
// Arm the vehicle (include motor)
// - Return: bool response
bool CNCInterfacePX4::armVehicle() {
    if (!home_set_) {
        ROS_ERROR("ARM REFUSED !!! NO GPS 3D FIX");
        return false;
    }

    auto arming_client = nh_.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    mavros_msgs::CommandBool srv_arm;
    srv_arm.request.value = true;
    if (arming_client.call(srv_arm) && srv_arm.response.success) {
        ROS_INFO("ARM SENT ->");
        return true;
    } else {
        ROS_ERROR("CANNOT SEND ARM COMMAND !!!");
        return false;
    }
}

/* Flight Services */
// Request Takeoff, Require 3D FIX and Armed
// - Input: int targetAltitude
// - Return: bool response
bool CNCInterfacePX4::takeoff(double dz) {
    if (!home_set_) {  // @todo: Also need to check is armed
        ROS_ERROR("ARM REFUSED !!! NO GPS 3D FIX");
        return false;
    }

    auto takeoff_client = nh_.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/takeoff");
    mavros_msgs::CommandTOL srv_takeoff{};

    srv_takeoff.request.altitude = dz;
    srv_takeoff.request.latitude = home_.geo.latitude;
    srv_takeoff.request.longitude = home_.geo.longitude;
    srv_takeoff.request.yaw = 0;

    if (takeoff_client.call(srv_takeoff) && srv_takeoff.response.success) {
        ROS_INFO("TAKEOFF SENT ->");
        return true;
    } else {
        ROS_ERROR("CANNOT SEND TAKEOFF COMMAND !!!");
        return false;
    }
}

// Request land
// - Return: bool response
bool CNCInterfacePX4::land() {
    auto land_client = nh_.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/land");
    mavros_msgs::CommandTOL srv_land{};

    if (land_client.call(srv_land) && srv_land.response.success) {
        ROS_INFO("LAND SENT ->");
        return true;
    } else {
        ROS_ERROR("CANNOT SEND LAND COMMAND !!!");
        return false;
    }
}

/* Public Getter */
// Get the home position set before takeoff
// - Return: mavros_msgs::HomePosition
mavros_msgs::HomePosition CNCInterfacePX4::getHomePosition() {
    return home_;
}

// Get relative altitude global_position/rel_alt
// - Return: std_msgs::Float64
std_msgs::Float64 CNCInterfacePX4::getRelativeAltitude() {
    return relative_altitude_;
}

/* CB Callbacks */
void CNCInterfacePX4::setHomeGeoPointCB(const mavros_msgs::HomePositionConstPtr& home) {
    home_ = *home;
    home_set_ = true;
    ROS_INFO("Home Set (WGS84 datum): %lf, %lf, %lf", home_.geo.latitude, home_.geo.longitude, home_.geo.altitude);
}

void CNCInterfacePX4::setRelativeAltitudeCB(const std_msgs::Float64ConstPtr& msg) {
    relative_altitude_ = *msg;
    ROS_INFO("Altitude: %f", relative_altitude_.data);
}

/* Threads */
void CNCInterfacePX4::watchAltitudeThread() {
    std_msgs::Float64 relative_altitude;
    auto node = boost::make_shared<ros::NodeHandle>();
    auto relative_pos_sub =
        node->subscribe<std_msgs::Float64>("mavros/global_position/rel_alt", 1,
                boost::bind(&CNCInterfacePX4::setRelativeAltitudeCB, this, _1));

    while (ros::ok()) {
        ros::spinOnce();
        rate_.sleep();
    }
}

/* Getter */
// Request Home Position
// [Only Call On Init]
void CNCInterfacePX4::getHomeGeoPoint() {
    // FCU Home position: See http://docs.ros.org/api/sensor_msgs/html/msg/NavSatFix.html
    auto home_sub = nh_.subscribe<mavros_msgs::HomePosition>("mavros/home_position/home", 1,
            boost::bind(&CNCInterfacePX4::setHomeGeoPointCB, this, _1));

    ROS_WARN("Waiting for 3D FIX...");
    while (ros::ok() && !home_set_) {
        ros::spinOnce();
        rate_.sleep();
    }
}
