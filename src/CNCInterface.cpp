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
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/SetMode.h>
#include <sensor_msgs/NavSatStatus.h>
#include <tf/tf.h>

#include <cstdlib>
#include <iostream>

#include <droneoa_ros/CNCInterface.hpp>
#include <droneoa_ros/Utils.hpp>

CNCInterface::CNCInterface() {
    watchHomePosThread();
}

CNCInterface::~CNCInterface() {
    delete thread_watch_Altitude_;
    delete thread_watch_GPSFix_;
    delete thread_watch_state_;
}

void CNCInterface::init(ros::NodeHandle nh, ros::Rate r) {
    n = nh;
    r_ = r;
    thread_watch_state_ = new boost::thread(boost::bind(&CNCInterface::watchStateThread, this));
    thread_watch_GPSFix_ = new boost::thread(boost::bind(&CNCInterface::watchGPSFixThread, this));
    thread_watch_Altitude_ = new boost::thread(boost::bind(&CNCInterface::watchAltitudeThread, this));
    thread_watch_IMU_ = new boost::thread(boost::bind(&CNCInterface::watchIMUThread, this));
    ROS_INFO("[CNC] init");
}

/*****************************************************
 * Mode Control
 */
// Set Flight Mode
// - Input: name of flight mode (please use PDN names)
// - Return: true is operation is successful. false otherwise.
bool CNCInterface::setMode(std::string modeName) {
    ros::ServiceClient cl = n.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
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

/*****************************************************
 * Safety
 */
// Arm the vehicle (include motor)
// - Return: false if arm failed, vehicle not ready. Otherwise true
bool CNCInterface::armVehicle() {
    ros::ServiceClient arming_cl = n.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    mavros_msgs::CommandBool srv;
    srv.request.value = true;

    if (!isReady(getMode())) {
        ROS_ERROR("NOT READY TO ARM UNDER MODE: %s", getMode().c_str());
        return false;
    }

    if (arming_cl.call(srv)) {
        ROS_INFO("ARM send ok %d", srv.response.success);
        return true;
    } else {
        ROS_ERROR("Failed arming or disarming");
        return false;
    }
}

// Check whether the vehicle is ready for to arm under certain mode
// - Input: mode name (Only Allow: GUIDED, STABLIZED, ALT_HOLD)
// - Return: true if ready to arm. Otherwise false
bool CNCInterface::isReady(std::string modeName) {
    if (!isConnected()) {
        ROS_ERROR("VEHICLE NOT CONNECTED !!!");
        return false;
    }
    if (modeName == FLT_MODE_GUIDED) {
        if (!isHomeSet_) {
            ROS_ERROR("No 3D Fix !!!");
            return false;
        }
        if (getMode() == FLT_MODE_GUIDED) {
            ROS_INFO("Ready To Arm GUIDED :)");
            return true;
        }
    } else if (modeName == FLT_MODE_STABILIZE) {
        if (getMode() == FLT_MODE_STABILIZE) {
            ROS_INFO("Ready To Arm STABILIZE :)");
            return true;
        }
    } else if (modeName == FLT_MODE_ALT_HOLD) {
        if (getMode() == FLT_MODE_ALT_HOLD) {
            ROS_INFO("Ready To Arm STABILIZE :)");
            return true;
        }
    }
    ROS_ERROR("NOT READY TO ARM !!!");
    return false;
}

/*****************************************************
 * Guided Flight Control
 */
// Takeoff Command
// - Input: float targetAltitude
// - Return: client send response
bool CNCInterface::takeoff(float targetAltitude) {
    targetAltitude = validAltitudeCMD(targetAltitude);
    ros::ServiceClient takeoff_cl = n.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/takeoff");
    mavros_msgs::CommandTOL srv_takeoff;
    srv_takeoff.request.altitude = targetAltitude;
    srv_takeoff.request.latitude = 0;
    srv_takeoff.request.longitude = 0;
    srv_takeoff.request.min_pitch = 0;
    srv_takeoff.request.yaw = 0;
    if (takeoff_cl.call(srv_takeoff)) {
        ROS_INFO("srv_takeoff send ok %d", srv_takeoff.response.success);
        targetAltitude_ = targetAltitude;
        return true;
    } else {
        ROS_ERROR("Failed Takeoff");
        return false;
    }
}

// Landing Command
// - Input: float minAboutAltitude
// - Return: client send response
bool CNCInterface::land(int minAboutAltitude) {
    ros::ServiceClient land_cl = n.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/land");
    mavros_msgs::CommandTOL srv_land;
    srv_land.request.altitude = minAboutAltitude;
    srv_land.request.latitude = 0;
    srv_land.request.longitude = 0;
    srv_land.request.min_pitch = 0;
    srv_land.request.yaw = 0;
    if (land_cl.call(srv_land)) {
        ROS_INFO("srv_land send ok %d", srv_land.response.success);
        return true;
    } else {
        ROS_ERROR("Failed Land");
        return false;
    }
}

// Set Yaw Command
// - Input: float targetYaw, bool isRelative (default false)
// - Return: client send response
bool CNCInterface::setYaw(float targetYaw, bool isRelative) {
    ros::ServiceClient cmdLong_cl = n.serviceClient<mavros_msgs::CommandLong>("/mavros/cmd/command");
    mavros_msgs::CommandLong srv_cmdLong;
    srv_cmdLong.request.command = mavros_msgs::CommandCode::CONDITION_YAW;
    srv_cmdLong.request.confirmation = 0;
    srv_cmdLong.request.param1 = targetYaw;  // Heading
    srv_cmdLong.request.param2 = 0;  // Speed in degree/s
    srv_cmdLong.request.param3 = 1;  // Direction -1 ccw, 1 cw
    srv_cmdLong.request.param4 = isRelative;
    if (cmdLong_cl.call(srv_cmdLong)) {
        ROS_INFO("setYaw send ok %d", srv_cmdLong.response.success);
        return true;
    } else {
        ROS_ERROR("Failed set Yaw");
        return false;
    }
}

// Set Maximum Speed Command
// - Input:
// --- float speedType: (0=Airspeed, 1=Ground Speed, 2=Climb Speed, 3=Descent Speed)
// --- float speed: (-1 indicates no change m/s)
// --- float isRelative: (0: absolute, 1: relative)
// - Return: client send response
bool CNCInterface::setMaxSpeed(float speedType, float speed, float isRelative) {
    speed = validSpeedCMD(speed);
    mavros_msgs::CommandLong srv;
    srv.request.command = mavros_msgs::CommandCode::DO_CHANGE_SPEED;
    srv.request.param1 = speedType;
    srv.request.param2 = speed;
    srv.request.param3 = -1;
    srv.request.param4 = isRelative;
    srv.request.param5 = 0;
    srv.request.param6 = 0;
    srv.request.param7 = 0;

    return generalLongCommand(srv);
}

/*****************************************************
 * Navigation
 */
// Replace Home Position [Global] [USE WITH CAUTION]
// - Input: 3D Global Coordinate
// - Return: client send response
bool CNCInterface::setHome(float targetLatitude, float targetLongitude, float targetAltitude) {
    mavros_msgs::CommandLong srv;
    srv.request.command = mavros_msgs::CommandCode::DO_SET_HOME;
    srv.request.param1 = 0;
    srv.request.param5 = targetLatitude;
    srv.request.param6 = targetLongitude;
    srv.request.param7 = targetAltitude;

    return generalLongCommand(srv);
}

/*****************************************************
 * Mission
 */
// Push New Waypoint List
// - Input: 3D Global Coordinate
// --- uint8_t isCurrent (2 = isCurrent Guided)
// --- uint16_t command (default NAV_WAYPOINT)
// - Return: client send response
bool CNCInterface::pushWaypoints(float x_lat, float y_long, float z_alt, uint8_t isCurrent, uint16_t command) {
    z_alt = validSpeedCMD(z_alt);
    ros::ServiceClient pushWP_cl = n.serviceClient<mavros_msgs::WaypointPush>("mavros/mission/push");
    mavros_msgs::WaypointPush wp_push_srv;  // List of Waypoints
    mavros_msgs::Waypoint wp;
    wp.frame          = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
    wp.command        = command;
    wp.is_current     = isCurrent;
    wp.autocontinue   = true;
    wp.x_lat          = x_lat;
    wp.y_long         = y_long;
    wp.z_alt          = z_alt;
    wp_push_srv.request.waypoints.push_back(wp);
    // Send WPs to Vehicle
    if (pushWP_cl.call(wp_push_srv)) {
        ROS_INFO("Send waypoints ok: %d", wp_push_srv.response.success);
        targetAltitude_ = z_alt;
        return true;
    } else {
        ROS_ERROR("Send waypoints FAILED.");
        return false;
    }
}

// Clear Waypoint List
// - Return: client send response
bool CNCInterface::clearWaypoint() {
    ros::ServiceClient clearWP_cl = n.serviceClient<mavros_msgs::WaypointClear>("mavros/mission/clear");
    mavros_msgs::WaypointClear wp_clear_srv;
    if (clearWP_cl.call(wp_clear_srv)) {
        ROS_INFO("Clear waypoints ok: %d", wp_clear_srv.response.success);
        return true;
    } else {
        ROS_ERROR("Clear waypoints FAILED.");
        return false;
    }
}

/*****************************************************
 * Callback
 */
/* State */
void CNCInterface::state_callback(const mavros_msgs::State::ConstPtr& msg) {
    current_state_ = *msg;
}
/* GPS Fix */
void CNCInterface::gpsFix_callback(const sensor_msgs::NavSatFixConstPtr& msg) {
    current_gps_fix_ = *msg;
}
/* Home Position */
void CNCInterface::homePos_callback(const mavros_msgs::HomePositionConstPtr& msg) {
    isHomeSet_ = true;
    current_home_pos_ = *msg;
}
/* Altitude */
void CNCInterface::altitude_callback(const std_msgs::Float64ConstPtr& msg) {
    current_relative_altitude_ = *msg;
}
/* Battery */
void CNCInterface::battery_callback(const sensor_msgs::BatteryStateConstPtr& msg) {
    current_battery_ = *msg;
}
/* IMU */
void CNCInterface::IMU_callback(const sensor_msgs::ImuConstPtr& msg) {
    current_IMU_Data_ = *msg;
}
void CNCInterface::Mag_callback(const sensor_msgs::MagneticFieldConstPtr& msg) {
    current_mag_ = *msg;
}

void CNCInterface::HUD_callback(const mavros_msgs::VFR_HUDConstPtr& msg) {
    current_hud_data_ = *msg;
}

/*****************************************************
 * Threads
 */
/* State */
void CNCInterface::watchStateThread() {
    auto state_sub =
        n.subscribe<mavros_msgs::State>("mavros/state", 1, boost::bind(&CNCInterface::state_callback, this, _1));
    auto battery_state_sub =
        n.subscribe<sensor_msgs::BatteryState>("mavros/battery", 1,
        boost::bind(&CNCInterface::battery_callback, this, _1));

    while (ros::ok()) {
        ros::spinOnce();
        r_.sleep();
    }
}
/* GPS Fix */
void CNCInterface::watchGPSFixThread() {
    auto gpsFix_sub =
        n.subscribe<sensor_msgs::NavSatFix>("mavros/global_position/global", 1,
            boost::bind(&CNCInterface::gpsFix_callback, this, _1));

    while (ros::ok()) {
        ros::spinOnce();
        r_.sleep();
    }
}
/* Home Position */
void CNCInterface::watchHomePosThread() {
    auto home_sub =
        n.subscribe<mavros_msgs::HomePosition>("mavros/home_position/home", 1,
            boost::bind(&CNCInterface::homePos_callback, this, _1));

    ROS_WARN("Waiting For 3D Fix ...");
    while (ros::ok() && (ENABLE_SAFETY_GPS && !isHomeSet_)) {
        ros::spinOnce();
        r_.sleep();
    }
}
/* Altitude */
void CNCInterface::watchAltitudeThread() {
    auto node = boost::make_shared<ros::NodeHandle>();  // @TODO: can we remove this ?
    auto relative_pos_sub =
        node->subscribe<std_msgs::Float64>("mavros/global_position/rel_alt", 1,
                boost::bind(&CNCInterface::altitude_callback, this, _1));

    while (ros::ok()) {
        ros::spinOnce();
        r_.sleep();
    }
}
/* IMU */
void CNCInterface::watchIMUThread() {
    auto node = boost::make_shared<ros::NodeHandle>();  // @TODO: can we remove this ?
    auto IMU_data_sub =
        node->subscribe<sensor_msgs::Imu>("mavros/imu/data", 1,
                boost::bind(&CNCInterface::IMU_callback, this, _1));
    auto HUD_data_sub =
        node->subscribe<mavros_msgs::VFR_HUD>("mavros/vfr_hud", 1,
                boost::bind(&CNCInterface::HUD_callback, this, _1));
    // auto Mag_data_sub =
    //     node->subscribe<sensor_msgs::MagneticField>("mavros/imu/mag", 1,
    //             boost::bind(&CNCInterface::Mag_callback, this, _1));

    while (ros::ok()) {
        ros::spinOnce();
        r_.sleep();
    }
}

/*****************************************************
 * Status
 */
/* State */
bool CNCInterface::isConnected() {
    return current_state_.connected;
}

bool CNCInterface::isArmed() {
    return current_state_.armed;
}

bool CNCInterface::isGuided() {
    return current_state_.guided;
}

std::string CNCInterface::getMode() {
    return current_state_.mode;
}

uint8_t CNCInterface::getSysStatus() {
    return current_state_.system_status;
}

/* GPS Fix */
GPSPoint CNCInterface::getCurrentGPSPoint() {
    return GPSPoint(current_gps_fix_.latitude, current_gps_fix_.longitude, current_gps_fix_.altitude);
}

/* Target */
float CNCInterface::getTargetAltitude() {
    return targetAltitude_;
}

GPSPoint CNCInterface::getTargetWaypoint() {
    return recentWaypoint_;
}

/* Altitude */
float CNCInterface::getRelativeAltitude() {
    return current_relative_altitude_.data;
}

/* Battery */
float CNCInterface::getBatteryVoltage() {
    return current_battery_.voltage;
}

/* IMU */
sensor_msgs::Imu CNCInterface::getIMUData() {
    return current_IMU_Data_;
}

geometry_msgs::Vector3 CNCInterface::getIMURawAttitude() {
    tf::Quaternion q;
    tf::quaternionMsgToTF(current_IMU_Data_.orientation, q);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    geometry_msgs::Vector3 result;
    result.x = roll;
    result.y = pitch;
    result.z = yaw;
    return result;
}

mavros_msgs::VFR_HUD CNCInterface::getHUDData() {
    return current_hud_data_;
}

/*****************************************************
 * User Simple Functions
 */
// Goto Global Waypoint
bool CNCInterface::gotoGlobal(float x_lat, float y_long, float z_alt) {
    // @TODO: check Guided mode
    bool rel = false;
    rel = clearWaypoint();
    if (!rel) {
        return rel;
    }
    rel = pushWaypoints(x_lat, y_long, z_alt);
    if (rel) {
        recentWaypoint_ = GPSPoint(x_lat, y_long, z_alt);
    }
    return rel;
}

// Goto Relative Waypoint (North+, East+)
bool CNCInterface::gotoRelative(float x_lat, float y_long, float z_alt = 10, bool isAltDelta) {
    // @TODO: check GPS available
    GPSPoint tmpPoint = getLocationMeter(getCurrentGPSPoint(), x_lat, y_long);
    return gotoGlobal(tmpPoint.latitude_, tmpPoint.longitude_, z_alt);
}

// Goto Target Head
bool CNCInterface::gotoHeading(float heading, float distance, float z_alt) {
    std::pair<float, float> tempRelative = getNorthEastDistanceFromHeading(heading, distance);
    return gotoRelative(tempRelative.first, tempRelative.second, z_alt);
}

/*****************************************************
 * Private CNC Functions
 */
bool CNCInterface::generalLongCommand(mavros_msgs::CommandLong commandMessage) {
    ros::ServiceClient cmdLong_cl = n.serviceClient<mavros_msgs::CommandLong>("/mavros/cmd/command");
    mavros_msgs::CommandLong srv_cmdLong = commandMessage;
    if (cmdLong_cl.call(srv_cmdLong)) {
        ROS_INFO("srv_cmdLong send ok %d", srv_cmdLong.response.success);
        return true;
    } else {
        ROS_ERROR("Failed set Yaw");
        return false;
    }
}
