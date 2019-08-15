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

#ifndef INCLUDE_DRONEOA_ROS_CNCINTERFACE_HPP_  // NOLINT
#define INCLUDE_DRONEOA_ROS_CNCINTERFACE_HPP_  // NOLINT

#include <ros/ros.h>
#include <mavros_msgs/Waypoint.h>
#include <mavros_msgs/WaypointPush.h>
#include <mavros_msgs/WaypointClear.h>
#include <mavros_msgs/CommandCode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/Altitude.h>
#include <mavros_msgs/HomePosition.h>
#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/VFR_HUD.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <std_msgs/Float64.h>

#include <string>
#include <vector>
#include <boost/bind.hpp>
#include <boost/thread/thread.hpp>

#include <droneoa_ros/GPSPoint.hpp>
#include <droneoa_ros/PDN.hpp>

class CNCInterface {
 public:
    CNCInterface();
    virtual ~CNCInterface();
    void init(ros::NodeHandle nh, ros::Rate r);

    // Mode Control
    bool setMode(std::string modeName);

    // Safety
    bool armVehicle();
    bool isReady(std::string modeName);

    // Guided Flight Control
    bool takeoff(float targetAltitude);
    bool land(int minAboutAltitude);
    bool setYaw(float targetYaw, bool isRelative = false);
    bool setMaxSpeed(float speedType, float speed, float isRelative);

    // Navigation
    bool setHome(float targetLatitude, float targetLongitude, float targetAltitude);

    // Mission
    bool pushWaypoints(float x_lat, float y_long, float z_alt, uint8_t isCurrent = 2,
        uint16_t command = mavros_msgs::CommandCode::NAV_WAYPOINT);
    bool clearWaypoint();

    // Callback
    void state_callback(const mavros_msgs::State::ConstPtr& msg);
    void gpsFix_callback(const sensor_msgs::NavSatFixConstPtr& msg);
    void homePos_callback(const mavros_msgs::HomePositionConstPtr& msg);
    void altitude_callback(const std_msgs::Float64ConstPtr& msg);
    void battery_callback(const sensor_msgs::BatteryStateConstPtr& msg);
    void IMU_callback(const sensor_msgs::ImuConstPtr& msg);
    void Mag_callback(const sensor_msgs::MagneticFieldConstPtr& msg);
    void HUD_callback(const mavros_msgs::VFR_HUDConstPtr& msg);

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
    /* Altitude */
    float getRelativeAltitude();
    /* Battery */
    float getBatteryVoltage();
    /* IMU */
    sensor_msgs::Imu getIMUData();
    geometry_msgs::Vector3 getIMURawAttitude();
    mavros_msgs::VFR_HUD getHUDData();

    // User Simple Function
    bool gotoGlobal(float x_lat, float y_long, float z_alt);
    bool gotoRelative(float x_lat, float y_long, float z_alt, bool isAltDelta = false);
    bool gotoHeading(float heading, float distance, float z_alt);

 private:
    ros::NodeHandle n;
    ros::Rate r_ = ros::Rate(10.0);

    float targetAltitude_ = 0;
    GPSPoint recentWaypoint_;

    bool isHomeSet_ = false;  // Note: this value will NOT be updated after becomeing true
    mavros_msgs::State current_state_;
    mavros_msgs::HomePosition current_home_pos_;
    mavros_msgs::VFR_HUD current_hud_data_;
    sensor_msgs::NavSatFix current_gps_fix_;
    sensor_msgs::BatteryState current_battery_;
    sensor_msgs::MagneticField current_mag_;
    std_msgs::Float64 current_relative_altitude_;
    sensor_msgs::Imu current_IMU_Data_;

    // Threads
    boost::thread* thread_watch_state_ = nullptr;
    boost::thread* thread_watch_GPSFix_ = nullptr;
    boost::thread* thread_watch_Altitude_ = nullptr;
    boost::thread* thread_watch_IMU_ = nullptr;
    void watchStateThread();
    void watchGPSFixThread();
    void watchHomePosThread();
    void watchAltitudeThread();
    void watchIMUThread();

    // Private CNC
    bool generalLongCommand(mavros_msgs::CommandLong commandMessage);

    // Config
    std::vector<float> RPYOffsets;
};

#endif  // NOLINT
