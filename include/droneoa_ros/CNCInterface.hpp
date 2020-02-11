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

    /**
     * @brief Set Flight Mode
     * @param modeName please use PDN flight mode names
     * @return true is operation is successful. false otherwise.
     */
    bool setMode(std::string modeName);

    // Safety
    /**
     * @brief Arm the vehicle (include motor)
     * @return false if arm failed, vehicle not ready. Otherwise true
     */
    bool armVehicle();
    /**
     * @brief Check whether the vehicle is ready for to arm under certain mode
     * @param modeName (Only Allow: GUIDED, STABLIZED, ALT_HOLD)
     * @return true if ready to arm. Otherwise false
     */
    bool isReady(std::string modeName);

    // Guided Flight Control
    bool takeoff(float targetAltitude);
    bool land(int minAboutAltitude);
    /**
     * @brief Set Yaw Command
     * @param targetYaw in degree
     * @param isRelative (default false)
     * @return client send response
     */
    bool setYaw(float targetYaw, bool isRelative = false);
    /**
     * @brief Set Maximum Speed Command
     * @param speedType (0=Airspeed, 1=Ground Speed, 2=Climb Speed, 3=Descent Speed)
     * @param speed (-1 indicates no change m/s)
     * @param isRelative (0: absolute, 1: relative)
     * @return client send response
     */
    bool setMaxSpeed(float speedType, float speed, float isRelative);

    // Navigation
    /**
     * @brief Replace Home Position [Global] [USE WITH CAUTION]
     * @param targetLatitude 3D Global Coordinate's Latitude
     * @param targetLongitude 3D Global Coordinate's Longitude
     * @param targetAltitude 3D Global Coordinate's Altitude
     * @return client send response
     */
    bool setHome(float targetLatitude, float targetLongitude, float targetAltitude);

    // Mission
    /**
     * @brief Push New Waypoint List
     * @param x_lat
     * @param y_long
     * @param z_alt
     * @param isCurrent (2 = isCurrent Guided)
     * @param command (default NAV_WAYPOINT)
     * @return client send response
     */
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
    /**
     * @brief Goto Global Waypoint, 3D GPS Point
     * @param x_lat
     * @param y_long
     * @param z_alt in meter
     * @return client send response
     */
    bool gotoGlobal(float x_lat, float y_long, float z_alt);
    /**
     * @brief Goto Relative Waypoint (North+, East+)
     * @param x_lat North+ in meter
     * @param y_long East+ in meter
     * @param z_alt default 10 in meter
     * @param isAltDelta not used
     * @return client send response
     */
    bool gotoRelative(float x_lat, float y_long, float z_alt, bool isAltDelta = false);
    /**
     * @brief Goto Target Head
     * @param heading in degree
     * @param distance in meter
     * @param z_alt in meter
     * @return client send response
     */
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

#endif  // INCLUDE_DRONEOA_ROS_CNCINTERFACE_HPP_  // NOLINT
