/* Copyright (C) 2020 DroneOA Group - All Rights Reserved
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

#ifndef HWI_INTERFACE_CNCINTERFACE_HPP_  // NOLINT
#define HWI_INTERFACE_CNCINTERFACE_HPP_  // NOLINT

#include <sensor_msgs/Imu.h>
#include <mavros_msgs/VFR_HUD.h>
#include <geometry_msgs/PoseStamped.h>

#include <string>

#include <droneoa_ros/HWI/Utils/GPSPoint.hpp>
#include <droneoa_ros/GUI/GUISubject.hpp>

namespace CNC {

class CNCInterface : public GUI::GUISubject {
 public:
    virtual ~CNCInterface() {}

    // Init
    virtual void initWatcherThread() = 0;

    // Commands
    virtual bool setMode(std::string modeName) = 0;
    virtual bool armVehicle() = 0;
    virtual bool takeoff(float targetAltitude) = 0;
    virtual bool land(int32_t minAboutAltitude) = 0;
    virtual bool setYaw(float targetYaw, bool isRelative = false) = 0;
    virtual bool setMaxSpeed(float speedType, float speed, float isRelative) = 0;
    virtual bool setHome(float targetLatitude, float targetLongitude, float targetAltitude) = 0;
    virtual bool gotoGlobal(float x_lat, float y_long, float z_alt) = 0;
    virtual bool gotoRelative(float x_lat, float y_long, float z_alt, bool isAltDelta = false) = 0;
    virtual bool gotoHeading(float heading, float distance, float z_alt) = 0;

    // Status & Checks
    virtual bool isReady(std::string modeName) = 0;
    virtual bool isConnected() = 0;
    virtual bool isArmed() = 0;
    virtual std::string getMode() = 0;
    virtual GPSPoint getCurrentGPSPoint() = 0;
    virtual float getRelativeAltitude() = 0;
    virtual float getBatteryVoltage() = 0;
    virtual uint8_t getSysStatus() = 0;
    virtual bool checkFModeExist(std::string modeName) = 0;
    virtual GPSPoint getTargetWaypoint() = 0;
    virtual geometry_msgs::PoseStamped getLocalPosition() = 0;
    /* IMU */
    virtual sensor_msgs::Imu getIMUData() = 0;
    virtual geometry_msgs::Vector3 getIMURawAttitude() = 0;
    virtual mavros_msgs::VFR_HUD getHUDData() = 0;
};

}  // namespace CNC

#endif  // HWI_INTERFACE_CNCINTERFACE_HPP_  // NOLINT
