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
 * Written by Bohan Shi <b34shi@edu.uwaterloo.ca>, April 2020
 */

#ifndef UT_MOCK_MOCK_RATE_HPP  // NOLINT
#define UT_MOCK_MOCK_RATE_HPP  // NOLINT

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <string>
#include <queue>

#include "../../include/droneoa_ros/HWI/interface/CNCInterface.hpp"

namespace CNC {

class CNCMock : public CNCInterface {
 public:
    CNCMock() {}
    virtual ~CNCMock() {}
    // Init
    MOCK_METHOD0(initWatcherThread, void());
    // Commands
    MOCK_METHOD1(setMode, bool(std::string modeName));
    MOCK_METHOD0(armVehicle, bool());
    MOCK_METHOD1(takeoff, bool(float targetAltitude));
    MOCK_METHOD1(land, bool(int32_t minAboutAltitude));
    MOCK_METHOD3(setYaw, bool(float targetYaw, bool isRelative, bool isFromOAC));
    MOCK_METHOD3(setMaxSpeed, bool(float speedType, float speed, float isRelative));
    MOCK_METHOD3(setHome, bool(float targetLatitude, float targetLongitude, float targetAltitude));
    MOCK_METHOD4(gotoGlobal, bool(float x_lat, float y_long, float z_alt, bool isFromOAC));
    MOCK_METHOD5(gotoRelative, bool(float x_lat, float y_long, float z_alt, bool isAltDelta, bool isFromOAC));
    MOCK_METHOD4(gotoHeading, bool(float heading, float distance, float z_alt, bool isFromOAC));
    // Local Mission
    MOCK_METHOD0(clearFCUWaypoint, bool());
    MOCK_METHOD0(clearLocalMissionQueue, void());
    MOCK_METHOD0(getLocalMissionQueue, std::queue<GPSPoint>());
    MOCK_METHOD1(pushLocalMissionQueue, void(GPSPoint wp));
    MOCK_METHOD0(popLocalMissionQueue, GPSPoint());
    MOCK_METHOD0(moveMissionToLocalQueue, void());
    // Status & Checks
    MOCK_METHOD1(isReady, bool(std::string modeName));
    MOCK_METHOD0(isConnected, bool());
    MOCK_METHOD0(isArmed, bool());
    MOCK_METHOD0(getMode, std::string());
    MOCK_METHOD0(getCurrentGPSPoint, GPSPoint());
    MOCK_METHOD0(getHomeGPSPoint, GPSPoint());
    MOCK_METHOD0(isHomeGPSSet, bool());
    MOCK_METHOD0(getRelativeAltitude, float());
    MOCK_METHOD0(getBatteryVoltage, float());
    MOCK_METHOD0(getSysStatus, uint8_t());
    MOCK_METHOD1(checkFModeExist, bool(std::string modeName));
    MOCK_METHOD0(getTargetWaypoint, GPSPoint());
    MOCK_METHOD0(getLocalPosition, geometry_msgs::PoseStamped());
    /* IMU */
    MOCK_METHOD0(getIMUData, sensor_msgs::Imu());
    MOCK_METHOD0(getIMURawAttitude, geometry_msgs::Vector3());
    MOCK_METHOD0(getHUDData, mavros_msgs::VFR_HUD());
};

}  // namespace CNC

#endif  // UT_MOCK_MOCK_RATE_HPP  // NOLINT
