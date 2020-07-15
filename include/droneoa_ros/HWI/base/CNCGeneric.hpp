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

#ifndef HWI_BASE_CNCGENERIC_HPP_  // NOLINT
#define HWI_BASE_CNCGENERIC_HPP_  // NOLINT

#include <mavros_msgs/State.h>
#include <mavros_msgs/Altitude.h>
#include <mavros_msgs/HomePosition.h>
#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/CommandCode.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/MagneticField.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/PoseStamped.h>

#include <string>
#include <queue>

#include <boost/bind.hpp>
#include <boost/thread/thread.hpp>

#include <droneoa_ros/PDN.hpp>
#include <droneoa_ros/HWI/interface/CNCInterface.hpp>
#include <droneoa_ros/GUI/GUISubject.hpp>

namespace CNC {

class CNCGeneric : public CNCInterface {
 public:
    CNCGeneric(ros::NodeHandle node, ros::Rate rate);
    virtual ~CNCGeneric();

    /***************************************************************************
     * Init
     */
    void initWatcherThread() override;

    /***************************************************************************
     * Commands
     */
    /**
     * @brief Set Flight Mode
     * @param modeName please use PDN flight mode names
     * @return true is operation is successful. false otherwise.
     */
    bool setMode(std::string modeName) override;
    /**
     * @brief Arm the vehicle (include motor)
     * @return false if arm failed, vehicle not ready. Otherwise true
     */
    bool armVehicle() override;
    bool takeoff(float targetAltitude) override;
    bool land(int32_t minAboutAltitude) override;
    /**
     * @brief Set Yaw Command
     * @param targetYaw in degree
     * @param isRelative (default false)
     * @return client send response
     */
    bool setYaw(float targetYaw, bool isRelative = false, bool isFromOAC = false) override;
    /**
     * @brief Set Maximum Speed Command
     * @param speedType (0=Airspeed, 1=Ground Speed, 2=Climb Speed, 3=Descent Speed)
     * @param speed (-1 indicates no change m/s)
     * @param isRelative (0: absolute, 1: relative)
     * @return client send response
     */
    bool setMaxSpeed(float speedType, float speed, float isRelative) override;
    /**
     * @brief Replace Home Position [Global] [USE WITH CAUTION]
     * @param targetLatitude 3D Global Coordinate's Latitude
     * @param targetLongitude 3D Global Coordinate's Longitude
     * @param targetAltitude 3D Global Coordinate's Altitude
     * @return client send response
     */
    bool setHome(float targetLatitude, float targetLongitude, float targetAltitude) override;
    /**
     * @brief Push guided waypoint based on local ENU coordination system
     * @param location LocalPoint target local postion
     * @param isFromOAC Does the caller has OAC privilege
     * @return client send response 
     */
    bool pushLocalENUWaypoint(const LocalPoint location, bool isFromOAC = false) override;

    /***************************************************************************
     * Local Mission Queue
     */

    void clearLocalMissionQueue() override;
    std::queue<Position3D> getLocalMissionQueue() override;
    void pushLocalMissionQueue(Position3D wp) override;
    Position3D popLocalMissionQueue() override;

    /***************************************************************************
     * Status & Checks
     */
    bool isConnected() override;
    bool isArmed() override;
    std::string getMode() override;
    GPSPoint getCurrentGPSPoint() override;
    GPSPoint getHomeGPSPoint() override;
    bool isHomeGPSSet() override;
    float getRelativeAltitude() override;
    float getBatteryVoltage() override;
    uint8_t getSysStatus() override;
    /* IMU */
    sensor_msgs::Imu getIMUData() override;
    geometry_msgs::Vector3 getIMURawAttitude() override;
    mavros_msgs::VFR_HUD getHUDData() override;
    /* Local Position */
    LocalPoint getLocalPosition() override;
    LocalPoint getCurrentLocalENUTarget() override;

    /***************************************************************************
     * Callback
     */
    void state_callback(const mavros_msgs::State::ConstPtr& msg);
    void gpsFix_callback(const sensor_msgs::NavSatFixConstPtr& msg);
    void homePos_callback(const mavros_msgs::HomePositionConstPtr& msg);
    void altitude_callback(const std_msgs::Float64ConstPtr& msg);
    void battery_callback(const sensor_msgs::BatteryStateConstPtr& msg);
    void IMU_callback(const sensor_msgs::ImuConstPtr& msg);
    void Mag_callback(const sensor_msgs::MagneticFieldConstPtr& msg);
    void HUD_callback(const mavros_msgs::VFR_HUDConstPtr& msg);
    void LocalPosition_callback(const geometry_msgs::PoseStampedConstPtr& msg);

 protected:
    ros::NodeHandle mNodeHandle;
    ros::Rate mRate = ros::Rate(GLOBAL_ROS_RATE);

    /***************************************************************************
     * Local Data
     */
    float mTargetAltitude;
    GPSPoint mRecentWaypoint;
    GPSPoint mHomeGPS;
    bool mIsHomeGPSSet = false;

    bool mIsHomeSet = false;  // Note: this value will NOT be updated after becomeing true
    mavros_msgs::State mCurrentState;
    mavros_msgs::HomePosition mCurrentHomePos;
    mavros_msgs::VFR_HUD mCurrentHudData;
    sensor_msgs::NavSatFix mCurrentGpsFix;
    sensor_msgs::BatteryState mCurrentBattery;
    sensor_msgs::MagneticField mCurrentMag;
    std_msgs::Float64 mCurrentRelativeAltitude;
    sensor_msgs::Imu mCurrentIMUData;
    geometry_msgs::PoseStamped mLocalPosition;
    std::queue<Position3D> mLocalMissionQueue;
    LocalPoint mCurrentNEULocalTarget;

    /***************************************************************************
     * Threads
     */
    boost::thread* mpThreadWatchState = nullptr;
    boost::thread* mpThreadWatchGPSFix = nullptr;
    boost::thread* mpThreadWatchAltitude = nullptr;
    boost::thread* mpThreadWatchIMU = nullptr;
    boost::thread* mpThreadWatchLocalPosition = nullptr;
    void watchStateThread();
    void watchGPSFixThread();
    void watchHomePosThread();
    void watchAltitudeThread();
    void watchIMUThread();
    void watchLocalPositionThread();

    /***************************************************************************
     * Publisher
     */
    ros::Publisher mGuiInfoPub;
    ros::Publisher mSetpointLocalPub;

    // Private CNC
    bool generalLongCommand(mavros_msgs::CommandLong commandMessage);
};

}  // namespace CNC

#endif  // HWI_BASE_CNCGENERIC_HPP_  // NOLINT
