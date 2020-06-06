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

#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/SetMode.h>
#include <sensor_msgs/NavSatStatus.h>
#include <std_msgs/String.h>
#include <tf/tf.h>

#include <cstdlib>
#include <iostream>

#include <droneoa_ros/HWI/base/CNCGeneric.hpp>
#include <droneoa_ros/PDN.hpp>
#include <droneoa_ros/Utils/GeneralUtils.hpp>
#include <droneoa_ros/HWI/Utils/CNCUtils.hpp>

namespace CNC {

CNCGeneric::CNCGeneric(ros::NodeHandle node, ros::Rate rate) : mTargetAltitude(0),
    mNodeHandle(node), mRate(rate) {}

CNCGeneric::~CNCGeneric() {
    if (mpThreadWatchAltitude) {
        mpThreadWatchAltitude->join();
        delete mpThreadWatchAltitude;
    }
    if (mpThreadWatchGPSFix) {
        mpThreadWatchGPSFix->join();
        delete mpThreadWatchGPSFix;
    }
    if (mpThreadWatchState) {
        mpThreadWatchState->join();
        delete mpThreadWatchState;
    }
    if (mpThreadWatchIMU) {
        mpThreadWatchIMU->join();
        delete mpThreadWatchIMU;
    }
    ROS_INFO("Destroy CNCGeneric");
}

/***************************************************************************
 * Init
 */
void CNCGeneric::initWatcherThread() {
    watchHomePosThread();
    mpThreadWatchState = new boost::thread(boost::bind(&CNCGeneric::watchStateThread, this));
    mpThreadWatchGPSFix = new boost::thread(boost::bind(&CNCGeneric::watchGPSFixThread, this));
    mpThreadWatchAltitude = new boost::thread(boost::bind(&CNCGeneric::watchAltitudeThread, this));
    mpThreadWatchIMU = new boost::thread(boost::bind(&CNCGeneric::watchIMUThread, this));
    mGuiInfoPub = mNodeHandle.advertise<std_msgs::String>("droneoa/gui_data", 1000);
    ROS_INFO("[CNC] init threads done");
}

/***************************************************************************
 * Commands
 */
bool CNCGeneric::setMode(std::string modeName) {
    ros::ServiceClient cl = mNodeHandle.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
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

bool CNCGeneric::armVehicle() {
    ros::ServiceClient arming_cl = mNodeHandle.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
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

bool CNCGeneric::takeoff(float targetAltitude) {
    targetAltitude = CNCUtility::validAltitudeCMD(targetAltitude);
    ros::ServiceClient takeoff_cl = mNodeHandle.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/takeoff");
    mavros_msgs::CommandTOL srv_takeoff;
    srv_takeoff.request.altitude = targetAltitude;
    srv_takeoff.request.latitude = 0;
    srv_takeoff.request.longitude = 0;
    srv_takeoff.request.min_pitch = 0;
    srv_takeoff.request.yaw = 0;
    if (takeoff_cl.call(srv_takeoff)) {
        ROS_INFO("srv_takeoff send ok %d", srv_takeoff.response.success);
        mTargetAltitude = targetAltitude;
        return true;
    } else {
        ROS_ERROR("Failed Takeoff");
        return false;
    }
}

bool CNCGeneric::land(int32_t minAboutAltitude) {
    ros::ServiceClient land_cl = mNodeHandle.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/land");
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

bool CNCGeneric::setYaw(float targetYaw, bool isRelative) {
    ros::ServiceClient cmdLong_cl = mNodeHandle.serviceClient<mavros_msgs::CommandLong>("/mavros/cmd/command");
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

bool CNCGeneric::setMaxSpeed(float speedType, float speed, float isRelative) {
    speed = CNCUtility::validSpeedCMD(speed);
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

bool CNCGeneric::setHome(float targetLatitude, float targetLongitude, float targetAltitude) {
    mavros_msgs::CommandLong srv;
    srv.request.command = mavros_msgs::CommandCode::DO_SET_HOME;
    srv.request.param1 = 0;
    srv.request.param5 = targetLatitude;
    srv.request.param6 = targetLongitude;
    srv.request.param7 = targetAltitude;

    return generalLongCommand(srv);
}

bool CNCGeneric::isReady(std::string modeName) {
    if (!isConnected()) {
        ROS_ERROR("VEHICLE NOT CONNECTED !!!");
        return false;
    }
    if (modeName == FLT_MODE_GUIDED) {
        if (!mIsHomeSet) {
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
 * Callback
 */
/* State */
void CNCGeneric::state_callback(const mavros_msgs::State::ConstPtr& msg) {
    mCurrentState = *msg;
}
/* GPS Fix */
void CNCGeneric::gpsFix_callback(const sensor_msgs::NavSatFixConstPtr& msg) {
    mCurrentGpsFix = *msg;
}
/* Home Position */
void CNCGeneric::homePos_callback(const mavros_msgs::HomePositionConstPtr& msg) {
    mIsHomeSet = true;
    mCurrentHomePos = *msg;
}
/* Altitude */
void CNCGeneric::altitude_callback(const std_msgs::Float64ConstPtr& msg) {
    mCurrentRelativeAltitude = *msg;
}
/* Battery */
void CNCGeneric::battery_callback(const sensor_msgs::BatteryStateConstPtr& msg) {
    mCurrentBattery = *msg;
}
/* IMU */
void CNCGeneric::IMU_callback(const sensor_msgs::ImuConstPtr& msg) {
    mCurrentIMUData = *msg;
}
void CNCGeneric::Mag_callback(const sensor_msgs::MagneticFieldConstPtr& msg) {
    mCurrentMag = *msg;
}

void CNCGeneric::HUD_callback(const mavros_msgs::VFR_HUDConstPtr& msg) {
    mCurrentHudData = *msg;
    // Post Infomation Package For GUI
    std_msgs::String guiMsg;
    std::stringstream ss;
    ss << getMode() << " " << getRelativeAltitude() << " " << getBatteryVoltage() << " " << getHUDData().climb << " "
        << getHUDData().heading << " " << getHUDData().groundspeed << " " << getHUDData().throttle;
    guiMsg.data = ss.str();
    mGuiInfoPub.publish(guiMsg);
}

/*****************************************************
 * Threads
 */
void CNCGeneric::watchStateThread() {
    auto state_sub =
        mNodeHandle.subscribe<mavros_msgs::State>("mavros/state", 1,
                boost::bind(&CNCGeneric::state_callback, this, _1));
    auto battery_state_sub =
        mNodeHandle.subscribe<sensor_msgs::BatteryState>("mavros/battery", 1,
                boost::bind(&CNCGeneric::battery_callback, this, _1));

    while (ros::ok()) {
        ros::spinOnce();
        mRate.sleep();
    }
}

void CNCGeneric::watchGPSFixThread() {
    auto gpsFix_sub =
        mNodeHandle.subscribe<sensor_msgs::NavSatFix>("mavros/global_position/global", 1,
            boost::bind(&CNCGeneric::gpsFix_callback, this, _1));

    while (ros::ok()) {
        ros::spinOnce();
        mRate.sleep();
    }
}

void CNCGeneric::watchHomePosThread() {
    auto home_sub =
        mNodeHandle.subscribe<mavros_msgs::HomePosition>("mavros/home_position/home", 1,
            boost::bind(&CNCGeneric::homePos_callback, this, _1));

    ROS_WARN("Waiting For 3D Fix ...");
    while (ros::ok() && (ENABLE_SAFETY_GPS && !mIsHomeSet)) {
        ros::spinOnce();
        mRate.sleep();
    }
}

void CNCGeneric::watchAltitudeThread() {
    auto node = boost::make_shared<ros::NodeHandle>();
    auto relative_pos_sub =
        mNodeHandle.subscribe<std_msgs::Float64>("mavros/global_position/rel_alt", 1,
                boost::bind(&CNCGeneric::altitude_callback, this, _1));

    while (ros::ok()) {
        ros::spinOnce();
        mRate.sleep();
    }
}
/* IMU */
void CNCGeneric::watchIMUThread() {
    auto IMU_data_sub =
        mNodeHandle.subscribe<sensor_msgs::Imu>("mavros/imu/data", 1,
                boost::bind(&CNCGeneric::IMU_callback, this, _1));
    auto HUD_data_sub =
        mNodeHandle.subscribe<mavros_msgs::VFR_HUD>("mavros/vfr_hud", 1,
                boost::bind(&CNCGeneric::HUD_callback, this, _1));
    // auto Mag_data_sub =
    //     mNodeHandle.subscribe<sensor_msgs::MagneticField>("mavros/imu/mag", 1,
    //             boost::bind(&CNCGeneric::Mag_callback, this, _1));

    while (ros::ok()) {
        ros::spinOnce();
        mRate.sleep();
    }
}

/*****************************************************
 * Status
 */
/* State */
bool CNCGeneric::isConnected() {
    return mCurrentState.connected;
}

bool CNCGeneric::isArmed() {
    return mCurrentState.armed;
}

std::string CNCGeneric::getMode() {
    return mCurrentState.mode;
}

uint8_t CNCGeneric::getSysStatus() {
    return mCurrentState.system_status;
}

/* GPS Fix */
GPSPoint CNCGeneric::getCurrentGPSPoint() {
    return GPSPoint(mCurrentGpsFix.latitude, mCurrentGpsFix.longitude, mCurrentGpsFix.altitude);
}

/* Altitude */
float CNCGeneric::getRelativeAltitude() {
    return mCurrentRelativeAltitude.data;
}

/* Battery */
float CNCGeneric::getBatteryVoltage() {
    return mCurrentBattery.voltage;
}

/* IMU */
sensor_msgs::Imu CNCGeneric::getIMUData() {
    return mCurrentIMUData;
}

geometry_msgs::Vector3 CNCGeneric::getIMURawAttitude() {
    tf::Quaternion q;
    tf::quaternionMsgToTF(mCurrentIMUData.orientation, q);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    geometry_msgs::Vector3 result;
    result.x = roll;
    result.y = pitch;
    result.z = yaw;
    return result;
}

mavros_msgs::VFR_HUD CNCGeneric::getHUDData() {
    return mCurrentHudData;
}

/*****************************************************
 * Private CNC Functions
 */
bool CNCGeneric::generalLongCommand(mavros_msgs::CommandLong commandMessage) {
    ros::ServiceClient cmdLong_cl = mNodeHandle.serviceClient<mavros_msgs::CommandLong>("/mavros/cmd/command");
    mavros_msgs::CommandLong srv_cmdLong = commandMessage;
    if (cmdLong_cl.call(srv_cmdLong)) {
        ROS_INFO("srv_cmdLong send ok %d", srv_cmdLong.response.success);
        return true;
    } else {
        ROS_ERROR("Failed set Yaw");
        return false;
    }
}

}  // namespace CNC
