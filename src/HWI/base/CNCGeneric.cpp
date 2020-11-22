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
#include <tf/tf.h>
#include <tf2/LinearMath/Quaternion.h>

#include <cstdlib>
#include <iostream>

#include <droneoa_ros/HWI/base/CNCGeneric.hpp>
#include <droneoa_ros/PDN.hpp>
#include <droneoa_ros/Utils/GeneralUtils.hpp>
#include <droneoa_ros/HWI/Utils/CNCUtils.hpp>
#include <droneoa_ros/OAC/OAC.hpp>

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
    if (mpThreadWatchLocalPosition) {
        mpThreadWatchLocalPosition->join();
        delete mpThreadWatchLocalPosition;
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
    mpThreadWatchLocalPosition = new boost::thread(boost::bind(&CNCGeneric::watchLocalPositionThread, this));
    mSetpointLocalPub = mNodeHandle.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10);
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

    if (isArmed()) {
        ROS_WARN("Vehicle Already Armed");
        return true;
    }

    if (!isReady(getMode())) {
        ROS_ERROR("NOT READY TO ARM UNDER MODE: %s", getMode().c_str());
        return false;
    }

    if (arming_cl.call(srv)) {
        ROS_INFO("ARM send ok %d", srv.response.success);
        mHomeGPS = getCurrentGPSPoint();
        mIsHomeGPSSet = true;
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

bool CNCGeneric::setYaw(float targetYaw, bool isRelative, bool isFromOAC) {
    if (!isFromOAC && msDP.getDataAsInt(DP::DP_ACTIVE_OAC_LEVEL) > 1) {
        ROS_WARN("SetYaw is ignored from non-OAC requester to ensure snesor facing forward !");
        return true;
    }
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

bool CNCGeneric::pushLocalENUWaypoint(const LocalPoint location, bool isFromOAC) {
    if (!isFromOAC && msDP.getDataAsInt(DP::DP_ACTIVE_OAC_LEVEL) > 1) {
        if (OAC_USE_SETPOINT_ENU) {
            pushLocalMissionQueue(location);
            return true;
        }
        ROS_WARN("Command Ignored Due to: Miss matched coordination system");
        return false;
    }
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = location.mX;
    pose.pose.position.y = location.mY;
    pose.pose.position.z = CNCUtility::validAltitudeCMD(location.mZ);
    tf::Quaternion rotation;
    // Calculate new heading (NED)
    LocalPoint current = getLocalPosition();
    float heading = std::atan2(current.mX - location.mX, current.mY - location.mY) + M_PI * (3/2);
    heading = (2*M_PI) - heading;
    // There is 90 degress rotation introduceed in the mavros
    pose.pose.orientation = tf::createQuaternionMsgFromYaw(heading + M_PI/2);
    mSetpointLocalPub.publish(pose);
    mCurrentNEULocalTarget = location;
    msDP.setData(DP::DP_CURR_SETPOINT_ENU_TARGET, mCurrentNEULocalTarget);
    ROS_WARN("Setpoint: %s", location.AsString().c_str());
    return true;
}

/*****************************************************
 * Callback (Update local data and DP data)
 */
/* State */
void CNCGeneric::state_callback(const mavros_msgs::State::ConstPtr& msg) {
    mCurrentState = *msg;
    msDP.setData(DP::DP_IS_ARMED, isArmed());
    msDP.setData(DP::DP_FLIGHT_MOD, getMode());
    msDP.setData(DP::DP_IS_CONNECTED, isConnected());
    msDP.setData(DP::DP_SYS_STATUS, getSysStatus());
    msDP.setData(DP::DP_IS_GUIDED, static_cast<bool>(mCurrentState.guided));
    GUI::GUISubject::notifyGUIPopups();
}
/* GPS Fix */
void CNCGeneric::gpsFix_callback(const sensor_msgs::NavSatFixConstPtr& msg) {
    mCurrentGpsFix = *msg;
    msDP.setData(DP::DP_GPS_LOC, getCurrentGPSPoint());
    GUI::GUISubject::notifyGUIPopups();
}

void CNCGeneric::LocalPosition_callback(const geometry_msgs::PoseStampedConstPtr& msg) {
    mLocalPosition = *msg;
    msDP.setData(DP::DP_LOCAL_LOC, getLocalPosition());
    GUI::GUISubject::notifyGUIPopups();
}

/* Home Position */
void CNCGeneric::homePos_callback(const mavros_msgs::HomePositionConstPtr& msg) {
    mIsHomeSet = true;
    mCurrentHomePos = *msg;
    msDP.setData(DP::DP_GPS_HOME, getHomeGPSPoint());
    GUI::GUISubject::notifyGUIPopups();
}
/* Altitude */
void CNCGeneric::altitude_callback(const std_msgs::Float64ConstPtr& msg) {
    mCurrentRelativeAltitude = *msg;
    msDP.setData(DP::DP_RELATIVE_ALTITUDE, getRelativeAltitude());
    GUI::GUISubject::notifyGUIPopups();
}
/* Battery */
void CNCGeneric::battery_callback(const sensor_msgs::BatteryStateConstPtr& msg) {
    mCurrentBattery = *msg;
    msDP.setData(DP::DP_BATTERY_VOLTAGE, getBatteryVoltage());
    GUI::GUISubject::notifyGUIPopups();
}
/* IMU */
void CNCGeneric::IMU_callback(const sensor_msgs::ImuConstPtr& msg) {
    mCurrentIMUData = *msg;
    msDP.setData(DP::DP_ORIENTATION_QUAT, mCurrentIMUData.orientation);
    msDP.setData(DP::DP_ORIENTATION_RPY, CNC::CNCUtility::quaternionToRPY(mCurrentIMUData.orientation));
    GUI::GUISubject::notifyGUIPopups();
}
void CNCGeneric::Mag_callback(const sensor_msgs::MagneticFieldConstPtr& msg) {
    mCurrentMag = *msg;
    GUI::GUISubject::notifyGUIPopups();
}

void CNCGeneric::HUD_callback(const mavros_msgs::VFR_HUDConstPtr& msg) {
    mCurrentHudData = *msg;
    msDP.setData(DP::DP_HEADING, mCurrentHudData.heading);
    msDP.setData(DP::DP_AIR_SPEED, mCurrentHudData.airspeed);
    msDP.setData(DP::DP_GROUND_SPEED, mCurrentHudData.groundspeed);
    msDP.setData(DP::DP_HUD_ALTITUDE, mCurrentHudData.altitude);
    msDP.setData(DP::DP_CLIMB_Rate, mCurrentHudData.climb);
    msDP.setData(DP::DP_THROTTLE, mCurrentHudData.throttle);
    GUI::GUISubject::notifyGUIPopups();
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
    while (ros::ok()) {
        // Check is home set, set indicate we have a 2D+ GPS fix
        if (mIsHomeSet) {
            break;
        }
        // Prevent uninitialized data pool
        bool safetySetting = true;
        try {
            safetySetting = boost::any_cast<bool>(msDP.getData(DP::CONF_SAFETY_GPS_FIX));
        } catch(boost::bad_any_cast& e) {
            ROS_WARN("GPS Fix Safety Setting Missing, Enabled by Default");
        }
        if (!safetySetting) {
            ROS_WARN("GPS Fix Safety Disabled");
            break;
        }
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

void CNCGeneric::watchLocalPositionThread() {
    auto localPose_sub =
        mNodeHandle.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 1,
            boost::bind(&CNCGeneric::LocalPosition_callback, this, _1));

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

GPSPoint CNCGeneric::getHomeGPSPoint() {
    return mHomeGPS;
}

bool CNCGeneric::isHomeGPSSet() {
    return mIsHomeGPSSet;
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

LocalPoint CNCGeneric::getLocalPosition() {
    return LocalPoint(mLocalPosition.pose.position.x,
            mLocalPosition.pose.position.y,
            mLocalPosition.pose.position.z);
}

LocalPoint CNCGeneric::getCurrentLocalENUTarget() {
    return mCurrentNEULocalTarget;
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

/***************************************************************************
 * Local Mission Queue
 */

void CNCGeneric::clearLocalMissionQueue() {
    mLocalMissionQueue = std::queue<Position3D>();
}

std::queue<Position3D> CNCGeneric::getLocalMissionQueue() {
    return mLocalMissionQueue;
}

void CNCGeneric::pushLocalMissionQueue(Position3D wp) {
    mLocalMissionQueue.push(wp);
}

Position3D CNCGeneric::popLocalMissionQueue() {
    Position3D nextP3D = mLocalMissionQueue.front();
    mLocalMissionQueue.pop();
    return nextP3D;
}

}  // namespace CNC
