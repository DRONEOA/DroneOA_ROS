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

#include <droneoa_ros/HWI/CNCArdupilot.hpp>
#include <droneoa_ros/PDN.hpp>
#include <droneoa_ros/Utils/GeneralUtils.hpp>
#include <droneoa_ros/HWI/Utils/CNCUtils.hpp>
#include <droneoa_ros/OAC/OAC.hpp>

namespace CNC {

CNCArdupilot::CNCArdupilot(ros::NodeHandle node, ros::Rate rate) : CNCGeneric(node, rate) {}

CNCArdupilot::~CNCArdupilot() {}

void CNCArdupilot::initWatcherThread() {
    CNCGeneric::initWatcherThread();
    //! @note Seems there is a bug in mavros. Which causing Reach message not send with SITL
    // mpThreadWatchWPReach = new boost::thread(boost::bind(&CNCArdupilot::watchReachThread, this));
    mpThreadWatchWPList = new boost::thread(boost::bind(&CNCArdupilot::watchWPListThread, this));
}

void CNCArdupilot::watchReachThread() {
    auto wpReach_sub =
        mNodeHandle.subscribe<mavros_msgs::WaypointReached>("mavros/mission/reached", 1,
            boost::bind(&CNCArdupilot::WP_reach_callback, this, _1));

    while (ros::ok()) {
        ros::spinOnce();
        mRate.sleep();
    }
}

void CNCArdupilot::watchWPListThread() {
    auto wpReach_sub =
        mNodeHandle.subscribe<mavros_msgs::WaypointList>("mavros/mission/waypoints", 1,
            boost::bind(&CNCArdupilot::WP_list_callback, this, _1));

    while (ros::ok()) {
        ros::spinOnce();
        mRate.sleep();
    }
}

void CNCArdupilot::WP_reach_callback(const mavros_msgs::WaypointReachedConstPtr& msg) {
    for (auto call : mpWpReachCallbackList) {
        if (call) {
            call();
        }
    }
    mpWpReachCallbackList.clear();
}

void CNCArdupilot::WP_list_callback(const mavros_msgs::WaypointListConstPtr& msg) {
    mWaypointList = *msg;
}

/***************************************************************************
 * Mission
 */
bool CNCArdupilot::pushWaypoints(float x_lat, float y_long, float z_alt, uint8_t isCurrent, uint16_t command) {
    z_alt = CNCUtility::validSpeedCMD(z_alt);
    ros::ServiceClient pushWP_cl = mNodeHandle.serviceClient<mavros_msgs::WaypointPush>("mavros/mission/push");
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
        mTargetAltitude = z_alt;
        return true;
    } else {
        ROS_ERROR("Send waypoints FAILED.");
        return false;
    }
}

bool CNCArdupilot::pushWaypoints(std::vector<GPSPoint> wpList, uint8_t isCurrent, uint16_t command) {
    ros::ServiceClient pushWP_cl = mNodeHandle.serviceClient<mavros_msgs::WaypointPush>("mavros/mission/push");
    mavros_msgs::WaypointPush wp_push_srv;  // List of Waypoints
    float z_alt = getRelativeAltitude();
    for (auto tmpWP : wpList) {
        mavros_msgs::Waypoint wp;
        wp.frame          = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
        wp.command        = command;
        wp.is_current     = isCurrent;
        wp.autocontinue   = true;
        wp.x_lat          = tmpWP.latitude_;
        wp.y_long         = tmpWP.longitude_;
        z_alt = CNCUtility::validSpeedCMD(tmpWP.altitude_);
        wp.z_alt          = z_alt;
        wp_push_srv.request.waypoints.push_back(wp);
    }
    // Send WPs to Vehicle
    if (pushWP_cl.call(wp_push_srv)) {
        ROS_INFO("Send waypoints ok: %d", wp_push_srv.response.success);
        mTargetAltitude = z_alt;
        return true;
    } else {
        ROS_ERROR("Send waypoints FAILED.");
        return false;
    }
}

mavros_msgs::WaypointPull CNCArdupilot::pullWaypoints() {
    mavros_msgs::WaypointPull pulledWp;
    ros::ServiceClient pullWP_cl = mNodeHandle.serviceClient<mavros_msgs::WaypointPush>("mavros/mission/pull");
    if (pullWP_cl.call(pulledWp) && pulledWp.response.success) {
        ROS_INFO("%d", pulledWp.response.wp_received);
        ROS_INFO("Waypoint pull success");
    }
}

bool CNCArdupilot::clearFCUWaypoint() {
    ros::ServiceClient clearWP_cl = mNodeHandle.serviceClient<mavros_msgs::WaypointClear>("mavros/mission/clear");
    mavros_msgs::WaypointClear wp_clear_srv;
    if (clearWP_cl.call(wp_clear_srv)) {
        ROS_INFO("Clear waypoints ok: %d", wp_clear_srv.response.success);
        return true;
    } else {
        ROS_ERROR("Clear waypoints FAILED.");
        return false;
    }
}

void CNCArdupilot::registForReachEvent(std::function<void()> callback) {
    if (callback) {
        mpWpReachCallbackList.push_back(callback);
    }
}

mavros_msgs::WaypointList CNCArdupilot::getWaypointList() {
    return mWaypointList;
}

/*****************************************************
 * User Simple Functions
 */
// Goto Global Waypoint
bool CNCArdupilot::gotoGlobal(float x_lat, float y_long, float z_alt, bool isFromOAC) {
    // @TODO: check Guided mode
    if (!isFromOAC && OAC::ACTIVE_OAC_LEVEL > 1) {
        clearLocalMissionQueue();
        pushLocalMissionQueue(GPSPoint(x_lat, y_long, z_alt));
        return true;
    }
    if (!clearFCUWaypoint()) {
        return false;
    }
    if (pushWaypoints(x_lat, y_long, z_alt)) {
        mRecentWaypoint = GPSPoint(x_lat, y_long, z_alt);
        return true;
    }
    return false;
}

// Goto Relative Waypoint (North+, East+)
bool CNCArdupilot::gotoRelative(float x_lat, float y_long, float z_alt, bool isAltDelta, bool isFromOAC) {
    // @TODO: check GPS available
    // @TODO: consider changing altitude first
    GPSPoint tmpPoint = CNCUtility::getLocationMeter(getCurrentGPSPoint(), x_lat, y_long);
    return gotoGlobal(tmpPoint.latitude_, tmpPoint.longitude_, z_alt, isFromOAC);
}

// Goto Target Head
bool CNCArdupilot::gotoHeading(float heading, float distance, float z_alt, bool isFromOAC) {
    std::pair<float, float> tempRelative = CNCUtility::getNorthEastDistanceFromHeading(heading, distance);
    return gotoRelative(tempRelative.first, tempRelative.second, z_alt, false, isFromOAC);
}

// Move Mission
void CNCArdupilot::moveMissionToLocalQueue() {
    clearLocalMissionQueue();
    for (auto waypoint : mWaypointList.waypoints) {
        pushLocalMissionQueue(GPSPoint(waypoint.x_lat, waypoint.y_long, waypoint.z_alt));
    }
}

/***************************************************************************
 * Public Helper
 */
bool CNCArdupilot::checkFModeExist(std::string modeName) {
    GeneralUtility::toUpperCaseStr(&modeName);
    if (FLT_MODE_STABILIZE == modeName) return true;
    if (FLT_MODE_ACRO == modeName) return true;
    if (FLT_MODE_ALT_HOLD == modeName) return true;
    if (FLT_MODE_AUTO == modeName) return true;
    if (FLT_MODE_GUIDED == modeName) return true;
    if (FLT_MODE_LOITER == modeName) return true;
    if (FLT_MODE_RTL == modeName) return true;
    if (FLT_MODE_CIRCLE == modeName) return true;
    if (FLT_MODE_LAND == modeName) return true;
    if (FLT_MODE_OF_LOITER == modeName) return true;
    if (FLT_MODE_AUTOTUNE == modeName) return true;
    if (FLT_MODE_POSHOLD == modeName) return true;
    if (FLT_MODE_BRAKE == modeName) return true;
    if (FLT_MODE_AVOID_ADSB == modeName) return true;
    if (FLT_MODE_GUIDED_NOGPS == modeName) return true;
    return false;
}

/***************************************************************************
 * Accessor
 */
float CNCArdupilot::getTargetAltitude() {
    return mTargetAltitude;
}

GPSPoint CNCArdupilot::getTargetWaypoint() {
    return mRecentWaypoint;
}

bool CNCArdupilot::isGuided() {
    return mCurrentState.guided;
}

}  // namespace CNC
