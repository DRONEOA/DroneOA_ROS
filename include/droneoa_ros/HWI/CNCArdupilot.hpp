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

#ifndef HWI_CNCARDUPILOT_HPP_  // NOLINT
#define HWI_CNCARDUPILOT_HPP_  // NOLINT

#include <ros/ros.h>
#include <mavros_msgs/Waypoint.h>
#include <mavros_msgs/WaypointPush.h>
#include <mavros_msgs/WaypointPull.h>
#include <mavros_msgs/WaypointClear.h>
#include <mavros_msgs/WaypointReached.h>
#include <mavros_msgs/WaypointList.h>

#include <string>
#include <vector>

#include <droneoa_ros/HWI/base/CNCGeneric.hpp>

namespace CNC {

class CNCArdupilot : public CNCGeneric{
    std::vector<std::function<void()>> mpWpReachCallbackList;
    mavros_msgs::WaypointList mWaypointList;

    boost::thread* mpThreadWatchWPReach = nullptr;
    boost::thread* mpThreadWatchWPList = nullptr;
    void watchReachThread();
    void watchWPListThread();

 public:
    CNCArdupilot(ros::NodeHandle node, ros::Rate rate);
    virtual ~CNCArdupilot();
    void initWatcherThread() override;

    /***************************************************************************
     * Mission
     */
    /**
     * @brief Push New Waypoint
     * @param x_lat
     * @param y_long
     * @param z_alt
     * @param isCurrent (2 = isCurrent Guided)
     * @param command (default NAV_WAYPOINT)
     * @return client send response
     */
    bool pushWaypoint(float x_lat, float y_long, float z_alt, uint8_t isCurrent = 2,
        uint16_t command = mavros_msgs::CommandCode::NAV_WAYPOINT);
    /**
     * @brief Push New Waypoint List
     * @param std::vector<GPSPoint> (Waypoints Global)
     * @param isCurrent (2 = isCurrent Guided)
     * @param command (default NAV_WAYPOINT)
     * @return client send response
     */
    bool pushWaypoints(const std::vector<GPSPoint> &wpList, uint8_t isCurrent = 2,
        uint16_t command = mavros_msgs::CommandCode::NAV_WAYPOINT);
    mavros_msgs::WaypointPull pullWaypoints();
    bool clearFCUWaypoint() override;
    void registForReachEvent(std::function<void()> callback);
    mavros_msgs::WaypointList getWaypointList();

    void WP_reach_callback(const mavros_msgs::WaypointReachedConstPtr& msg);
    void WP_list_callback(const mavros_msgs::WaypointListConstPtr& msg);

    /***************************************************************************
     * User Simple Function
     */
    /**
     * @brief Goto Global Waypoint, 3D GPS Point
     * @param x_lat
     * @param y_long
     * @param z_alt in meter
     * @return client send response
     */
    bool gotoGlobal(float x_lat, float y_long, float z_alt, bool isFromOAC = false) override;
    /**
     * @brief Goto Relative Waypoint (North+, East+)
     * @param x_lat North+ in meter
     * @param y_long East+ in meter
     * @param z_alt default 10 in meter
     * @param isAltDelta not used
     * @return client send response
     */
    bool gotoRelative(float north, float east, float z_alt = 10, bool isAltDelta = false,
            bool isFromOAC = false) override;
    /**
     * @brief Goto Target Head
     * @param heading in degree
     * @param distance in meter
     * @param z_alt in meter
     * @return client send response
     */
    bool gotoHeading(float heading, float distance, float z_alt, bool isFromOAC = false) override;

    /**
     * @brief Using pushWaypoints if isGlobal = true, use setpoint otherwise (WIP)
     * 
     * @param wpList list of missions (GPS if global, relative to last init location if local)
     * @param isGlobal whether the mission is using global (GPS) coordination system
     * @return client send response 
     */
    bool pushMission(const std::vector<GPSPoint> &wpList, bool isGlobal = true) override;

    /**
     * @brief Move missions in FCU queue to local mission queue. (For OA Operation)
     */
    void moveMissionToLocalQueue() override;

    /***************************************************************************
     * Public Helper
     */
    /**
     * @brief Check if a flight mode name is valid
     * @param modeName 
     * @return true if name is valid
     */
    bool checkFModeExist(std::string modeName) override;

    /***************************************************************************
     * Accessor
     */
    float getTargetAltitude();
    GPSPoint getTargetWaypoint() override;
    bool isGuided();
};

}  // namespace CNC

#endif  // HWI_CNCARDUPILOT_HPP_  // NOLINT
