/* Copyright (C) DroneOA Group - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 * Written by Bohan Shi <b34shi@edu.uwaterloo.ca>, August 2019
 */

#ifndef INCLUDE_DRONEOA_ROS_CNCINTERFACEPX4_HPP_  // NOLINT
#define INCLUDE_DRONEOA_ROS_CNCINTERFACEPX4_HPP_  // NOLINT

#include <boost/bind.hpp>
#include <boost/thread/thread.hpp>

#include <ros/ros.h>

#include <sensor_msgs/NavSatFix.h>
#include <mavros_msgs/HomePosition.h>
#include <std_msgs/Float64.h>

class CNCInterfacePX4 {
 public:
    CNCInterfacePX4();
    virtual ~CNCInterfacePX4();
    void init(ros::NodeHandle nh, ros::Rate r);

    /* Mode Control */
    bool setMode(std::string modeName);

    /* Safety Feature */
    bool armVehicle();

    /* Flight Services */
    bool takeoff(double dz);
    bool land();

    /* Public Getter */
    mavros_msgs::HomePosition getHomePosition();
    std_msgs::Float64 getRelativeAltitude();

 private:
    ros::NodeHandle nh_;
    ros::Rate rate_ = ros::Rate(1.0);

    bool home_set_ = false;

    /* Data */
    mavros_msgs::HomePosition home_{};
    std_msgs::Float64 relative_altitude_;

    boost::thread* thread_watch_alt_ = nullptr;

    /* CB Callbacks */
    void setHomeGeoPointCB(const mavros_msgs::HomePositionConstPtr& home);
    void setRelativeAltitudeCB(const std_msgs::Float64ConstPtr& msg);

    /* Threads */
    void watchAltitudeThread();

    /* Private Getter */
    void getHomeGeoPoint();
};

#endif