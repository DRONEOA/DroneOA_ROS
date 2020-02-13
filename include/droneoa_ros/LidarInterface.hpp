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

#ifndef INCLUDE_DRONEOA_ROS_LIDARINTERFACE_HPP_  // NOLINT
#define INCLUDE_DRONEOA_ROS_LIDARINTERFACE_HPP_  // NOLINT

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

#include <map>
#include <vector>
#include <string>
#include <utility>
#include <boost/bind.hpp>
#include <boost/thread/thread.hpp>
#include <opencv2/highgui/highgui.hpp>

// #define DEBUG_LIDAR_POPUP

typedef std::vector<float> degreeSector;

static const char* LIDAR_SOURCE_YDLIDAR = "/scan";
static const char* LIDAR_SOURCE_UE4 = "/sitl_lidar_test";

class LidarInterface {
 public:
    LidarInterface();
    virtual ~LidarInterface();
    /**
     * @brief Init the Lidar Interface
     * Seperated from constructor due to the planned restart feature
     * @param nh ROS Node Handler
     * @param r Rate of ROS node
     */
    void init(ros::NodeHandle nh, ros::Rate r);

    // Callback
    void lidar_callback(const sensor_msgs::LaserScanConstPtr& msg);

    // Accesser
    float getMaxAngle();  /*!< Unit: rad */
    float getMinAngle();  /*!< Unit: rad */
    float getMaxRange();  /*!< Unit: rad */
    float getMinRnage();  /*!< Unit: rad */
    std::map<float, degreeSector> getScannerDataMap();  /*!< Unit: m */
    float getIncreament();  /*!< Unit: rad */
    std::pair<float, float> getClosestSectorData();  /*!< Unit: m */

    /**
     * @brief Change lidar data source [USE WITH CAUTION]
     * @param lidarSource string of the new lidar data source
     */
    void changeLidarSource(std::string lidarSource);

    // Debug
    void printLidarInfo();

 private:
    ros::NodeHandle n_;
    ros::Rate r_ = ros::Rate(10.0);

    // Data
    std::string currentLidarSource_;
    sensor_msgs::LaserScan scannerData_;  // Raw data
    std::map<float, degreeSector> scannerDataMap_;  // Degree - Range map

    // Threads
    boost::thread* thread_watch_lidar_ = nullptr;
    void watchLidarThread();

    // Subscriber
    ros::Subscriber lidar_sub_;

    // Processor
    void generateDataMap();
    void generateDegreeSector();

    // Debug
    void drawLidarData();
};

#endif  // INCLUDE_DRONEOA_ROS_LIDARINTERFACE_HPP_  // NOLINT
