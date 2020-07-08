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

#ifndef HWI_BASE_LIDARGENERIC_HPP_  // NOLINT
#define HWI_BASE_LIDARGENERIC_HPP_  // NOLINT

#include <vector>
#include <utility>
#include <string>
#include <map>
#include <boost/bind.hpp>
#include <boost/thread/thread.hpp>

#include <droneoa_ros/PDN.hpp>
#include <droneoa_ros/HWI/interface/LidarInterface.hpp>

namespace Lidar {

typedef std::vector<float> degreeSector;

class LidarGeneric : public LidarInterface {
 public:
    LidarGeneric(ros::NodeHandle node, ros::Rate rate);
    virtual ~LidarGeneric();

    /***************************************************************************
     * Init
     */
    void initWatcherThread() override;
    /**
     * @brief Change lidar data source [USE WITH CAUTION]
     * @param lidarSource string of the new lidar data source
     */
    void changeLidarSource(std::string lidarSource);

    /***************************************************************************
     * Basic Data
     */
    float getMaxAngle() override;  /*!< Unit: rad */
    float getMinAngle() override;  /*!< Unit: rad */
    float getMaxRange() override;  /*!< Unit: rad */
    float getMinRnage() override;  /*!< Unit: rad */
    float getScanTime() override; /*!< Unit: s */
    float getTimeIncreament() override;  /*!< Unit: s */
    float getAngleIncreament() override;  /*!< Unit: rad */

    /***************************************************************************
     * Data Stream
     */
    sensor_msgs::LaserScan getRawDataMap() override;
    std::map<float, degreeSector> getSectorDataMap();  /*!< Unit: m, per degree */
    std::pair<float, float> getClosestSectorData();  /*!< Unit: m */

    /***************************************************************************
     * Debug
     */
    void printLidarInfo() override;

    /***************************************************************************
     * Callback
     */
    void lidar_callback(const sensor_msgs::LaserScanConstPtr& msg);

 protected:
    ros::NodeHandle mNodeHandle;
    ros::Rate mRate = ros::Rate(GLOBAL_ROS_RATE);

    /***************************************************************************
     * Data
     */
    std::string mCurrentLidarSource;
    sensor_msgs::LaserScan mScannerData;  // Raw data
    std::map<float, degreeSector> mSectorDataMap;  // Degree - Range map

    /***************************************************************************
     * Threads
     */
    boost::thread* mpThreadWatchLidar = nullptr;
    void watchLidarThread();

    // Subscriber
    ros::Subscriber mLidarSub;

    // Processor
    virtual void generateDataMap();
    virtual void generateDegreeSector();
};

}  // namespace Lidar

#endif  // HWI_BASE_LIDARGENERIC_HPP_  // NOLINT
