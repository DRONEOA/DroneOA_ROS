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
 * Written by Bohan Shi <b34shi@uwaterloo.ca>, Nov 2020
 */

#ifndef UTILS_DATAPOOL_SERVER_  // NOLINT
#define UTILS_DATAPOOL_SERVER_  // NOLINT

#include <ros/ros.h>
#include <droneoa_ros/GetDPData.h>
#include <droneoa_ros/SetDPData.h>
#include <droneoa_ros/GetDPDataAsString.h>
#include <droneoa_ros/DataPoolPair.h>
#include <droneoa_ros/DataPoolSync.h>

#include <droneoa_ros/Utils/DataPool.hpp>
#include <boost/thread.hpp>

namespace DP {

/*******************************************************************************
 * @brief DataPoolServer
 * Listen:
 * - Option1:
 *   Request specific data via services
 * - Option2:
 *   MainNode Pkg (DataPoolServer)
 *              | sync msg 1hz
 *              v
 *   APP Pkg with APP Node (DataPool)
 *              | use DP subscriber to pick data within package
 *              v
 *   APP Node (DataPoolSubscriber)
 * 
 * Modify:
 *   Use service to modify
 */
class DataPoolServer : public DataPool {
    ros::NodeHandle mNodeHandle;
    /**
     * @brief Set configs with Default configuration set
     */
    void setDefaultConfig();
    /**
     * @brief Publish DataPool data to sync with clients
     */
    boost::thread* thread_publish_sync_data_ = nullptr;
    droneoa_ros::DataPoolSync buildSyncData();
    void publishSyncDataThread();
    /**
     * @brief Handle Get / Set Services
     */
    ros::ServiceServer mGetService;
    ros::ServiceServer mGetStrService;
    ros::ServiceServer mSetService;
    bool handleGetDataRequest(droneoa_ros::GetDPData::Request  &req,  // NOLINT
            droneoa_ros::GetDPData::Response &res);  // NOLINT
    bool handleGetStrDataRequest(droneoa_ros::GetDPDataAsString::Request  &req,  // NOLINT
            droneoa_ros::GetDPDataAsString::Response &res);  // NOLINT
    bool handleSetDataRequest(droneoa_ros::SetDPData::Request  &req,  // NOLINT
            droneoa_ros::SetDPData::Response &res);  // NOLINT

 public:
    DataPoolServer();
    virtual ~DataPoolServer();
};

}  // namespace DP

#endif  // UTILS_DATAPOOL_SERVER_  // NOLINT
