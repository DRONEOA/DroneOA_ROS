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

#ifndef UTILS_DATAPOOL_Client_  // NOLINT
#define UTILS_DATAPOOL_Client_  // NOLINT

#include <droneoa_ros/DataPoolPair.h>
#include <droneoa_ros/DataPoolSync.h>

#include <droneoa_ros/Utils/DataPool.hpp>
#include <boost/thread.hpp>

namespace DP {

class DataPoolClient : public DataPool {
    // Handle DataPool Sync Data
    boost::thread* thread_watch_sync_data_ = nullptr;
    void updateDPWithSyncMsg(const droneoa_ros::DataPoolSync::ConstPtr& msg);
    void watchSYNCDataThread();
 public:
    DataPoolClient();
    virtual ~DataPoolClient();
};

}  // namespace DP

#endif  // UTILS_DATAPOOL_Client_  // NOLINT
