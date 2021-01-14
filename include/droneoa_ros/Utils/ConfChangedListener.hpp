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
 * Written by Clair Hu <clair.hu.1@uwaterloo.ca>, Dec 2020
 */

#ifndef UTILS_CONF_CHANGED_LISTENER_  // NOLINT
#define UTILS_CONF_CHANGED_LISTENER_  // NOLINT

#include <string>
#include <boost/thread.hpp>
#include <droneoa_ros/Utils/DataPool.hpp>
#include <droneoa_ros/Utils/JsonUtils.hpp>
#include <droneoa_ros/Utils/DataPoolEntries.hpp>

namespace CONF {

class ConfChangedListener {
    void watchConfFileThread();
    DP::DataPool mDataPool;
    float mConfigCheckChangesFrequency;
    boost::thread* thread_watch_command_ = nullptr;
    JSON::JsonUtils* mJsonUtils;

 public:
    explicit ConfChangedListener(std::string filePath);
    virtual ~ConfChangedListener();
    virtual void conf_callback();
};

}  // namespace CONF

#endif  // NOLINT
