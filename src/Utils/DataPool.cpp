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
 * Written by Bohan Shi <b34shi@uwaterloo.ca>, Oct 2020
 */

#include <droneoa_ros/Utils/DataPool.hpp>

namespace DP {

std::mutex DataPool::mContainerMutex;
std::map<std::string, boost::any> DataPool::mDataPoolContainer;

DataPool::DataPool() {}

DataPool::~DataPool() {
    mDataPoolContainer.clear();
}

boost::any DataPool::getData(std::string name) {
    std::lock_guard<std::mutex> l(mContainerMutex);
    if (mDataPoolContainer.find(name) == mDataPoolContainer.end()) {
        ROS_ERROR("[DP] Requested Data: %s Does Not Exist !!!", name.c_str());
        return boost::any();
    }
    return mDataPoolContainer[name];
}

void DataPool::setData(std::string name, boost::any data) {
    std::lock_guard<std::mutex> l(mContainerMutex);
    mDataPoolContainer[name] = data;
}

void DataPool::printAllEntry() {
    ROS_INFO("[DP] DataPool Full Info:");
    for (auto entry : mDataPoolContainer) {
        ROS_INFO("    %s", entry.first.c_str());
    }
}

}  // namespace DP
