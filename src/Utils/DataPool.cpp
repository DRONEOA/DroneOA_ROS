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
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <droneoa_ros/Utils/DataPoolSubscriber.hpp>
#include <droneoa_ros/HWI/Utils/LocalPoint.hpp>
#include <droneoa_ros/HWI/Utils/GPSPoint.hpp>
#include <droneoa_ros/Utils/GeneralUtils.hpp>

namespace DP {

std::mutex DataPool::mContainerMutex;
std::map<std::string, boost::any> DataPool::mDataPoolContainer;
std::vector<DataPoolSubscriber*> DataPool::mSubscriberList;

DataPool::DataPool() {}

DataPool::~DataPool() {}

void DataPool::notifyAll(ENTRY_TYPES type, std::string entryName) {
    for (auto sub : mSubscriberList) {
        ENTRY_TYPES subscriberType = sub->getTypeToSubscribe();
        if (subscriberType == ENTRY_TYPES::NONE) {
            continue;
        }
        if (subscriberType == ENTRY_TYPES::ALL || type == subscriberType) {
            sub->onDataPoolUpdate(entryName);
        }
    }
}

boost::any DataPool::getData(std::string name) {
    std::lock_guard<std::mutex> l(mContainerMutex);
    for (auto entry : mDataPoolContainer) {
        std::string dbName = entry.first;
        GeneralUtility::toLowerCaseStr(&dbName);
        GeneralUtility::toLowerCaseStr(&name);
        if (dbName == name) {
            return entry.second;
        }
    }
    ROS_ERROR("[DP] Requested Data: %s Does Not Exist !!!", name.c_str());
    return boost::any();
}

std::string DataPool::getDataAsString(std::string name) {
    boost::any targetData = getData(name);
    if (targetData.empty()) return "ERROR";
    try {
        if (can_cast_to<bool>(targetData)) {
            return std::to_string(boost::any_cast<bool>(targetData));
        } else if (can_cast_to<std::string>(targetData)) {
            return boost::any_cast<std::string>(targetData);
        } else if (can_cast_to<uint8_t>(targetData)) {
            return std::to_string(boost::any_cast<uint8_t>(targetData));
        } else if (can_cast_to<int16_t>(targetData)) {
            return std::to_string(boost::any_cast<int16_t>(targetData));
        } else if (can_cast_to<uint16_t>(targetData)) {
            return std::to_string(boost::any_cast<uint16_t>(targetData));
        } else if (can_cast_to<int32_t>(targetData)) {
            return std::to_string(boost::any_cast<int32_t>(targetData));
        } else if (can_cast_to<uint32_t>(targetData)) {
            return std::to_string(boost::any_cast<uint32_t>(targetData));
        } else if (can_cast_to<float>(targetData)) {
            return std::to_string(boost::any_cast<float>(targetData));
        } else if (can_cast_to<LocalPoint>(targetData)) {
            return boost::any_cast<LocalPoint>(targetData).AsString();
        } else if (can_cast_to<GPSPoint>(targetData)) {
            return boost::any_cast<GPSPoint>(targetData).AsString();
        } else if (can_cast_to<geometry_msgs::Quaternion>(targetData)) {
            geometry_msgs::Quaternion geo_msg = boost::any_cast<geometry_msgs::Quaternion>(targetData);
            std::string result = "w:" + std::to_string(geo_msg.w) +
                                " x:" + std::to_string(geo_msg.x) +
                                " y:" + std::to_string(geo_msg.y) +
                                " z:" + std::to_string(geo_msg.z);
            return result;
        } else if (can_cast_to<geometry_msgs::Vector3>(targetData)) {
            geometry_msgs::Vector3 geo_msg = boost::any_cast<geometry_msgs::Vector3>(targetData);
            std::string result = "x:" + std::to_string(geo_msg.x) +
                                " y:" + std::to_string(geo_msg.y) +
                                " z:" + std::to_string(geo_msg.z);
            return result;
        } else {
            ROS_WARN("[DP] Unknown Data Type");
        }
    } catch(boost::bad_any_cast& e) {
        ROS_ERROR("[DP] Convert any data to string failed");
    }
    return "ERROR";
}

int DataPool::getDataAsInt(std::string name) {
    boost::any targetData = getData(name);
    if (targetData.empty()) {
        //! @note Data does not exist yet
        return 0;
    }
    try {
        if (can_cast_to<bool>(targetData)) {
            return boost::any_cast<bool>(targetData);
        } else if (can_cast_to<std::string>(targetData)) {
            return std::stoi(boost::any_cast<std::string>(targetData));
        } else if (can_cast_to<uint8_t>(targetData)) {
            return boost::any_cast<uint8_t>(targetData);
        } else if (can_cast_to<int16_t>(targetData)) {
            return boost::any_cast<int16_t>(targetData);
        } else if (can_cast_to<uint16_t>(targetData)) {
            return boost::any_cast<uint16_t>(targetData);
        } else if (can_cast_to<int32_t>(targetData)) {
            return boost::any_cast<int32_t>(targetData);
        } else if (can_cast_to<uint32_t>(targetData)) {
            return boost::any_cast<uint32_t>(targetData);
        } else {
            ROS_ERROR("Data type cannot convert to int");
        }
    } catch(boost::bad_any_cast& e) {
        ROS_ERROR("Fail to convert to int");
    }
    return 0;
}

void DataPool::setData(std::string name, boost::any data) {
    std::lock_guard<std::mutex> l(mContainerMutex);
    mDataPoolContainer[name] = data;
    notifyAll(ENTRY_TYPES::DATA, name);
}

void DataPool::setConfig(std::string name, boost::any data) {
    std::lock_guard<std::mutex> l(mContainerMutex);
    mDataPoolContainer[name] = data;
    notifyAll(ENTRY_TYPES::CONFIG, name);
}

void DataPool::registerEvents(DataPoolSubscriber* subscriber) {
    if (subscriber) {
        mSubscriberList.push_back(subscriber);
    }
}

void DataPool::printAllEntry() {
    ROS_INFO("[DP] All DataPool Entries:");
    for (auto entry : mDataPoolContainer) {
        ROS_INFO("    %s", entry.first.c_str());
    }
}

void DataPool::printAllEntryWithData() {
    ROS_INFO("[DP] All DataPool Entries And Data:");
    for (auto entry : mDataPoolContainer) {
        ROS_INFO("    %-30s %-50s", entry.first.c_str(), getDataAsString(entry.first).c_str());
    }
}

}  // namespace DP
