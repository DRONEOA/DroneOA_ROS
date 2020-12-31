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
#include <typeindex>

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

bool DataPool::checkDataChanged(boost::any oldData, boost::any newData) {
    try {
        if (can_cast_to<bool>(oldData) && can_cast_to<bool>(newData)) {
            return boost::any_cast<bool>(oldData) != boost::any_cast<bool>(newData);
        } else if (can_cast_to<std::string>(oldData) && can_cast_to<std::string>(newData)) {
            return boost::any_cast<std::string>(oldData) != boost::any_cast<std::string>(newData);
        } else if (can_cast_to<int32_t>(oldData) && can_cast_to<int32_t>(newData)) {
            return boost::any_cast<int32_t>(oldData) != boost::any_cast<int32_t>(newData);
        } else if (can_cast_to<uint32_t>(oldData) && can_cast_to<uint32_t>(newData)) {
            return boost::any_cast<uint32_t>(oldData) != boost::any_cast<uint32_t>(newData);
        } else if (can_cast_to<float>(oldData) && can_cast_to<float>(newData)) {
            return boost::any_cast<float>(oldData) != boost::any_cast<float>(newData);
        } else if (can_cast_to<geometry_msgs::Quaternion>(oldData) &&
                can_cast_to<geometry_msgs::Quaternion>(newData)) {
            return boost::any_cast<geometry_msgs::Quaternion>(oldData) !=
                    boost::any_cast<geometry_msgs::Quaternion>(newData);;
        } else if (can_cast_to<geometry_msgs::Vector3>(oldData) &&
                can_cast_to<geometry_msgs::Vector3>(newData)) {
            return boost::any_cast<geometry_msgs::Vector3>(oldData) !=
                    boost::any_cast<geometry_msgs::Vector3>(newData);;
        } else {
            ROS_DEBUG("[DP] Compare any data failed");
            return false;
        }
    } catch(boost::bad_any_cast& e) {
        ROS_DEBUG("[DP] Compare any data failed");
        return false;
    }
}

void DataPool::setData(std::string name, boost::any data) {
    std::lock_guard<std::mutex> l(mContainerMutex);
    if ( mDataPoolContainer.find(name) == mDataPoolContainer.end() ) {
        // Not exist before
        mDataPoolContainer[name] = data;
        notifyAll(ENTRY_TYPES::DATA, name);
    } else {
        boost::any oldData = mDataPoolContainer[name];
        mDataPoolContainer[name] = data;
        if (checkDataChanged(oldData, mDataPoolContainer[name])) {
            notifyAll(ENTRY_TYPES::DATA, name);
        }
    }
}

void DataPool::setConfig(std::string name, boost::any data) {
    std::lock_guard<std::mutex> l(mContainerMutex);
    if ( mDataPoolContainer.find(name) == mDataPoolContainer.end() ) {
        // Not exist before
        mDataPoolContainer[name] = data;
        notifyAll(ENTRY_TYPES::CONFIG, name);
    } else {
        boost::any oldData = mDataPoolContainer[name];
        mDataPoolContainer[name] = data;
        if (checkDataChanged(oldData, mDataPoolContainer[name])) {
            notifyAll(ENTRY_TYPES::CONFIG, name);
        }
    }
}

bool DataPool::addEntryLocal(std::string name, boost::any initialData, bool isConfig) {
    // Verify entry not already exist
    if (isConfig) {
        if ( CONF_TYPE_MAP.find(name) != CONF_TYPE_MAP.end() ) {
            ROS_ERROR("[DP] Entry Already Exist");
            return false;
        }
        CONF_TYPE_MAP.insert({name, initialData.type()});
    } else {
        if ( DP_TYPE_MAP.find(name) != DP_TYPE_MAP.end() ) {
            ROS_ERROR("[DP] Entry Already Exist");
            return false;
        }
        DP_TYPE_MAP.insert({name, initialData.type()});
    }
    setData(name, initialData);
    return true;
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
