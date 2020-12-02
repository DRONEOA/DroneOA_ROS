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

#include <droneoa_ros/Utils/DataPoolServer.hpp>
#include <droneoa_ros/PDN.hpp>

namespace DP {

DataPoolServer::DataPoolServer() {
    setDefaultConfig();
    mGetService = mNodeHandle.advertiseService(DP_GET_SERVICE_NAME,
        &DataPoolServer::handleGetDataRequest, this);
    mSetService = mNodeHandle.advertiseService(DP_SET_SERVICE_NAME,
        &DataPoolServer::handleSetDataRequest, this);
    mGetStrService = mNodeHandle.advertiseService(DP_GET_STR_SERVICE_NAME,
        &DataPoolServer::handleGetStrDataRequest, this);
    thread_publish_sync_data_ = new boost::thread(boost::bind(&DataPoolServer::publishSyncDataThread, this));
}

DataPoolServer::~DataPoolServer() {
    if (thread_publish_sync_data_) delete thread_publish_sync_data_;
    mDataPoolContainer.clear();
}

void DataPoolServer::setDefaultConfig() {
    setData(CONF_SAFETY_GPS_FIX, true);
    setData(DP_ACTIVE_OAC_LEVEL, int32_t(0));
}

bool DataPoolServer::handleGetStrDataRequest(droneoa_ros::GetDPDataAsString::Request  &req,
        droneoa_ros::GetDPDataAsString::Response &res) {
    std::string targetEntryName = req.entry_name;
    res.data_str = getDataAsString(targetEntryName);
    if (res.data_str == "ERROR") return false;
    return true;
}

bool DataPoolServer::handleGetDataRequest(droneoa_ros::GetDPData::Request  &req,
        droneoa_ros::GetDPData::Response &res) {
    std::string targetEntryName = req.entry_name;
    boost::any targetData = getData(targetEntryName);
    if (targetData.empty()) {
        return false;
    }
    try {
        if (can_cast_to<bool>(targetData)) {
            res.data_bool = boost::any_cast<bool>(targetData);
        } else if (can_cast_to<std::string>(targetData)) {
            res.data_str = boost::any_cast<std::string>(targetData);
        } else if (can_cast_to<int32_t>(targetData)) {
            res.data_int32 = boost::any_cast<int32_t>(targetData);
        } else if (can_cast_to<uint32_t>(targetData)) {
            res.data_uint32 = boost::any_cast<uint32_t>(targetData);
        } else if (can_cast_to<float>(targetData)) {
            res.data_float = boost::any_cast<float>(targetData);
        } else if (can_cast_to<geometry_msgs::Quaternion>(targetData)) {
            res.data_quat = boost::any_cast<geometry_msgs::Quaternion>(targetData);
        } else if (can_cast_to<geometry_msgs::Vector3>(targetData)) {
            res.data_vec3 = boost::any_cast<geometry_msgs::Vector3>(targetData);
        } else {
            return false;
        }
    } catch(boost::bad_any_cast& e) {
        return false;
    }
    return true;
}

bool DataPoolServer::handleSetDataRequest(droneoa_ros::SetDPData::Request  &req,
        droneoa_ros::SetDPData::Response &res) {
    std::string targetEntryName = req.entry_name;
    res.success = false;
    // Find entry in type map
    if ( DP_TYPE_MAP.find(targetEntryName) != DP_TYPE_MAP.end() ) {
        std::type_index requiredType = DP_TYPE_MAP.at(targetEntryName);
        if (requiredType == typeid(bool)) {
            setData(targetEntryName, req.data_bool);
        } else if (requiredType == typeid(std::string)) {
            setData(targetEntryName, req.data_str);
        } else if (requiredType == typeid(int32_t)) {
            setData(targetEntryName, req.data_int32);
        } else if (requiredType == typeid(uint32_t)) {
            setData(targetEntryName, req.data_uint32);
        } else if (requiredType == typeid(float)) {
            setData(targetEntryName, req.data_float);
        } else if (requiredType == typeid(geometry_msgs::Vector3)) {
            setData(targetEntryName, req.data_vec3);
        } else if (requiredType == typeid(geometry_msgs::Quaternion)) {
            setData(targetEntryName, req.data_quat);
        } else {
            ROS_ERROR("[DataPool Server] Set operation is not supported for this data type.");
            return false;
        }
        res.success = true;
        return true;
    }
    ROS_ERROR("[DataPool Server] Set Data Operation Aborted: type mapping not exist");
    return false;
}

droneoa_ros::DataPoolSync DataPoolServer::buildSyncData() {
    droneoa_ros::DataPoolSync message;
    for (auto entry : mDataPoolContainer) {
        droneoa_ros::DataPoolPair line;
        line.entry_name = entry.first;
        boost::any targetData = getData(entry.first);
        if (targetData.empty()) {
            continue;
        }
        try {
            if (can_cast_to<bool>(targetData)) {
                line.data_bool = boost::any_cast<bool>(targetData);
            } else if (can_cast_to<std::string>(targetData)) {
                line.data_str = boost::any_cast<std::string>(targetData);
            } else if (can_cast_to<int32_t>(targetData)) {
                line.data_int32 = boost::any_cast<int32_t>(targetData);
            } else if (can_cast_to<uint32_t>(targetData)) {
                line.data_uint32 = boost::any_cast<uint32_t>(targetData);
            } else if (can_cast_to<float>(targetData)) {
                line.data_float = boost::any_cast<float>(targetData);
            } else if (can_cast_to<geometry_msgs::Quaternion>(targetData)) {
                line.data_quat = boost::any_cast<geometry_msgs::Quaternion>(targetData);
            } else if (can_cast_to<geometry_msgs::Vector3>(targetData)) {
                line.data_vec3 = boost::any_cast<geometry_msgs::Vector3>(targetData);
            } else {
                continue;
            }
        } catch(boost::bad_any_cast& e) {
            continue;
        }
        message.data_pairs.push_back(line);
    }
    return message;
}

void DataPoolServer::publishSyncDataThread() {
    ros::Rate rate(DP_SYNC_RATE);
    ros::Publisher syncDataPublisher =
        mNodeHandle.advertise<droneoa_ros::DataPoolSync>(DP_SYNC_TOPIC_NAME, 10);
    while (mNodeHandle.ok()) {
        syncDataPublisher.publish(buildSyncData());
        rate.sleep();
    }
}

}  // namespace DP
