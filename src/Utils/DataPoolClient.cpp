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

#include <droneoa_ros/Utils/DataPoolClient.hpp>
#include <droneoa_ros/PDN.hpp>

namespace DP {

void DataPoolClient::updateDPWithSyncMsg(const droneoa_ros::DataPoolSync::ConstPtr& msg) {
    for (droneoa_ros::DataPoolPair line : (*msg).data_pairs) {
        // Find entry in type map
        bool isConfig = false;
        std::type_index requiredType = typeid(bool);
        if ( DP_TYPE_MAP.find(line.entry_name) != DP_TYPE_MAP.end() ) {
            isConfig = false;
            requiredType = DP_TYPE_MAP.at(line.entry_name);
        } else if ( CONF_TYPE_MAP.find(line.entry_name) != CONF_TYPE_MAP.end() ) {
            isConfig = true;
            requiredType = CONF_TYPE_MAP.at(line.entry_name);
        } else {
            // Customized Entry will trigger a lot of this warning message
            //! @todo better logging solution?
            // ROS_WARN("[DP Client] Missing mapping for: %s. Ignored.", line.entry_name.c_str());
            continue;
        }
        // Type checking & Update Value
        if (requiredType == typeid(bool)) {
            if (isConfig) {
                setConfig(line.entry_name, line.data_bool);
                continue;
            }
            setData(line.entry_name, line.data_bool);
        } else if (requiredType == typeid(std::string)) {
            if (isConfig) {
                setConfig(line.entry_name, line.data_str);
                continue;
            }
            setData(line.entry_name, line.data_str);
        } else if (requiredType == typeid(int32_t)) {
            if (isConfig) {
                setConfig(line.entry_name, line.data_int32);
                continue;
            }
            setData(line.entry_name, line.data_int32);
        } else if (requiredType == typeid(uint32_t)) {
            if (isConfig) {
                setConfig(line.entry_name, line.data_uint32);
                continue;
            }
            setData(line.entry_name, line.data_uint32);
        } else if (requiredType == typeid(float)) {
            if (isConfig) {
                setConfig(line.entry_name, line.data_float);
                continue;
            }
            setData(line.entry_name, line.data_float);
        } else if (requiredType == typeid(geometry_msgs::Vector3)) {
            if (isConfig) {
                setConfig(line.entry_name, line.data_vec3);
                continue;
            }
            setData(line.entry_name, line.data_vec3);
        } else if (requiredType == typeid(geometry_msgs::Quaternion)) {
            if (isConfig) {
                setConfig(line.entry_name, line.data_quat);
                continue;
            }
            setData(line.entry_name, line.data_quat);
        } else {
            ROS_WARN("[DP Client] Miss match type for: %s. Ignored.", line.entry_name.c_str());
        }
    }
}

void DataPoolClient::watchSYNCDataThread() {
    auto rate = ros::Rate(10);
    auto node = boost::make_shared<ros::NodeHandle>();
    auto data_sub =
        node->subscribe<droneoa_ros::DataPoolSync>(DP_SYNC_TOPIC_NAME, 1,
            boost::bind(&DataPoolClient::updateDPWithSyncMsg, this, _1));

    while (ros::ok()) {
        ros::spinOnce();
        rate.sleep();
    }
}

DataPoolClient::DataPoolClient() {
    thread_watch_sync_data_ = new boost::thread(boost::bind(&DataPoolClient::watchSYNCDataThread, this));
}

DataPoolClient::~DataPoolClient() {
    if (thread_watch_sync_data_) delete thread_watch_sync_data_;
}

}  // namespace DP
