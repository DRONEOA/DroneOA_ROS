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

#include <droneoa_ros/Utils/ConfChangedListener.hpp>
#include <ros/ros.h>
#include <fstream>
#include <droneoa_ros/Utils/JsonUtils.hpp>
#include <droneoa_ros/Utils/DataPool.hpp>

namespace CONF {

ConfChangedListener::ConfChangedListener(std::string filePath) {
    thread_watch_command_ = new boost::thread(boost::bind(&ConfChangedListener::watchConfFileThread, this));
    mJsonUtils = new JSON::JsonUtils(filePath);
}

ConfChangedListener::~ConfChangedListener() {
    if (mJsonUtils) {
        delete(mJsonUtils);
    }
    if (thread_watch_command_) {
        delete(thread_watch_command_);
    }
}

void ConfChangedListener::conf_callback() {
    ROS_DEBUG("[Conf] watch conf file thread while loop");
    for (auto param : mJsonUtils->readJsonToMap()) {
        try {
            std::type_index requiredType = DP::CONF_TYPE_MAP.at(param.first);
            if (requiredType == typeid(std::string)) {
                mDataPool.setConfig(param.first, param.second);
            } else if (requiredType == typeid(int32_t)) {
                mDataPool.setConfig(param.first, boost::lexical_cast<int32_t>(param.second));
            } else if (requiredType == typeid(uint32_t)) {
                mDataPool.setConfig(param.first, boost::lexical_cast<uint32_t>(param.second));
            } else if (requiredType == typeid(float)) {
                mDataPool.setConfig(param.first, boost::lexical_cast<float>(param.second));
            } else if (requiredType == typeid(bool)) {
                std::transform(param.second.begin(), param.second.end(), param.second.begin(), ::tolower);
                if (param.second == "true" || param.second == "1") {
                    mDataPool.setConfig(param.first, true);
                } else if (param.second == "false" || param.second == "0") {
                    mDataPool.setConfig(param.first, false);
                }
            }
        } catch (boost::bad_lexical_cast &e) {
            ROS_WARN("[Conf] User defined config parameter type not correct;  %s", e.what());
        } catch (const std::out_of_range& e) {
            continue;
        }
    }
}

void ConfChangedListener::watchConfFileThread() {
    mConfigCheckChangesFrequency = boost::any_cast<float>(mDataPool.getData(DP::CONF_CHECK_CHANGES_FREQUENCY));
    auto rate = ros::Rate(mConfigCheckChangesFrequency);
    while (ros::ok) {
        conf_callback();
        mConfigCheckChangesFrequency = boost::any_cast<float>(mDataPool.getData(DP::CONF_CHECK_CHANGES_FREQUENCY));
        rate = ros::Rate(mConfigCheckChangesFrequency);
        rate.sleep();
    }
}

}  // namespace CONF
