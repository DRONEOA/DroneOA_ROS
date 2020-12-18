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

#include <droneoa_ros/Utils/JsonUtils.hpp>
#include <ros/ros.h>

JsonUtils::JsonUtils(std::string filePath) {
    mFilePath = filePath;
}

JsonUtils::~JsonUtils() {
}

std::unordered_map<std::string, std::string> JsonUtils::readJsonToMap() {
    Json::Value config;
    std::ifstream ifs;
    try {
        ifs = std::ifstream(mFilePath);
        ifs >> config;
    } catch (std::exception e) {
        ROS_WARN("%s", e.what());
    }
    mParamNames = config.getMemberNames();
    for (auto parameter : mParamNames) {
        mParams[parameter] = config[parameter].asString();
    }
    ifs.close();
    
    return mParams;
}

bool JsonUtils::writeMapToJson(std::unordered_map<std::string, std::string> map) {
    std::ofstream json;
    json.open(mFilePath);

    try {
        Json::Value value;
        for (auto param : map) {
            value[param.first] = Json::Value(param.second);
        }

        Json::StyledWriter styledWriter;
        json << styledWriter.write(value);

    } catch (std::exception e) {
        ROS_WARN("%s", e.what());
    }

    json.close();

    return true;
}
