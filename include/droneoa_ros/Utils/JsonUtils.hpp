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

#ifndef UTILS_JSON_UTILS_  // NOLINT
#define UTILS_JSON_UTILS_  // NOLINT

#include <json/value.h>
#include <jsoncpp/json/json.h>
#include <vector>
#include <string>
#include <unordered_map>
#include <fstream>
#include <iostream>
#include <exception>

class JsonUtils {
    std::string mFilePath;
    std::vector<std::string> mParamNames;
    std::unordered_map<std::string, std::string> mParams;

 public:
    explicit JsonUtils(std::string filePath);
    virtual ~JsonUtils();
    std::unordered_map<std::string, std::string> readJsonToMap();
    bool writeMapToJson(std::unordered_map<std::string, std::string> map);
};

#endif  // NOLINT
