/* Copyright (C) 2019 DroneOA Group - All Rights Reserved
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
 * Written by Bohan Shi <b34shi@edu.uwaterloo.ca>, Feb. 2020
 * All Reference Attached
 */

#include <math.h>
#include <algorithm>
#include <cctype>
#include <fstream>
#include <sstream>
#include <droneoa_ros/Utils/GeneralUtils.hpp>

// #define RAD2DEG(x) ((x)*180./M_PI)
float GeneralUtility::radToDeg(float rad) {
    float degAngle = rad * 180.0 / M_PI;
    return degAngle;
}

std::vector<float> GeneralUtility::getFloatDataFromConfig(std::string path, std::string keyName) {
    std::ifstream infile(path);
    std::string line;
    std::vector<float> result;
    while (std::getline(infile, line)) {
        if (line.empty()) {
            continue;
        }
        std::istringstream iss(line);
        std::string s;
        iss >> s;
        if (s == keyName) {
            for (; iss >> s; ) {
                result.push_back(std::stof(s));
            }
            break;
        }
    }
    return result;
}

void GeneralUtility::toLowerCaseStr(std::string* input) {
    std::transform(input->begin(), input->end(), input->begin(), ::tolower);
}

void GeneralUtility::toUpperCaseStr(std::string* input) {
    std::transform(input->begin(), input->end(), input->begin(), ::toupper);
}

void GeneralUtility::removeSpaces(std::string *cmd) {
    std::istringstream iss(*cmd);
    std::string word;
    std::string out;
    while (iss >> word) {
        if (!out.empty()) {
            out += ' ';
        }
        out += word;
    }
    *cmd = out;
}
