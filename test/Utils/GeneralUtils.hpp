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
 * Written by Bohan Shi <b34shi@edu.uwaterloo.ca>, April 2020
 */

#ifndef UT_GENERALUTILS_HPP_  // NOLINT
#define UT_GENERALUTILS_HPP_  // NOLINT

#include <fstream>
#include <string>
#include <cstdint>

static const char* logFilePath = "./unittest_result.txt";

bool logTestResult(const std::string &testName, uint32_t ret) {
    std::ofstream outfile;
    outfile.open(logFilePath, std::ios_base::app);
    if (outfile.fail()) {
        outfile.open(logFilePath, std::fstream::out);
    }
    if (ret != 0) {
        outfile << "UnitTest " << testName << " FAILED !!!" << std::endl;
    } else {
        outfile << "UnitTest " << testName << " PASSED :)" << std::endl;
    }
    return ret;
}

#endif  // UT_GENERALUTILS_HPP_  // NOLINT
