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

#ifndef PM_CMD_PARSER_  // NOLINT
#define PM_CMD_PARSER_  // NOLINT

#include <string>
#include <vector>
#include <map>

namespace PM {

struct PackageRecord {
    std::string name;
    std::string url;
    std::string branch;
    bool startWithMainNode;
    PackageRecord();
    explicit PackageRecord(std::string name);
    std::string toString();
};

class CommandParser {
    std::string DRONEOA_PATH;
    std::map<std::string, PackageRecord> mPackageList;
    void install(std::vector<std::string> tokens);
    void update(std::vector<std::string> tokens);
    void uninstall(std::vector<std::string> tokens);
    void list(std::vector<std::string> tokens);
    bool rebuild();
    void launch();
    void shutdown(bool killAllNodes);
    void printHelp();
    // File Operation
    bool writeListToFile();
    bool readListFromFile();
 public:
    CommandParser();
    ~CommandParser();
    bool parseInput(std::string input);
    bool parseInput(std::vector<std::string> tokens);
};

}  // namespace PM

#endif  // NOLINT
