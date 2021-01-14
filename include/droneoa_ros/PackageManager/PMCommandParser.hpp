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

#include <ros/ros.h>
#include <std_msgs/String.h>

#include <string>
#include <vector>
#include <map>

#include <boost/thread.hpp>
#include <droneoa_ros/Utils/JsonUtils.hpp>

namespace PM {

/**
 * @brief Abstract data structure to represent a package
 */
struct PackageRecord {
    std::string name;
    std::string url;
    std::string branch;
    bool startWithMainNode;
    PackageRecord();
    explicit PackageRecord(std::string name);
    std::string toString(bool forJSON = false);
    static PackageRecord fromString(std::string data);
};

class CommandParser {
    ros::NodeHandle mNodeHandle;
    std::string DRONEOA_PATH;
    std::map<std::string, PackageRecord> mPackageList;
    bool isSkipVerify;
    void install(std::vector<std::string> tokens);
    void update(std::vector<std::string> tokens);
    void uninstall(std::vector<std::string> tokens);
    void list(std::vector<std::string> tokens);
    bool rebuild();
    bool launch(std::vector<std::string> tokens);
    void shutdown(std::vector<std::string> tokens);
    void safeShutdownMainNode();
    void printHelp();
    void printModuleNameHelp();
    int runPMScripts(std::string scriptName, std::vector<std::string> tokens);
    // File Operation
    JSON::JsonUtils mJSONUtil;
    bool writeListToFile();
    bool readListFromFile();
    // Handle new input from Console Service
    boost::thread* thread_watch_command_ = nullptr;
    void watchCommandThread();

 public:
    CommandParser();
    ~CommandParser();
    /**
     * @brief Process input command, separate tokens into a vector.
     * The next step is parseInput(std::vector<std::string> tokens).
     * @param input console input string
     * @return true if command is valid and successfully executed
     * @return false if command is invalid or error during execution
     */
    bool parseInput(std::string input);
    bool parseInput(std::vector<std::string> tokens);
    /**
     * @brief Handle new input flag, request the message with module name PM
     * @param msg msg from topic NEW_INPUT_FLAG_TOPIC_NAME
     */
    void command_callback(const std_msgs::String::ConstPtr& msg);
};

}  // namespace PM

#endif  // NOLINT
