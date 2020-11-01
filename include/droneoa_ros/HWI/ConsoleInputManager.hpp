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
 */

#ifndef HWI_CONSOLEINPUTMANAGER_HPP_  // NOLINT
#define HWI_CONSOLEINPUTMANAGER_HPP_  // NOLINT

#include <std_msgs/String.h>

#include <vector>
#include <string>
#include <utility>

#include <droneoa_ros/HWI/interface/CNCInterface.hpp>
#include <droneoa_ros/HWI/base/LidarGeneric.hpp>
#include <droneoa_ros/HWI/RSC.hpp>
#include <droneoa_ros/OAC/OAC.hpp>

namespace IO {

typedef std::pair<std::string, std::vector<std::string>> CMDPair;
class ConsoleInputManager {
 public:
    explicit ConsoleInputManager(bool* masterSwitch);
    virtual ~ConsoleInputManager();

    bool init(CNC::CNCInterface* cnc, Depth::RSC *rsc, OAC::OAController *oac, Lidar::LidarGeneric *lidar,
         OAC::CMDRunner *runner);

    /**
     * @brief Pass the input console command, and execute if valid
     * @param cmd the input command (only support single command in this version)
     * @return whether the command is valid and request is successfuly sent
     */
    bool parseAndExecuteConsole(std::string cmd);

    void command_callback(const std_msgs::String::ConstPtr& msg);

 private:
    CMDPair currentCommand_;
    bool* masterSwitch_;
    ros::NodeHandle mNodeHandle;

    // Parser
    OAC::CMDParser* mpParser;
    Command::CommandQueue mGeneratedCMDQueue;

    // Pointer to controller
    CNC::CNCInterface *mpCNC;
    Depth::RSC *mpRSC;
    OAC::OAController *oac_;
    Lidar::LidarGeneric *mpLidar;

    // Listen to command from topic
    boost::thread* thread_watch_command_ = nullptr;
    void watchCommandThread();

    // Publisher
    ros::Publisher mUnKnownCmdPub;
    std::string mCurrentProcessingCMD;
    void publishUnhandledCMD();

    // Statue
    bool mIsBuildingQueue;

    // Private Handlers
    bool splitModuleCommand(std::string cmd);
    bool buildCommandQueue();

    // Module Handlers
    bool buildCNCCommands();
    bool buildOACCommands();
    bool buildRSCCommands();
    bool buildLIDARCommands();
    bool buildQuickCommands();
    bool buildQueueCommands();

    // Helper
    void printFormatHelper();
    void printModuleHelper();
    void printCNCHelper();
    void printRSCHelper();
    void printOACHelper();
    void printLIDARHelper();
    void printQuickHelper();
    void printQueueHelper();
};

}  // namespace IO

#endif  // HWI_CONSOLEINPUTMANAGER_HPP_  // NOLINT
