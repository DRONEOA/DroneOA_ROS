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
 * Written by Bohan Shi <b34shi@edu.uwaterloo.ca>, August 2019
 */

#ifndef INCLUDE_DRONEOA_ROS_OACONTROLLER_HPP_  // NOLINT
#define INCLUDE_DRONEOA_ROS_OACONTROLLER_HPP_  // NOLINT

#include <string>
#include <vector>
#include <map>
#include <utility>

#include <droneoa_ros/CNCInterface.hpp>
#include <droneoa_ros/LidarInterface.hpp>
#include <droneoa_ros/RSCInterface.hpp>
#include <droneoa_ros/OAUtils/BaseAlg.hpp>
#include <droneoa_ros/OAUtils/CAAlgLidar.hpp>
#include <droneoa_ros/OAUtils/CAAlgDepthCam.hpp>

enum SYS_State {
    SYS_IDLE,
    SYS_SAFE,  // optional
    SYS_EVALUATED,
    SYS_PLANNED,
    SYS_EXEC,
    SYS_ABORT
};

enum SYS_Algs {
    ALG_BUG,
    ALG_VFF,
    ALG_COLLISION_LIDAR,
    ALG_COLLISION_DEPTH,
    ALG_COLLISION_AI
};

enum SYS_SelectedDetermineFun {
    DET_STAGE1,
    DET_STAGE2,
    DET_STAGE3,
    DET_INVALID
};

typedef std::vector<std::pair<CMD_QUEUE_TYPES, std::string>> CommandQueue;
typedef std::vector<std::pair<DATA_QUEUE_TYPES, std::string>> DataQueue;

#define DEBUG_OAC

class OAController {
 public:
    OAController(CNCInterface *cnc, LidarInterface *lidar, RSCInterface *rsc, ros::Rate r);
    virtual ~OAController();

    void init(CNCInterface *cnc, LidarInterface *lidar, RSCInterface *rsc);
    void tick();

    void masterSwitch(bool isOn);  // Pause / Resume OA Controller

 private:
    bool evaluate();  // Collect and Evaluate data from sensors
    bool plan();  // Plan next waypoint use selected algorithm(s)
    bool execute();  // Execute next waypoint use CNC Interface
    bool abort();  // About execution and "Brake"
    std::vector<SYS_Algs> selectAlgorithm();  // Determine which algorithm to use
    SYS_SelectedDetermineFun selectDetermineFunction();  // Determine which determine function to use

    // Thread
    bool isOn_ = false;
    void masterThread();
    boost::thread* thread_oac_master_;

    ros::Rate r_ = ros::Rate(1);;
    CNCInterface *cnc_;
    LidarInterface *lidar_;
    RSCInterface *rsc_;
    std::map<SYS_Algs, BaseAlg*> algorithmInstances_;
    bool isTerminated = false;

    // Current State
    SYS_State currState_ = SYS_State::SYS_IDLE;
    std::vector<SYS_Algs> selectedAlgorithm_;
    SYS_SelectedDetermineFun selectedDetermineFun_;
    std::map<SYS_Algs, CommandQueue> algCMDmap_;
    std::map<SYS_Algs, DataQueue> algDATAmap_;
};

#endif  // NOLINT
