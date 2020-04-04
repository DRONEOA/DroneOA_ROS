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
#include <droneoa_ros/OAC/BaseAlg.hpp>
#include <droneoa_ros/OAC/CAAlgLidar.hpp>
#include <droneoa_ros/OAC/CAAlgDepthCam.hpp>
#include <droneoa_ros/OAC/OAAlgFGM.hpp>
#include <droneoa_ros/OAC/CMDParser.hpp>

// #define DEBUG_OAC

/**
 * @brief Obstacle Avoidance Controller System states
 */
enum SYS_State {
    SYS_IDLE,
    SYS_SAFE,  // optional
    SYS_EVALUATED,
    SYS_PLANNED,
    SYS_EXEC,
    SYS_ABORT,
};

/**
 * @brief Obstacle Avoidance Controller System states' name
 */
static const char* SYS_STATE_NAME[] {
    "SYS_IDLE",
    "SYS_SAFE",
    "SYS_EVALUATED",
    "SYS_PLANNED",
    "SYS_EXEC",
    "SYS_ABORT",
};

/**
 * @brief Supported Algorithms
 */
enum SYS_Algs {
    ALG_BUG,  // S2 If time allows
    ALG_VFF,  // S2 If time allows
    ALG_FGM,  // S2
    ALG_VISION,  // S2
    ALG_COLLISION_LIDAR,  // S1
    ALG_COLLISION_DEPTH,  // S1
    ALG_AI,  // S3
    ALG_SLAM  // Wishlist :)
};

/**
 * @brief Supported Decision Strategies (Between algorithms if more than 1)
 * Planned Future Feature
 */
enum SYS_SelectedDetermineFun {
    DET_STAGE1,
    DET_STAGE2,
    DET_STAGE3,
    DET_INVALID
};

class OAController {
    CMDParser *parserExecuter_ = nullptr;
    CMDRunner *theRunner_ = nullptr;

 public:
    OAController(CNCInterface *cnc, LidarInterface *lidar, RSCInterface *rsc, CMDRunner *runner, ros::Rate r);
    virtual ~OAController();

    /**
     * @brief Init the OA Controller
     * Seperated from constructor due to the planned restart feature
     * @param cnc pointer to command and control interface
     * @param lidar pointer to lidar interface
     * @param rsc pointer to realsense camera interface
     */
    void init(CNCInterface *cnc, LidarInterface *lidar, RSCInterface *rsc);
    /**
     * @brief Tick event, which stepping the state machine
     */
    void tick();
    /**
     * @brief Control Switch to pause / resume the OAController
     * @param isOn new switch state (true = on)
     */
    void masterSwitch(bool isOn);  // Pause / Resume OA Controller
    /**
     * @brief Get current state of the OAController
     * @return a string of the status
     */
    std::string getStatus();

 private:
    bool evaluate();  // Collect and Evaluate data from sensors
    bool plan();  // Plan next waypoint use selected algorithm(s)
    bool execute();  // Execute next waypoint use CNC Interface
    bool abort();  // About execution and "Brake"
    bool popAlgorithmFromSelected(SYS_Algs algName);
    std::vector<SYS_Algs> selectAlgorithm();  // Determine which algorithm to use
    SYS_SelectedDetermineFun selectDetermineFunction();  // Determine which determine function to use

    // Execution Determination
    void determineFunStage1();
    void determineFunStage2();
    void determineFunStage3();

    // Thread
    bool isOn_ = false;
    void masterThread();
    boost::thread *thread_oac_master_;

    ros::Rate r_ = ros::Rate(OAC_REFRESH_FREQ);
    CNCInterface *cnc_;
    LidarInterface *lidar_;
    RSCInterface *rsc_;
    std::map<SYS_Algs, BaseAlg*> algorithmInstances_;
    bool isTerminated = false;

    // Current State
    SYS_State currState_ = SYS_State::SYS_IDLE;
    std::vector<SYS_Algs> selectedAlgorithm_;
    SYS_SelectedDetermineFun selectedDetermineFun_;
    std::map<SYS_Algs, Command::CommandQueue> algCMDmap_;
    std::map<SYS_Algs, Command::DataQueue> algDATAmap_;
};

#endif  // INCLUDE_DRONEOA_ROS_OACONTROLLER_HPP_  // NOLINT
