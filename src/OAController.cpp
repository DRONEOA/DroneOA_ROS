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

#include <droneoa_ros/OAController.hpp>
#include <string>

OAController::OAController(CNCInterface *cnc, LidarInterface *lidar, RSCInterface *rsc, ros::Rate r) {
    init(cnc, lidar, rsc);
    r_ = r;

    thread_oac_master_ = new boost::thread(boost::bind(&OAController::masterThread, this));
}

OAController::~OAController() {
    for (const auto& elem : algorithmInstances_) {
        if (elem.second) {
            delete elem.second;
        }
    }
    if (thread_oac_master_) {
        delete thread_oac_master_;
        ROS_WARN("[OAC] MASTER THREAD ENDED");
    }
}

// Init OA Controller (for restart)
// - Input: CNCInterface *, LidarInterface *, RSCInterface *
void OAController::init(CNCInterface *cnc, LidarInterface *lidar, RSCInterface *rsc) {
    cnc_ = cnc;
    lidar_ = lidar;
    rsc_ = rsc;
    currState_ = SYS_State::SYS_IDLE;
    // create algorithm instances
    algorithmInstances_[SYS_Algs::ALG_COLLISION] = new CollisionAvoidanceAlg(cnc_, lidar_, rsc_);
    ROS_INFO("[OACONTROLLER] init");
}

// Switch on/off the tick event
void OAController::masterSwitch(bool isOn) {
    isOn_ = isOn;
    if (isOn_) {
        ROS_WARN("[OAC] MASTER RESUMED");
        return;
    }
    ROS_WARN("[OAC] MASTER PAUSED");
}

// Tick Event, Automatically switch states and run handlers
void OAController::tick() {
    if (isTerminated) {
        ROS_WARN("[OAC] Terminated");
        return;
    }

    switch (currState_) {
        case SYS_State::SYS_IDLE:
        #ifdef DEBUG_OAC
            ROS_INFO("[TICK] SYS_IDLE");
        #endif
            evaluate();
            break;
        case SYS_State::SYS_EVALUATED:
        #ifdef DEBUG_OAC
            ROS_INFO("[TICK] SYS_EVALUATED");
        #endif
            plan();
            break;
        case SYS_State::SYS_PLANNED:
        #ifdef DEBUG_OAC
            ROS_INFO("[TICK] SYS_PLANNED");
        #endif
            execute();
            break;
        case SYS_State::SYS_ABORT:
        #ifdef DEBUG_OAC
            ROS_INFO("[TICK] SYS_ABORT");
        #endif
            abort();
            break;
        case SYS_State::SYS_EXEC:
        #ifdef DEBUG_OAC
            ROS_INFO("[TICK] SYS_EXEC");
        #endif
            currState_ = SYS_State::SYS_IDLE;
            break;
        case SYS_State::SYS_SAFE:
        #ifdef DEBUG_OAC
            ROS_INFO("[TICK] SYS_SAFE");
        #endif
            currState_ = SYS_State::SYS_IDLE;
            break;
        default:
            // @todo error state
            ROS_ERROR("[TICK] Unknown state");
            break;
    }
}

void OAController::masterThread() {
    ROS_WARN("[OAC] MASTER THREAD START");

    while (ros::ok()) {
        if (isOn_) {
            tick();
        }
        ros::spinOnce();
        r_.sleep();
    }
}

bool OAController::evaluate() {
    // Entry state: SYS_IDLE
    selectedAlgorithm_.clear();
    selectedAlgorithm_ = selectAlgorithm();

    for (SYS_Algs tmp : selectedAlgorithm_) {
        algorithmInstances_[tmp]->collect();
        // @todo handle false return - remove from selected
    }

    if (selectedAlgorithm_.size() == 0) {
        currState_ = SYS_State::SYS_IDLE;
        // @todo ERROR LOG
        return false;
    }

    currState_ = SYS_State::SYS_EVALUATED;
    return true;
}

bool OAController::plan() {
    // Entry state: SYS_EVALUATED
    // @todo run planner for each selected algorithm
    algCMDmap_.clear();
    for (SYS_Algs tmp : selectedAlgorithm_) {
        algorithmInstances_[tmp]->plan();
        algCMDmap_[tmp] = (algorithmInstances_[tmp])->getCommandQueue();
    #ifdef DEBUG_OAC
        ROS_INFO("[OAC] PLAN NODE: %d", tmp);
        for (auto cmdline : algCMDmap_[tmp]) {
            ROS_INFO("            CMD: %d with %s", cmdline.first, cmdline.second.c_str());
        }
    #endif
        // @todo handle false return - remove from selected
        // @todo determine if is safe to keep current path
    }

    if (selectedAlgorithm_.size() == 0) {
        currState_ = SYS_State::SYS_IDLE;
        // @todo ERROR LOG
        return false;
    }

    currState_ = SYS_State::SYS_PLANNED;
    return true;
}

bool OAController::execute() {
    // Entry state: SYS_PLANNED
    switch (selectedDetermineFun_) {
        case SYS_SelectedDetermineFun::DET_STAGE1:
            // @todo handler & parser
            break;
        case SYS_SelectedDetermineFun::DET_STAGE2:
            // @todo handler & parser
            break;
        case SYS_SelectedDetermineFun::DET_STAGE3:
            // @todo handler & parser
            break;
        default:
            break;
    }

    currState_ = SYS_State::SYS_EXEC;
    return true;
}

bool OAController::abort() {
    if (cnc_->getMode() == FLT_MODE_BRAKE) {
        // @todo determine whether to exit abort state and resume OA loop
        // currState_ = SYS_State::SYS_IDLE;
        isTerminated = true;
        return true;
    }
    if (cnc_->setMode(FLT_MODE_BRAKE)) {
        ROS_INFO("[ABORT] set BRAKE mode");
        isTerminated = true;
        return true;
    } else {
        // @todo handle fail to abort
        ROS_ERROR("[ABORT] fail to set BRAKE mode");
        return false;
    }
}

std::vector<SYS_Algs> OAController::selectAlgorithm() {
    // @todo select algorthm according to environment and config
    std::vector<SYS_Algs> result;
    result.push_back(SYS_Algs::ALG_COLLISION);
    return result;
}

SYS_SelectedDetermineFun OAController::selectDetermineFunction() {
    // @todo select determine function according to config
    return SYS_SelectedDetermineFun::DET_STAGE1;
}
