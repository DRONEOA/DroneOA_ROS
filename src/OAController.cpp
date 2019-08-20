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

OAController::OAController() {
    // @todo create algorithm instances
}

OAController::~OAController() {}

// Init OA Controller
// - Input: CNC Interface Instance
void OAController::init(CNCInterface cnc) {
    cnc_ = cnc;
    currState_ = sysState::SYS_IDLE;
    // @todo init algorithm instances
}

// Tick Event, Automatically switch states and run handlers
void OAController::tick() {
    if (isTerminated) {
        return;
    }
    switch (currState_) {
        case sysState::SYS_IDLE:
            ROS_DEBUG("[TICK] SYS_IDLE");
            evaluate();
            break;
        case sysState::SYS_EVALUATED:
            ROS_DEBUG("[TICK] SYS_EVALUATED");
            plan();
            break;
        case sysState::SYS_PLANNED:
            ROS_DEBUG("[TICK] SYS_PLANNED");
            execute();
            break;
        case sysState::SYS_ABORT:
            ROS_DEBUG("[TICK] SYS_ABORT");
            abort();
            break;
        case sysState::SYS_EXEC:
            ROS_DEBUG("[TICK] SYS_EXEC");
            currState_ = sysState::SYS_IDLE;
            break;
        case sysState::SYS_SAFE:
            ROS_DEBUG("[TICK] SYS_SAFE");
            currState_ = sysState::SYS_IDLE;
            break;
        default:
            // @todo error state
            ROS_ERROR("[TICK] Unknown state");
            break;
    }
}

bool OAController::evaluate() {
    // Entry state: SYS_IDLE
    // @todo run collector for each selected algorithm
    for (sysSelectedAlgs tmp : selectedAlgorithm_) {
        algorithmInstances_[tmp].collect();
        // @todo handle false return
    }

    selectedAlgorithm_ = selectAlgorithm();

    if (selectedAlgorithm_.size() == 0) {
        currState_ = sysState::SYS_IDLE;
        return false;
    }

    currState_ = sysState::SYS_EVALUATED;
    return true;
}

bool OAController::plan() {
    // Entry state: SYS_EVALUATED
    // @todo run planner for each selected algorithm
    for (sysSelectedAlgs tmp : selectedAlgorithm_) {
        algorithmInstances_[tmp].plan();
        // @todo handle false return
        // @todo determine if is safe to keep current path
    }

    currState_ = sysState::SYS_PLANNED;
    return true;
}

bool OAController::execute() {
    // Entry state: SYS_PLANNED
    selectedDetermineFun_ = selectDetermineFunction();
    switch (selectedDetermineFun_) {
        case sysSelectedDetermineFun::DET_STAGE1:
            // @todo handler
            break;
        case sysSelectedDetermineFun::DET_STAGE2:
            // @todo handler
            break;
        case sysSelectedDetermineFun::DET_STAGE3:
            // @todo handler
            break;
        default:
            break;
    }

    currState_ = sysState::SYS_EXEC;
    return true;
}

bool OAController::abort() {
    if (cnc_.getMode() == FLT_MODE_BRAKE) {
        // @todo determine whether to exit abort state and resume OA loop
        // currState_ = sysState::SYS_IDLE;
        isTerminated = true;
        return true;
    }
    if (cnc_.setMode(FLT_MODE_BRAKE)) {
        ROS_INFO("[ABORT] set BRAKE mode");
        isTerminated = true;
        return true;
    } else {
        // @todo handle fail to abort
        ROS_ERROR("[ABORT] fail to set BRAKE mode");
        return false;
    }
}

std::vector<sysSelectedAlgs> OAController::selectAlgorithm() {
    // @todo
    std::vector<sysSelectedAlgs> result;
    // result.push_back(sysSelectedAlgs::ALG_AI_COLLISION);
    // result.push_back(sysSelectedAlgs::ALG_LIDAR_COLLISION);
    // result.push_back(sysSelectedAlgs::ALG_VISUAL_COLLISION);
    return result;
}

sysSelectedDetermineFun OAController::selectDetermineFunction() {
    // @todo
    return sysSelectedDetermineFun::DET_STAGE1;
}
