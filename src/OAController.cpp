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

OAController::OAController(CNCInterface *cnc, LidarInterface *lidar, RSCInterface *rsc,
        CMDRunner *runner, ros::Rate r) {
    theRunner_ = runner;
    r_ = r;
    init(cnc, lidar, rsc);

    thread_oac_master_ = new boost::thread(boost::bind(&OAController::masterThread, this));
}

OAController::~OAController() {
    if (parserExecuter_) delete parserExecuter_;
    for (const auto& elem : algorithmInstances_) {
        if (elem.second) {
            delete elem.second;
        }
    }
    if (thread_oac_master_) {
        delete thread_oac_master_;
        ROS_WARN("[OAC] MASTER THREAD ENDED");
    }
    ROS_INFO("Destroy OAController");
}

// Init OA Controller (for restart)
// - Input: CNCInterface *, LidarInterface *, RSCInterface *
void OAController::init(CNCInterface *cnc, LidarInterface *lidar, RSCInterface *rsc) {
    cnc_ = cnc;
    lidar_ = lidar;
    rsc_ = rsc;
    currState_ = SYS_State::SYS_IDLE;
    // init parser
    if (parserExecuter_) {
        delete parserExecuter_;
    }
    parserExecuter_ = new CMDParser(cnc_, theRunner_);
    // create algorithm instances
    for (auto tmp : algorithmInstances_) {
        if (tmp.second) delete tmp.second;
    }
    algorithmInstances_[SYS_Algs::ALG_COLLISION_LIDAR] = new CAAlgLidar(cnc_, lidar_);
    algorithmInstances_[SYS_Algs::ALG_COLLISION_DEPTH] = new CAAlgDepthCam(cnc_, rsc_);
    algorithmInstances_[SYS_Algs::ALG_FGM] = new OAAlgFGM(cnc_, lidar_);
    //! @todo create new alg instance here
    ROS_INFO("[OAC] init");
}

std::string OAController::getStatus() {
    if (!isOn_) {
        return "PAUSED";
    }
    return SYS_STATE_NAME[currState_];
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
            evaluate();
            break;
        case SYS_State::SYS_EVALUATED:
            plan();
            break;
        case SYS_State::SYS_PLANNED:
            execute();
            break;
        case SYS_State::SYS_ABORT:
            abort();
            break;
        case SYS_State::SYS_EXEC:
            currState_ = SYS_State::SYS_IDLE;
            break;
        case SYS_State::SYS_SAFE:
            currState_ = SYS_State::SYS_IDLE;
            break;
        default:
            // Should not be here
            ROS_ERROR("[TICK] Unknown state");
            break;
    }
}

void OAController::masterThread() {
    ROS_WARN("[OAC] MASTER THREAD START - PAUSED");

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
    selectedAlgorithm_ = selectAlgorithm();

    for (SYS_Algs tmp : selectedAlgorithm_) {
        if (!algorithmInstances_[tmp]->collect()) {
            // Remove from selected if collect with error
            popAlgorithmFromSelected(tmp);
        }
    }

    if (selectedAlgorithm_.size() == 0) {
        currState_ = SYS_State::SYS_IDLE;
        ROS_ERROR("NO VALID ALGORITHM ENABLED !!!");
        return false;
    }

    currState_ = SYS_State::SYS_EVALUATED;
    return true;
}

bool OAController::plan() {
    // Entry state: SYS_EVALUATED
    algCMDmap_.clear();
    algDATAmap_.clear();
    for (SYS_Algs tmp : selectedAlgorithm_) {
        if (!algorithmInstances_[tmp]->plan()) {
            popAlgorithmFromSelected(tmp);
            continue;
        }
        algCMDmap_[tmp] = (algorithmInstances_[tmp])->getCommandQueue();
        algDATAmap_[tmp] = (algorithmInstances_[tmp])->getDataQueue();
    #ifdef DEBUG_OAC
        ROS_INFO("[OAC] PLAN NODE: %d", tmp);
        for (auto cmdline : algCMDmap_[tmp]) {
            ROS_INFO("            CMD: %d with %s", cmdline.first, cmdline.second.c_str());
        }
        for (auto dataline : algDATAmap_[tmp]) {
            ROS_INFO("            DATA: %d with %s", dataline.first, dataline.second.c_str());
        }
    #endif
    }

    if (selectedAlgorithm_.size() == 0) {
        currState_ = SYS_State::SYS_IDLE;
        ROS_ERROR("NO VALID ALGORITHM ENABLED !!!");
        return false;
    }

    currState_ = SYS_State::SYS_PLANNED;
    return true;
}

bool OAController::execute() {
    // Entry state: SYS_PLANNED
    selectDetermineFunction();
    switch (selectedDetermineFun_) {
        case SYS_SelectedDetermineFun::DET_STAGE1:
            determineFunStage1();
            break;
        case SYS_SelectedDetermineFun::DET_STAGE2:
            determineFunStage2();
            break;
        case SYS_SelectedDetermineFun::DET_STAGE3:
            determineFunStage3();
            break;
        default:
            currState_ = SYS_State::SYS_ABORT;
            return false;
    }

    currState_ = SYS_State::SYS_EXEC;
    return true;
}

void OAController::determineFunStage1() {
    double lidarConf, depthConf;
    try {
        algDATAmap_.at(SYS_Algs::ALG_COLLISION_LIDAR);
        for (auto dataline : algDATAmap_[SYS_Algs::ALG_COLLISION_LIDAR]) {
            if (dataline.first == Command::DATA_QUEUE_TYPES::DATA_CONFIDENCE) {
                lidarConf = std::stof(dataline.second);
            }
        }
    } catch(...) {
        lidarConf = 0.0;
    }
    try {
        algDATAmap_.at(SYS_Algs::ALG_COLLISION_DEPTH);
        for (auto dataline : algDATAmap_[SYS_Algs::ALG_COLLISION_DEPTH]) {
            if (dataline.first == Command::DATA_QUEUE_TYPES::DATA_CONFIDENCE) {
                depthConf = std::stof(dataline.second);
            }
        }
    } catch(...) {
        depthConf = 0.0;
    }
    if (depthConf >= 0.9 || lidarConf >= 0.9) {
        if (depthConf >= lidarConf) {
            parserExecuter_->parseCMDQueue(algCMDmap_[SYS_Algs::ALG_COLLISION_DEPTH]);
        } else {
            parserExecuter_->parseCMDQueue(algCMDmap_[SYS_Algs::ALG_COLLISION_LIDAR]);
        }
    }
}

void OAController::determineFunStage2() {
    //! @todo
}

void OAController::determineFunStage3() {
    //! @todo
}

bool OAController::abort() {
    if (cnc_->getMode() == FLT_MODE_BRAKE) {
        // Re-init is required to re-enable the OAC
        isTerminated = true;
        return true;
    }
    if (cnc_->setMode(FLT_MODE_BRAKE)) {
        ROS_INFO("[ABORT] set BRAKE mode");
        isTerminated = true;
        return true;
    }
    //! @todo handle fail to abort
    ROS_ERROR("[ABORT] fail to set BRAKE mode");
    return false;
}

bool OAController::popAlgorithmFromSelected(SYS_Algs algName) {
    auto it = std::find(selectedAlgorithm_.begin(), selectedAlgorithm_.end(), algName);
    if (it != selectedAlgorithm_.end()) {
        selectedAlgorithm_.erase(it);
        return true;
    }
    return false;
}

std::vector<SYS_Algs> OAController::selectAlgorithm() {
    // @todo select algorthm according to environment
    selectedAlgorithm_.clear();
    if (OAC_STAGE_SETTING == 1) {
        if (ENABLE_LIDAR) selectedAlgorithm_.push_back(SYS_Algs::ALG_COLLISION_LIDAR);
        if (ENABLE_RSC) selectedAlgorithm_.push_back(SYS_Algs::ALG_COLLISION_DEPTH);
    } else if (OAC_STAGE_SETTING == 2) {
        if (ENABLE_LIDAR) selectedAlgorithm_.push_back(SYS_Algs::ALG_FGM);
        // if (ENABLE_RSC) selectedAlgorithm_.push_back(SYS_Algs::ALG_VISION);
    } else if (OAC_STAGE_SETTING == 3) {
        if (ENABLE_LIDAR) selectedAlgorithm_.push_back(SYS_Algs::ALG_FGM);
        // if (ENABLE_RSC) selectedAlgorithm_.push_back(SYS_Algs::ALG_VISION);
        // selectedAlgorithm_.push_back(SYS_Algs::ALG_AI);
        // selectedAlgorithm_.push_back(SYS_Algs::ALG_SLAM);
    } else {
        ROS_ERROR("Invalid OAC Stage Setting !!!");
    }
    return selectedAlgorithm_;
}

SYS_SelectedDetermineFun OAController::selectDetermineFunction() {
    if (OAC_STAGE_SETTING == 1) {
        selectedDetermineFun_ = SYS_SelectedDetermineFun::DET_STAGE1;
    } else if (OAC_STAGE_SETTING == 2) {
        selectedDetermineFun_ = SYS_SelectedDetermineFun::DET_STAGE2;
    } else if (OAC_STAGE_SETTING == 3) {
        selectedDetermineFun_ = SYS_SelectedDetermineFun::DET_STAGE3;
    } else {
        selectedDetermineFun_ = SYS_SelectedDetermineFun::DET_INVALID;
    }
    return selectedDetermineFun_;
}
