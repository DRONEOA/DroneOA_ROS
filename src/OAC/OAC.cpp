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

#include <droneoa_ros/OAC/OAC.hpp>
#include <string>

namespace OAC {

OAController::OAController(CNC::CNCInterface *cnc, Lidar::LidarGeneric *lidar, Depth::RSC *rsc,
        CMDRunner *runner, ros::Rate r) {
    mpTheRunner = runner;
    mR = r;
    init(cnc, lidar, rsc);

    mpOACMasterThread = new boost::thread(boost::bind(&OAController::masterThread, this));
}

OAController::~OAController() {
    if (mpParserExecuter) delete mpParserExecuter;
    for (const auto& elem : mAlgorithmInstances) {
        if (elem.second) {
            delete elem.second;
        }
    }

    // Wait for master thread to end
    if (mpOACMasterThread) {
        mpOACMasterThread->join();
        delete mpOACMasterThread;
        ROS_WARN("[OAC] MASTER THREAD ENDED");
    }
    ROS_INFO("Destroy OAController");
}

// Init OA Controller (for restart)
// - Input: CNCInterface *, LidarInterface *, RSCInterface *
void OAController::init(CNC::CNCInterface *cnc, Lidar::LidarGeneric *lidar, Depth::RSC *rsc) {
    mpCNC = cnc;
    mpLidar = lidar;
    mpRSC = rsc;
    mCurrState = SYS_State::SYS_IDLE;
    // Init command parser and executor
    // Note: Runner is uniqueue across the system at the moment
    if (mpParserExecuter) {
        delete mpParserExecuter;
    }
    mpParserExecuter = new CMDParser(mpCNC, mpTheRunner);
    // Create algorithm instances
    for (auto tmp : mAlgorithmInstances) {
        if (tmp.second) delete tmp.second;
    }
    // Collision are alwayse enable as safety gurantee
    mAlgorithmInstances[SYS_Algs::ALG_COLLISION_LIDAR] = new CAAlgLidar(mpCNC, mpLidar);
    mAlgorithmInstances[SYS_Algs::ALG_COLLISION_DEPTH] = new CAAlgDepthCam(mpCNC, mpRSC);
    if (OAC_STAGE_SETTING == 2) {
        // Add FGM for stage 2, Obstacle Avoidance (Local Path Planning)
        mAlgorithmInstances[SYS_Algs::ALG_FGM] = new OAAlgFGM(mpCNC, mpLidar);
    } else if (OAC_STAGE_SETTING == 3) {
        // Add RRT for stage 3, Obstacle Avoidance (Global Path Planning)
        mAlgorithmInstances[SYS_Algs::ALG_RRT] = new OAAlgRRT(mpCNC);
    }
    //! @todo create new alg instance here
    ROS_INFO("[OAC] init");
}

std::string OAController::getStatus() {
    if (!mIsOn) {
        return "PAUSED";
    }
    return SYS_STATE_NAME[mCurrState];
}

// Switch on/off the tick event
void OAController::masterSwitch(bool isOn) {
    mIsOn = isOn;
    if (mIsOn) {
        ROS_WARN("[OAC] MASTER RESUMED");
        return;
    }
    ROS_WARN("[OAC] MASTER PAUSED");
}

// Tick Event, Automatically switch states and run handlers
void OAController::tick() {
    if (mIsTerminated) {
        ROS_WARN("[OAC] Terminated");
        return;
    }

    switch (mCurrState) {
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
            mCurrState = SYS_State::SYS_IDLE;
            break;
        case SYS_State::SYS_SAFE:
            mCurrState = SYS_State::SYS_IDLE;
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
        if (mIsOn) {
            tick();
        }
        ros::spinOnce();
        mR.sleep();
    }
}

bool OAController::evaluate() {
    // Entry state: SYS_IDLE
#ifdef DEBUG_OAC
    ROS_INFO("========== CYCLE ==========");
#endif
    // Populate Selected Algorithm List
    mSelectedAlgorithm = selectAlgorithm();
    // Call each algorithm instances to collect data and evaluate
    for (SYS_Algs tmp : mSelectedAlgorithm) {
    #ifdef DEBUG_OAC
        ROS_INFO("[OAC] Evaluate %s", SYS_Algs_STR[tmp]);
    #endif
        if (!mAlgorithmInstances[tmp]->collect()) {
            // Remove from selected if collect with error
            popAlgorithmFromSelected(tmp);
        }
    }
    // No valid algorithms left is an Error
    if (mSelectedAlgorithm.size() == 0) {
        mCurrState = SYS_State::SYS_IDLE;
        ROS_ERROR("NO VALID ALGORITHM ENABLED !!!");
        return false;
    }
    // Set next state if success
    mCurrState = SYS_State::SYS_EVALUATED;
    return true;
}

bool OAController::plan() {
    // Entry state: SYS_EVALUATED
    mAlgCMDmap.clear();
    mAlgDATAmap.clear();
    // Call each algorithm to plan for next action(s)
    for (SYS_Algs tmp : mSelectedAlgorithm) {
    #ifdef DEBUG_OAC
        ROS_INFO("[OAC] PLAN NODE: %s", SYS_Algs_STR[tmp]);
    #endif
        if (!mAlgorithmInstances[tmp]->plan()) {
            popAlgorithmFromSelected(tmp);
            continue;
        }
        mAlgCMDmap[tmp] = (mAlgorithmInstances[tmp])->getCommandQueue();
        mAlgDATAmap[tmp] = (mAlgorithmInstances[tmp])->getDataQueue();
    #ifdef DEBUG_OAC
        for (auto cmdline : mAlgCMDmap[tmp]) {
            ROS_INFO("            CMD: %d with %s", cmdline.first, cmdline.second.c_str());
        }
        for (auto dataline : mAlgDATAmap[tmp]) {
            ROS_INFO("            DATA: %d with %s", dataline.first, dataline.second.c_str());
        }
    #endif
    }
    // No valid algorithms left is an Error
    if (mSelectedAlgorithm.size() == 0) {
        mCurrState = SYS_State::SYS_IDLE;
        ROS_ERROR("NO VALID ALGORITHM ENABLED !!!");
        return false;
    }
    // Set next state if success
    mCurrState = SYS_State::SYS_PLANNED;
    return true;
}

bool OAController::execute() {
    // Entry state: SYS_PLANNED
    // Select the determine/voting function based on configuration
    selectDetermineFunction();
#ifdef DEBUG_OAC
    ROS_INFO("[OAC] Execute Det: %d", selectedDetermineFun_);
#endif
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
            mCurrState = SYS_State::SYS_ABORT;
            return false;
    }
    // Set next state if success
    mCurrState = SYS_State::SYS_EXEC;
    return true;
}

void OAController::determineFunStage1() {
    // Collision Avoidance only have  one action BRAKE
    double lidarConf, depthConf;
    try {
        mAlgDATAmap.at(SYS_Algs::ALG_COLLISION_LIDAR);
        for (auto dataline : mAlgDATAmap[SYS_Algs::ALG_COLLISION_LIDAR]) {
            if (dataline.first == Command::DATA_QUEUE_TYPES::DATA_CONFIDENCE) {
                lidarConf = std::stof(dataline.second);
            }
        }
    } catch(...) {
        lidarConf = 0.0;
    }
    try {
        mAlgDATAmap.at(SYS_Algs::ALG_COLLISION_DEPTH);
        for (auto dataline : mAlgDATAmap[SYS_Algs::ALG_COLLISION_DEPTH]) {
            if (dataline.first == Command::DATA_QUEUE_TYPES::DATA_CONFIDENCE) {
                depthConf = std::stof(dataline.second);
            }
        }
    } catch(...) {
        depthConf = 0.0;
    }
    if (depthConf >= 0.9 || lidarConf >= 0.9) {
        if (depthConf >= lidarConf) {
            mpParserExecuter->parseCMDQueue(mAlgCMDmap[SYS_Algs::ALG_COLLISION_DEPTH]);
        } else {
            mpParserExecuter->parseCMDQueue(mAlgCMDmap[SYS_Algs::ALG_COLLISION_LIDAR]);
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
    // Entry state: ANY
#ifdef DEBUG_OAC
    ROS_WARN("[OAC] ABORT");
#endif
    // Attempt to stop the aircraft
    if (mpCNC->getMode() == FLT_MODE_BRAKE) {
        // Re-init is required to re-enable the OAC
        mIsTerminated = true;
        return true;
    }
    if (mpCNC->setMode(FLT_MODE_BRAKE)) {
        ROS_INFO("[ABORT] set BRAKE mode");
        mIsTerminated = true;
        return true;
    }
    //! @todo handle fail to abort
    ROS_ERROR("[ABORT] fail to set BRAKE mode");
    return false;
}

bool OAController::popAlgorithmFromSelected(SYS_Algs algIndex) {
    auto it = std::find(mSelectedAlgorithm.begin(), mSelectedAlgorithm.end(), algIndex);
    if (it != mSelectedAlgorithm.end()) {
        mSelectedAlgorithm.erase(it);
        ROS_WARN("%s Is Removed From Selected Algorithms", SYS_Algs_STR[algIndex]);
        return true;
    }
    return false;
}

std::vector<SYS_Algs> OAController::selectAlgorithm() {
    //! @todo Automatically select algorthm according to environment, then we don't need config any more
    mSelectedAlgorithm.clear();
    if (OAC_STAGE_SETTING == 1) {
        if (ENABLE_LIDAR) mSelectedAlgorithm.push_back(SYS_Algs::ALG_COLLISION_LIDAR);
        if (ENABLE_RSC) mSelectedAlgorithm.push_back(SYS_Algs::ALG_COLLISION_DEPTH);
    } else if (OAC_STAGE_SETTING == 2) {
        if (ENABLE_LIDAR) mSelectedAlgorithm.push_back(SYS_Algs::ALG_FGM);
    } else if (OAC_STAGE_SETTING == 3) {
        if (ENABLE_OCTOMAP) mSelectedAlgorithm.push_back(SYS_Algs::ALG_RRT);
    } else {
        ROS_ERROR("Invalid OAC Stage Setting !!!");
    }
    return mSelectedAlgorithm;
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

}  // namespace OAC
