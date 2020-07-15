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
 * Written by Bohan Shi <b34shi@edu.uwaterloo.ca>, July 2020
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <droneoa_ros/OAC/OAAlgRRT.hpp>
#include <droneoa_ros/OAC/OAC.hpp>
#include <droneoa_ros/HWI/Utils/CNCUtils.hpp>

namespace OAC {

OAAlgRRT::OAAlgRRT(CNC::CNCInterface *cnc) : BaseAlg(cnc), mCurrentSolutionRevision(0) {
    init();
}

OAAlgRRT::~OAAlgRRT() {
    CMDQueue_.clear();
}

void OAAlgRRT::init() {
}

bool OAAlgRRT::collect() {
    // Collect data directly from interface as required
    if (!mpCNC) {
        ROS_ERROR("[OAAlgFGM] Missing CNC pointer !!!");
        return false;
    }
    /***************************************************
     * Set Start Position
     ***************************************************/
    //! @note -Y axis is facing North (UE4 Verified)
    //! @note -X axis is facing East (UE4 Verified)
    if (OAC_USE_SETPOINT_ENU) {
        setStartPosENU();
    } else {
        setStartPosGPS();
    }
    /***************************************************
     * Set Goal Position
     ***************************************************/
    //! @note -Y axis is facing North (UE4 Verified)
    //! @note -X axis is facing East (UE4 Verified)
    if (OAC_USE_SETPOINT_ENU) {
        setGoalPosENU();
    } else {
        setGoalPosGPS();
    }
    return true;
}

bool OAAlgRRT::plan() {
    CMDQueue_.clear();
    DATAQueue_.clear();
    // Populate Command Queue
    if (OAC_USE_SETPOINT_ENU) {
        populateCMD_ENU();
    } else {
        populateCMD_GPS();
    }
    // Populate Data Queue
    populateDATA();
    return true;
}

void OAAlgRRT::setStartPosGPS() {
    GPSPoint currentGPS = mpCNC->getCurrentGPSPoint();
    LocalPoint startPos(-mpCNC->getLocalPosition().mX,
            -mpCNC->getLocalPosition().mY,
            mpCNC->getLocalPosition().mZ);
    if (mpCNC->isHomeGPSSet())  {
        //! @note GPS Location appear to be more accurate in simulator or with tracking camera
        //!       So we use GPS home location is available
        std::pair<float, float> fromHomeXY =
                CNC::CNCUtility::getNorthEastFromPoints(mpCNC->getHomeGPSPoint(), currentGPS);
        startPos.mX = -fromHomeXY.second;
        startPos.mY = -fromHomeXY.first;
    }
    if (LocalPoint::getDistBetweenPos3D(startPos, previousStart) > POS3D_COMPARE_DIFF_MAX) {
        previousStart = startPos;
        mPlanner.setStartPos(startPos);
    }
}

void OAAlgRRT::setStartPosENU() {
    LocalPoint startPos(-mpCNC->getLocalPosition().mX,
            -mpCNC->getLocalPosition().mY,
            mpCNC->getLocalPosition().mZ);
    if (LocalPoint::getDistBetweenPos3D(startPos, previousStart) > POS3D_COMPARE_DIFF_MAX) {
        previousStart = startPos;
        mPlanner.setStartPos(startPos);
    }
}

void OAAlgRRT::setGoalPosGPS() {
    GPSPoint currentGPS = mpCNC->getCurrentGPSPoint();
    GPSPoint goalGPS((mpCNC->getLocalMissionQueue()).front().mX,
            (mpCNC->getLocalMissionQueue()).front().mY,
            (mpCNC->getLocalMissionQueue()).front().mZ);
    std::pair<float, float> targetXY = CNC::CNCUtility::getNorthEastFromPoints(currentGPS, goalGPS);
    if (mpCNC->isHomeGPSSet()) {
        std::pair<float, float> fromHomeXY =
                CNC::CNCUtility::getNorthEastFromPoints(mpCNC->getHomeGPSPoint(), currentGPS);
        targetXY.second += fromHomeXY.second;
        targetXY.first += fromHomeXY.first;
    } else {
        LocalPoint posToHome = mpCNC->getLocalPosition();
        targetXY.first += posToHome.mY;
        targetXY.second += posToHome.mX;
    }
    // Target position need to be different from start position
    LocalPoint targetPos;
    // Check if GPSPoint data is valid
    if (goalGPS.mZ >= RRT_MIN_GOAL_HEIGHT &&
            std::sqrt(targetXY.first*targetXY.first + targetXY.second*targetXY.second) < RRT_MAX_PLANNING_DISTANCE) {
        targetPos.mX = -targetXY.second;
        targetPos.mY = -targetXY.first;
        targetPos.mZ = goalGPS.mZ;
    }
    if (LocalPoint::getDistBetweenPos3D(targetPos, previousGoal) > POS3D_COMPARE_DIFF_MAX) {
        previousGoal = targetPos;
        mPlanner.setTargetPos(targetPos);
    }
}

void OAAlgRRT::setGoalPosENU() {
    LocalPoint goalPos(-(mpCNC->getLocalMissionQueue()).front().mX,
            -(mpCNC->getLocalMissionQueue()).front().mY,
            (mpCNC->getLocalMissionQueue()).front().mZ);
    if (LocalPoint::getDistBetweenPos3D(goalPos, previousGoal) > POS3D_COMPARE_DIFF_MAX) {
        previousGoal = goalPos;
        mPlanner.setTargetPos(goalPos);
    }
}

void OAAlgRRT::populateDATA() {
    // Populate Data Queue
    bool solutionExist = mPlanner.isSolutionExist();
    DATAQueue_.push_back(Command::DataLine(Command::DATA_QUEUE_TYPES::DATA_ALG_NAME, SYS_Algs_STR[SYS_Algs::ALG_RRT]));
    DATAQueue_.push_back(Command::DataLine(Command::DATA_QUEUE_TYPES::DATA_CONFIDENCE,
            solutionExist ? std::to_string(1.0f) : std::to_string(0.0f)));
    DATAQueue_.push_back(Command::DataLine(Command::DATA_QUEUE_TYPES::DATA_SOLUTION_COST,
            solutionExist ?
            std::to_string(mPlanner.getPathCost()) :
            std::to_string(std::numeric_limits<float>::infinity())));
}

void OAAlgRRT::populateCMD_GPS() {
    // Populate Command Queue
    bool solutionExist = mPlanner.isSolutionExist();
    if (solutionExist) {
        std::vector<LocalPoint> wpLocal;
        int32_t newSolVersion = mPlanner.getPathAndRevision(&wpLocal);
        if (mCurrentSolutionRevision < newSolVersion) {
            // Reduce max speed
            CMDQueue_.push_back(Command::CommandLine(Command::CMD_QUEUE_TYPES::CMD_SET_MAX_VELOCITY,
                    std::to_string(RRT_MAX_SPEED)));
            //! @note -x is east
            //! @note -y is north
            bool first = true;
            for (auto wp : wpLocal) {
                if (first) {
                    first = false;
                    continue;
                }
                GPSPoint homeGPS = mpCNC->getCurrentGPSPoint();
                if (mpCNC->isHomeGPSSet()) {
                    homeGPS = mpCNC->getHomeGPSPoint();
                } else {
                    wp.mX += mpCNC->getLocalPosition().mX;
                    wp.mY += mpCNC->getLocalPosition().mY;
                }
                GPSPoint wpGPS = CNC::CNCUtility::getLocationMeter(homeGPS, -(wp.mY), -(wp.mX));
                CMDQueue_.push_back(Command::CommandLine(Command::CMD_QUEUE_TYPES::CMD_GOTO_GLOBAL_GPS,
                        std::to_string(wpGPS.mX) + " " + std::to_string(wpGPS.mY)
                        + " " + std::to_string(wp.mZ)));
                CMDQueue_.push_back(Command::CommandLine(Command::CMD_QUEUE_TYPES::CMD_UNTIL, "arrwp"));
            }
            mCurrentSolutionRevision = newSolVersion;
        }
    }
}

void OAAlgRRT::populateCMD_ENU() {
    // Populate Command Queue
    bool solutionExist = mPlanner.isSolutionExist();
    if (solutionExist) {
        std::vector<LocalPoint> wpLocal;
        int32_t newSolVersion = mPlanner.getPathAndRevision(&wpLocal);
        if (mCurrentSolutionRevision < newSolVersion) {
            // Reduce max speed
            CMDQueue_.push_back(Command::CommandLine(Command::CMD_QUEUE_TYPES::CMD_SET_MAX_VELOCITY,
                    std::to_string(RRT_MAX_SPEED)));
            //! @note -x is east
            //! @note -y is north
            bool first = true;
            for (auto wp : wpLocal) {
                if (first) {
                    first = false;
                    continue;
                }
                CMDQueue_.push_back(Command::CommandLine(Command::CMD_QUEUE_TYPES::CMD_GOTO_GLOBAL_ENU,
                        std::to_string(-wp.mX) + " " + std::to_string(-wp.mY)
                        + " " + std::to_string(wp.mZ)));
                CMDQueue_.push_back(Command::CommandLine(Command::CMD_QUEUE_TYPES::CMD_UNTIL, "arrwp"));
            }
            mCurrentSolutionRevision = newSolVersion;
        }
    }
}

}  // namespace OAC
