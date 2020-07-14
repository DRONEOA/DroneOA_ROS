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
    GPSPoint currentGPS = mpCNC->getCurrentGPSPoint();
    /***************************************************
     * Set Start Position
     ***************************************************/
    //! @note -Y axis is facing North (UE4 Verified)
    //! @note -X axis is facing East (UE4 Verified)
    Position3D startPos(-mpCNC->getLocalPosition().pose.position.x,
            -mpCNC->getLocalPosition().pose.position.y,
            mpCNC->getLocalPosition().pose.position.z);
    if (mpCNC->isHomeGPSSet())  {
        //! @note GPS Location appear to be more accurate in simulator or with tracking camera
        //!       So we use GPS home location is available
        std::pair<float, float> fromHomeXY =
                CNC::CNCUtility::getNorthEastFromPoints(mpCNC->getHomeGPSPoint(), currentGPS);
        startPos.mX = -fromHomeXY.second;
        startPos.mY = -fromHomeXY.first;
    }
    if (OMPLPlanner::getDistBetweenPos3D(startPos, previousStart) > DIST_COMPARE_DIFF_MAX) {
        previousStart = startPos;
        mPlanner.setStartPos(startPos);
    }
    /***************************************************
     * Set Goal Position
     ***************************************************/
    //! @note -Y axis is facing North (UE4 Verified)
    //! @note -X axis is facing East (UE4 Verified)
    GPSPoint goalGPS = (mpCNC->getLocalMissionQueue()).front();
    std::pair<float, float> targetXY = CNC::CNCUtility::getNorthEastFromPoints(currentGPS, goalGPS);
    if (mpCNC->isHomeGPSSet()) {
        std::pair<float, float> fromHomeXY =
                CNC::CNCUtility::getNorthEastFromPoints(mpCNC->getHomeGPSPoint(), currentGPS);
        targetXY.second += fromHomeXY.second;
        targetXY.first += fromHomeXY.first;
    } else {
        geometry_msgs::PoseStamped posToHome = mpCNC->getLocalPosition();
        targetXY.first += posToHome.pose.position.y;
        targetXY.second += posToHome.pose.position.x;
    }
    // Target position need to be different from start position
    Position3D targetPos;
    // Check if GPSPoint data is valid
    if (goalGPS.altitude_ >= MIN_GOAL_HEIGHT &&
            std::sqrt(targetXY.first*targetXY.first + targetXY.second*targetXY.second) < MAX_PLANNING_DISTANCE) {
        targetPos.mX = -targetXY.second;
        targetPos.mY = -targetXY.first;
        targetPos.mZ = goalGPS.altitude_;
    }
    if (OMPLPlanner::getDistBetweenPos3D(targetPos, previousGoal) > DIST_COMPARE_DIFF_MAX) {
        previousGoal = targetPos;
        mPlanner.setTargetPos(targetPos);
    }
    return true;
}

bool OAAlgRRT::plan() {
    CMDQueue_.clear();
    DATAQueue_.clear();
    // Populate Command Queue
    if (CMD_MODE == 0) {
        populateCMD_MQueue();
    } else if (CMD_MODE == 1) {
        populateCMD_AUTO();
    } else {
        populateCMD_Setpoint();
    }
    // Populate Data Queue
    populateDATA();
    return true;
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

void OAAlgRRT::populateCMD_AUTO() {
    // Populate Command Queue
    bool solutionExist = mPlanner.isSolutionExist();
    if (solutionExist) {
        std::vector<Position3D> wpLocal;
        int32_t newSolVersion = mPlanner.getPathAndRevision(&wpLocal);
        std::string dataStr = "";
        if (mCurrentSolutionRevision < newSolVersion) {
            // Reduce max speed
            CMDQueue_.push_back(Command::CommandLine(Command::CMD_QUEUE_TYPES::CMD_SET_MAX_VELOCITY,
                    std::to_string(MAX_SPEED)));
            CMDQueue_.push_back(Command::CommandLine(Command::CMD_QUEUE_TYPES::CMD_CHMOD,
                    FLT_MODE_AUTO));
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
                    mpCNC->getLocalPosition();
                    wp.mX += mpCNC->getLocalPosition().pose.position.x;
                    wp.mY += mpCNC->getLocalPosition().pose.position.y;
                }
                GPSPoint wpGPS = CNC::CNCUtility::getLocationMeter(homeGPS, -(wp.mY), -(wp.mX));
                dataStr += std::to_string(wpGPS.latitude_) + "," + std::to_string(wpGPS.longitude_)
                        + "," + std::to_string(wp.mZ) + " ";
            }
            CMDQueue_.push_back(Command::CommandLine(Command::CMD_QUEUE_TYPES::CMD_PUSH_MISSION_QUEUE, dataStr));
            mCurrentSolutionRevision = newSolVersion;
        }
    }
}

void OAAlgRRT::populateCMD_MQueue() {
    // Populate Command Queue
    bool solutionExist = mPlanner.isSolutionExist();
    if (solutionExist) {
        std::vector<Position3D> wpLocal;
        int32_t newSolVersion = mPlanner.getPathAndRevision(&wpLocal);
        if (mCurrentSolutionRevision < newSolVersion) {
            // Reduce max speed
            CMDQueue_.push_back(Command::CommandLine(Command::CMD_QUEUE_TYPES::CMD_SET_MAX_VELOCITY,
                    std::to_string(MAX_SPEED)));
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
                    mpCNC->getLocalPosition();
                    wp.mX += mpCNC->getLocalPosition().pose.position.x;
                    wp.mY += mpCNC->getLocalPosition().pose.position.y;
                }
                GPSPoint wpGPS = CNC::CNCUtility::getLocationMeter(homeGPS, -(wp.mY), -(wp.mX));
                CMDQueue_.push_back(Command::CommandLine(Command::CMD_QUEUE_TYPES::CMD_GOTO_GLOBAL,
                        std::to_string(wpGPS.latitude_) + " " + std::to_string(wpGPS.longitude_)
                        + " " + std::to_string(wp.mZ)));
                CMDQueue_.push_back(Command::CommandLine(Command::CMD_QUEUE_TYPES::CMD_UNTIL, "arrwp"));
            }
            mCurrentSolutionRevision = newSolVersion;
        }
    }
}

void OAAlgRRT::populateCMD_Setpoint() {
    //! @todo Blocked until setpoint function is implemented
    ROS_ERROR("Set Point Function Is WIP !!!");
}

}  // namespace OAC
