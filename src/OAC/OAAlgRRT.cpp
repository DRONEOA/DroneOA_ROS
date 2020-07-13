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

OAAlgRRT::OAAlgRRT(CNC::CNCInterface *cnc) : BaseAlg(cnc) {
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
    //! @todo get data from planner
    CMDQueue_.clear();
    DATAQueue_.clear();
    DATAQueue_.push_back(Command::DataLine(Command::DATA_QUEUE_TYPES::DATA_ALG_NAME, SYS_Algs_STR[SYS_Algs::ALG_RRT]));
    DATAQueue_.push_back(Command::DataLine(Command::DATA_QUEUE_TYPES::DATA_CONFIDENCE,
            mPlanner.isSolutionExist() ? std::to_string(1.0f) : std::to_string(0.0f)));
    DATAQueue_.push_back(Command::DataLine(Command::DATA_QUEUE_TYPES::DATA_SOLUTION_COST,
            mPlanner.isSolutionExist() ?
            std::to_string(mPlanner.getPathCost()) :
            std::to_string(std::numeric_limits<float>::infinity())));
    return true;
}

}  // namespace OAC
