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
    // If current location is different from previous one. Update start. (face -y axis)
    Position3D startPos(mpCNC->getLocalPosition().pose.position.x,
            -mpCNC->getLocalPosition().pose.position.y,
            mpCNC->getLocalPosition().pose.position.z);
    if (OMPLPlanner::getDistBetweenPos3D(startPos, previousStart) > DIST_COMPARE_DIFF_MAX) {
        previousStart = startPos;
        mPlanner.setStartPos(startPos);
    }
    //! @note -Y axis is facing North (UE4 Verified)
    //! @note -X axis is facing East (UE4 Verified)
    GPSPoint goalGPS = (mpCNC->getLocalMissionQueue()).front();
    GPSPoint currentGPS = mpCNC->getCurrentGPSPoint();
    if (goalGPS == currentGPS) {
        //! @todo arrived
        //! @todo should this be checked in OAC
        ROS_WARN("ARRIVED!!!");
    } else {
        // Target position need to be different from start position
        Position3D targetPos;
        // Check if GPSPoint data is valid
        if (CNC::CNCUtility::getDistanceMeter(currentGPS, goalGPS) < MAX_PLANNING_DISTANCE) {
            std::pair<float, float> targetXY = CNC::CNCUtility::getNorthEastFromPoints(currentGPS, goalGPS);
            targetPos.mX = -targetXY.second;
            targetPos.mY = -targetXY.first;
            targetPos.mZ = goalGPS.altitude_;
        }
        if (OMPLPlanner::getDistBetweenPos3D(targetPos, previousGoal) > DIST_COMPARE_DIFF_MAX) {
            previousGoal = targetPos;
            mPlanner.setTargetPos(targetPos);
        }
    }
    return true;
}

bool OAAlgRRT::plan() {
    //! @todo get data from planner
    CMDQueue_.clear();
    DATAQueue_.clear();
    DATAQueue_.push_back(Command::DataLine(Command::DATA_QUEUE_TYPES::DATA_ALG_NAME, SYS_Algs_STR[SYS_Algs::ALG_RRT]));
    DATAQueue_.push_back(Command::DataLine(Command::DATA_QUEUE_TYPES::DATA_CONFIDENCE, std::to_string(0.0f)));
    //! @todo remain false until implemented
    return true;
}

}  // namespace OAC
