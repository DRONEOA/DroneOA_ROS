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

#include <droneoa_ros/CNCInterface.hpp>
#include <droneoa_ros/OAUtils/BaseAlg.hpp>

enum sysState {
    SYS_IDLE,
    SYS_SAFE,  // optional
    SYS_EVALUATED,
    SYS_PLANNED,
    SYS_EXEC,
    SYS_ABORT
};

enum sysSelectedAlgs {
    ALG_BUG,
    ALG_VFF,
    ALG_VISUAL_COLLISION,
    ALG_AI_COLLISION,
    ALG_LIDAR_COLLISION
};

enum sysSelectedDetermineFun {
    DET_STAGE1,
    DET_STAGE2,
    DET_STAGE3
};

class OAController {
 public:
    OAController();
    virtual ~OAController();

    void init(CNCInterface cnc);
    void tick();

 private:
    bool evaluate();  // Collect and Evaluate data from sensors
    bool plan();  // Plan next waypoint use selected algorithm(s)
    bool execute();  // Execute next waypoint use CNC Interface
    bool abort();  // About execution and "Brake"
    std::vector<sysSelectedAlgs> selectAlgorithm();  // Determine which algorithm to use
    sysSelectedDetermineFun selectDetermineFunction();  // Determine which determine function to use

    CNCInterface cnc_;
    std::map<sysSelectedAlgs, BaseAlg> algorithmInstances_;
    bool isTerminated = false;

    // Next State
    float relativeTurnAngle_ = 0;
    float targetSpeed_ = 0;
    GPSPoint nextWaypoint_;
    GPSPoint nextRelativeXYZ_;

    // Current State
    sysState currState_ = sysState::SYS_IDLE;
    std::vector<sysSelectedAlgs> selectedAlgorithm_;
    sysSelectedDetermineFun selectedDetermineFun_;
    float currentHeading_;
    float currentSpeed_;
};

#endif  // NOLINT
