/* Copyright (C) 2020 DroneOA Group - All Rights Reserved
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
 * Written by Bohan Shi <b34shi@edu.uwaterloo.ca>, January 2020
 */

#ifndef INCLUDE_DRONEOA_ROS_OAUTILS_CMDRUNNER_HPP_
#define INCLUDE_DRONEOA_ROS_OAUTILS_CMDRUNNER_HPP_

#include <boost/thread.hpp>
#include "droneoa_ros/OAUtils/Command.hpp"
#include "droneoa_ros/CNCInterface.hpp"

enum RUNNER_STATE {
    INIT = 0,
    RUNNING,
    PAUSED,
    STOPPED,
    IDLE
};

static const char* RUNNER_STATE_STR[] = {
    "INIT",
    "RUNNING",
    "PAUSED",
    "STOPPED",
    "IDLE"
};

static const int RUNNER_TICK_TIME = 100;

class CMDRunner {
    void runnerRoutine();
    boost::thread *runnerThread;
    CNCInterface *cnc_;

    CommandQueue theCMDQueue;
    RUNNER_STATE runnerState;
    bool shutdown;
    boost::shared_mutex queue_mutex;
    boost::shared_mutex state_mutex;
    boost::shared_mutex shutdown_mutex;

    void clearCMDQueue();
    bool populateCMDQueue(CommandQueue commands);
    bool toggleState(RUNNER_STATE newState);
    bool isShutDownRequested();
 public:
    explicit CMDRunner(CNCInterface *cnc);
    virtual ~CMDRunner();
    bool setupRunner(CommandQueue commands);
    bool startRunner();
    //! @todo(shibohan) interrupt and management

    RUNNER_STATE getRunnerState();
};

#endif  // INCLUDE_DRONEOA_ROS_OAUTILS_CMDRUNNER_HPP_
