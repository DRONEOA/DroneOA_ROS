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

#ifndef OAC_CMDRUNNER_HPP_  // NOLINT
#define OAC_CMDRUNNER_HPP_  // NOLINT

#include <boost/thread.hpp>
#include <droneoa_ros/OAC/Command.hpp>
#include <droneoa_ros/HWI/interface/CNCInterface.hpp>

namespace OAC {

/**
 * @brief 5 States Of The Runner
 */
enum RUNNER_STATE {
    INIT = 0,  /*!< initilize the runner */
    RUNNING,  /*!< the runner is processing a cmd queue, delay is treated as running */
    PAUSED,  /*!< the runner is pause due to unsatified condition of manual input */
    STOPPED,  /*!< the runner is stopped due to fatal error or shutdown */
    IDLE,  /*!< the runner is idle or have an empty cmd queue */
};

static const char* RUNNER_STATE_STR[] = {
    "INIT",
    "RUNNING",
    "PAUSED",
    "STOPPED",
    "IDLE"
};

/**
 * @brief Command specific enums
 */
enum UNTIL_MODE {
    NONE = 0,  /*!< Until: None (disabled) */
    ARRWP,  /*!< Until: Arrive at next waypoint */
    CLRWP,  /*!< Until: Clear all waypoints */
};

/**
 * @brief Interval between runner ticks (usually 50 ~ 250)
 */
static const int32_t RUNNER_TICK_TIME = 100;

class CMDRunner {
    void runnerRoutine();
    boost::thread *runnerThread;
    CNC::CNCInterface *mpCNC;

    Command::CommandQueue theCMDQueue;
    RUNNER_STATE runnerState;
    bool shutdown;
    boost::shared_mutex queue_mutex;
    boost::shared_mutex state_mutex;

    uint32_t mWaypointListSize;
    UNTIL_MODE mWaitUntilMode;  // 0 = none; 1 = arrwp; 2 = clrwp
    bool checkReachLastWP();

    void clearCMDQueue();
    bool populateCMDQueue(Command::CommandQueue commands);
    bool toggleState(RUNNER_STATE newState);
    bool isShutDownRequested();

 public:
    explicit CMDRunner(CNC::CNCInterface *cnc);
    virtual ~CMDRunner();
    /**
     * @brief Send a new command queue to the runner
     * This operation will replace the original command queue in the runner
     * @param commands a CommandQueue
     * @return true operation is successful
     * @return false condition unsatisfied or command queue contains error
     */
    bool setupRunner(Command::CommandQueue commands);
    bool startRunner();
    //! @todo(shibohan) interrupt and management

    /**
     * @brief Get the current Runner State
     * @return RUNNER_STATE
     */
    RUNNER_STATE getRunnerState();

    // Callbacks
    void reachWaypointCallback();
};

}  // namespace OAC

#endif  // OAC_CMDRUNNER_HPP_  // NOLINT
