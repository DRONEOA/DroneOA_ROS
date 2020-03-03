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

#include <droneoa_ros/OAC/CMDRunner.hpp>

CMDRunner::CMDRunner(CNCInterface *cnc) : runnerState(RUNNER_STATE::INIT), shutdown(false), cnc_(cnc) {
    toggleState(RUNNER_STATE::INIT);
    runnerThread = new boost::thread(boost::bind(&CMDRunner::runnerRoutine, this));
}

void CMDRunner::clearCMDQueue() {
    ROS_DEBUG("[CMDRunner] %s", __func__);
    boost::unique_lock<boost::shared_mutex> uniqueLock(queue_mutex);
    theCMDQueue.clear();
}

bool CMDRunner::populateCMDQueue(CommandQueue commands) {
    //! @todo(shibohan) pre-check commands here
    ROS_DEBUG("[CMDRunner] %s", __func__);
    boost::unique_lock<boost::shared_mutex> uniqueLock(queue_mutex);
    theCMDQueue.clear();
    theCMDQueue = commands;
    return true;
}

bool CMDRunner::toggleState(RUNNER_STATE newState) {
    //! @todo(shibohan) check for invalid state changes
    boost::unique_lock<boost::shared_mutex> uniqueLock(state_mutex);
    if (runnerState != newState) {
        ROS_WARN("[CMDRunner] %s from %s to %s", __func__, RUNNER_STATE_STR[runnerState], RUNNER_STATE_STR[newState]);
        runnerState = newState;
    }
    return true;
}

bool CMDRunner::isShutDownRequested() {
    boost::shared_lock<boost::shared_mutex> lock(shutdown_mutex);
    return shutdown;
}

bool CMDRunner::setupRunner(CommandQueue commands) {
    populateCMDQueue(commands);
}

bool CMDRunner::startRunner() {
    //! @todo do we need this ?
}

RUNNER_STATE CMDRunner::getRunnerState() {
    boost::shared_lock<boost::shared_mutex> lock(state_mutex);
    return runnerState;
}

/*
 * Main Thread
 */

void CMDRunner::runnerRoutine() {
    uint32_t internalTimmer = 0;
    while (true) {
        boost::this_thread::sleep(boost::posix_time::milliseconds(RUNNER_TICK_TIME));
        // Terminate Thread On Shutdown Request
        if (isShutDownRequested()) {
            toggleState(RUNNER_STATE::STOPPED);
            break;
        }
        if (theCMDQueue.empty()) {
            toggleState(RUNNER_STATE::IDLE);
            internalTimmer = 0;
            continue;
        } else {
            toggleState(RUNNER_STATE::RUNNING);
        }
        // Run the timer if any
        if (internalTimmer > 0) {
            if (internalTimmer >= RUNNER_TICK_TIME) {
                internalTimmer -= RUNNER_TICK_TIME;
                continue;
            } else {
                internalTimmer = 0;
            }
        }
        if (theCMDQueue.front().first == CMD_QUEUE_TYPES::CMD_DELAY_MSEC) {
            // Handle Delay
            try {
                internalTimmer = std::stoi(theCMDQueue.front().second);
                ROS_WARN("[CMDRunner] Timer Start With: %u", internalTimmer);
                theCMDQueue.erase(theCMDQueue.begin());
            } catch(...) {
                ROS_ERROR("Delay Time Data Invalid");
                clearCMDQueue();
            }
        } else {
            // Common Instant Commands
            parseCMD(cnc_, theCMDQueue.front());
            theCMDQueue.erase(theCMDQueue.begin());
        }
    }
}

CMDRunner::~CMDRunner() {
    if (runnerThread) {
        boost::unique_lock<boost::shared_mutex> uniqueLock(shutdown_mutex);
        shutdown = true;
    }
    ROS_INFO("Destroy CMDRunner");
}
