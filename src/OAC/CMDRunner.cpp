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

#include <ros/ros.h>
#include <droneoa_ros/OAC/CMDRunner.hpp>
#include <droneoa_ros/HWI/CNCArdupilot.hpp>

namespace OAC {

CMDRunner::CMDRunner(CNC::CNCInterface *cnc) : runnerState(RUNNER_STATE::INIT), shutdown(false), mpCNC(cnc),
        mWaitUntilMode(UNTIL_MODE::NONE) {
    toggleState(RUNNER_STATE::INIT);
    runnerThread = new boost::thread(boost::bind(&CMDRunner::runnerRoutine, this));
}

void CMDRunner::clearCMDQueue() {
    ROS_DEBUG("[CMDRunner] %s", __func__);
    boost::unique_lock<boost::shared_mutex> uniqueLock(queue_mutex);
    theCMDQueue.clear();
}

bool CMDRunner::populateCMDQueue(Command::CommandQueue commands) {
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
    return shutdown;
}

bool CMDRunner::setupRunner(Command::CommandQueue commands) {
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
            // In case last command is a wait, we ignore it
            toggleState(RUNNER_STATE::IDLE);
            internalTimmer = 0;
            continue;
        } else {
            toggleState(RUNNER_STATE::RUNNING);
        }
        // Run the timer if any
        if (internalTimmer > 0) {
            if (mWaitUntilMode == UNTIL_MODE::ARRWP) {
                // Recheck whether wp list size changed for until command
                CNC::CNCArdupilot* advCNC = dynamic_cast<CNC::CNCArdupilot*>(mpCNC);
                if (!advCNC) {
                    ROS_ERROR("This FCU does not support arrwp command !!!");
                }
                int currentWPListSize = advCNC->getWaypointList().waypoints.size();
                if ((mWaypointListSize != currentWPListSize) || (currentWPListSize == 1 && checkReachLastWP())) {
                    //! @note Seems there is a bug in mavros. The last WP will remain in list with SITL
                    mWaitUntilMode = UNTIL_MODE::NONE;
                    internalTimmer = 0;
                }
            } else if (mWaitUntilMode == UNTIL_MODE::CLRWP) {
                // Recheck whether wp list size changed for until command
                CNC::CNCArdupilot* advCNC = dynamic_cast<CNC::CNCArdupilot*>(mpCNC);
                if (!advCNC) {
                    ROS_ERROR("This FCU does not support arrwp command !!!");
                }
                int currentWPListSize = advCNC->getWaypointList().waypoints.size();
                if ((currentWPListSize == 0) || (currentWPListSize == 1 && checkReachLastWP())) {
                    //! @note Seems there is a bug in mavros. The last WP will remain in list with SITL
                    mWaitUntilMode = UNTIL_MODE::NONE;
                    internalTimmer = 0;
                }
            }
            if (internalTimmer >= RUNNER_TICK_TIME) {
                internalTimmer -= RUNNER_TICK_TIME;
                continue;
            } else {
                mWaitUntilMode = UNTIL_MODE::NONE;  // Timeout reset
                internalTimmer = 0;
            }
        }
        if (theCMDQueue.front().first == Command::CMD_QUEUE_TYPES::CMD_DELAY_MSEC) {
            // Handle Delay
            try {
                internalTimmer = std::stoi(theCMDQueue.front().second);
                ROS_WARN("[CMDRunner] Timer Start With: %u", internalTimmer);
                theCMDQueue.erase(theCMDQueue.begin());
            } catch(...) {
                ROS_ERROR("[CMDRunner] Delay Time Data Invalid");
                clearCMDQueue();
            }
        } else if (theCMDQueue.front().first == Command::CMD_QUEUE_TYPES::CMD_UNTIL) {
            // Handle Until
            try {
                if (theCMDQueue.front().second == "arrwp") {
                    // Until Arrive At Waypoint
                    CNC::CNCArdupilot* advCNC = dynamic_cast<CNC::CNCArdupilot*>(mpCNC);
                    if (!advCNC) {
                        ROS_ERROR("This FCU does not support arrwp command !!!");
                        throw 1;
                    }
                    //! @note Seems there is a bug in mavros. Which causing Reach message not send with SITL
                    // advCNC->registForReachEvent(std::bind(&CMDRunner::reachWaypointCallback, this));
                    mWaypointListSize = advCNC->getWaypointList().waypoints.size();
                    mWaitUntilMode = UNTIL_MODE::ARRWP;
                    ROS_WARN("[CMDRunner] Until Arrive At Waypoint. Current Num WP: %u", mWaypointListSize);
                } else if (theCMDQueue.front().second == "clrwp") {
                    // Until Clear All Waypoints
                    CNC::CNCArdupilot* advCNC = dynamic_cast<CNC::CNCArdupilot*>(mpCNC);
                    if (!advCNC) {
                        ROS_ERROR("This FCU does not support arrwp command !!!");
                        throw 1;
                    }
                    mWaitUntilMode = UNTIL_MODE::CLRWP;
                    ROS_WARN("[CMDRunner] Until Clear All Waypoints");
                } else {
                    // Invalid mode data
                    throw 1;
                }
                ROS_WARN("[CMDRunner] Until Start with timeout limit: %u", RUNNER_TIMEOUT_LIMIT);
                internalTimmer = RUNNER_TIMEOUT_LIMIT;
                theCMDQueue.erase(theCMDQueue.begin());
            } catch(...) {
                ROS_ERROR("[CMDRunner] Until Data Invalid");
                clearCMDQueue();
            }
        } else {
            // Common Instant Commands
            Command::parseCMD(mpCNC, theCMDQueue.front());
            theCMDQueue.erase(theCMDQueue.begin());
        }
    }
}

// Helpers
bool CMDRunner::checkReachLastWP() {
    CNC::CNCArdupilot* advCNC = dynamic_cast<CNC::CNCArdupilot*>(mpCNC);
    if (!advCNC) {
        return false;
    }
    uint32_t currentWPListSize = advCNC->getWaypointList().waypoints.size();
    if (currentWPListSize == 0) {
        return true;
    }
    GPSPoint currentPos = mpCNC->getCurrentGPSPoint();
    GPSPoint wp = GPSPoint(advCNC->getWaypointList().waypoints[0].x_lat,
                            advCNC->getWaypointList().waypoints[0].y_long,
                            advCNC->getWaypointList().waypoints[0].z_alt);
    return wp == currentPos;
}

// Callbasks
void CMDRunner::reachWaypointCallback() {
    ROS_WARN("[CMDRunner] Reached A Waypoint");
    //! @note Seems there is a bug in mavros. Which causing Reach message not send with SITL
    // mReachWaypoint = true;
}

CMDRunner::~CMDRunner() {
    if (runnerThread) {
        shutdown = true;
        runnerThread->join();
        delete runnerThread;
    }
    ROS_INFO("Destroy CMDRunner");
}

}  // namespace OAC
