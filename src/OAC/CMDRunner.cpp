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

CMDRunner::CMDRunner(CNC::CNCInterface *cnc) : runnerState(RUNNER_STATE::INIT), shutdown(false), mInternalTimmer(0),
        mpCNC(cnc), mWaitUntilMode(UNTIL_MODE::NONE) {
    toggleState(RUNNER_STATE::INIT);
    runnerThread = new boost::thread(boost::bind(&CMDRunner::runnerRoutine, this));
}

void CMDRunner::clearCMDQueue() {
    ROS_DEBUG("[CMDRunner] %s", __func__);
    boost::unique_lock<boost::shared_mutex> uniqueLock(queue_mutex);
    theCMDQueue.clear();
    resetTimer();
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
    resetTimer();
    populateCMDQueue(commands);
}

void CMDRunner::cancelCurrentQueueExec() {
    clearCMDQueue();
}

RUNNER_STATE CMDRunner::getRunnerState() {
    boost::shared_lock<boost::shared_mutex> lock(state_mutex);
    return runnerState;
}

void CMDRunner::resetTimer() {
    mWaitUntilMode = UNTIL_MODE::NONE;
    mInternalTimmer = 0;
}

/*
 * Main Thread
 */

void CMDRunner::runnerRoutine() {
    while (true) {
        boost::this_thread::sleep(boost::posix_time::milliseconds(RUNNER_TICK_TIME));
        // Terminate Thread On Shutdown Request
        if (isShutDownRequested()) {
            toggleState(RUNNER_STATE::STOPPED);
            break;
        }
        if (theCMDQueue.empty()) {
            //! @note In case last command is a wait, we ignore it.
            //     Any Time Wait (at the end) Start --> Queue Empty --> next tick --> Timer reset
            toggleState(RUNNER_STATE::IDLE);
            resetTimer();
            continue;
        } else {
            toggleState(RUNNER_STATE::RUNNING);
        }
        // Run the timer if any
        if (mInternalTimmer > 0) {
            recheckUntilCommand();
            if (mInternalTimmer >= RUNNER_TICK_TIME) {
                mInternalTimmer -= RUNNER_TICK_TIME;
                continue;
            } else {
                resetTimer();
            }
        }
        // Handle runner specific commands
        if (theCMDQueue.front().first == Command::CMD_QUEUE_TYPES::CMD_DELAY_MSEC) {
            // Handle Delay
            try {
                mInternalTimmer = std::stoi(theCMDQueue.front().second);
                ROS_WARN("[CMDRunner] Timer Start With: %u", mInternalTimmer);
                theCMDQueue.erase(theCMDQueue.begin());
            } catch(...) {
                ROS_ERROR("[CMDRunner] Delay Time Data Invalid");
                clearCMDQueue();
            }
        } else if (theCMDQueue.front().first == Command::CMD_QUEUE_TYPES::CMD_UNTIL) {
            // Handle Until
            handleUntilCommand();
        } else {
            // Common Instant Commands
            // Using OAC privilege by default
            if (!Command::parseCMD(mpCNC, theCMDQueue.front(), true)) {
                // Command Execution Failed. Brake and cancel queue execution.
                ROS_ERROR("[CMDRunner] Error During Command Exec. Queue Exec Canceled. Set vehicle to BRAKE.");
                clearCMDQueue();
                Command::parseCMD(mpCNC, Command::CommandLine(
                        Command::CMD_QUEUE_TYPES::CMD_CHMOD, FLT_MODE_BRAKE), true);
                continue;
            }
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

bool CMDRunner::handleUntilCommand() {
    try {
        std::vector<std::string> dataTokens = Command::getDataListFromString(theCMDQueue.front().second);
        if (dataTokens.at(0) == "arrwp") {
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
        } else if (dataTokens.at(0) == "clrwp") {
            // Until Clear All Waypoints
            CNC::CNCArdupilot* advCNC = dynamic_cast<CNC::CNCArdupilot*>(mpCNC);
            if (!advCNC) {
                ROS_ERROR("This FCU does not support arrwp command !!!");
                throw 1;
            }
            mWaitUntilMode = UNTIL_MODE::CLRWP;
            ROS_WARN("[CMDRunner] Until Clear All Waypoints");
        } else if (dataTokens.at(0) == "alt") {
            if (dataTokens.size() >= 3) {
                if (dataTokens.at(1) == "eq") {
                    mWaitUntilMode = UNTIL_MODE::ALTEQ;
                } else if (dataTokens.at(1) == "gt") {
                    mWaitUntilMode = UNTIL_MODE::ALTGT;
                } else if (dataTokens.at(1) == "lt") {
                    mWaitUntilMode = UNTIL_MODE::ALTLT;
                } else {
                    throw 1;
                }
            }
            mUntilAlt = std::stof(dataTokens.at(2));
            ROS_WARN("[CMDRunner] Until Altitude %s: %f", dataTokens.at(1).c_str(), mUntilAlt);
        } else {
            // Invalid mode data
            throw 1;
        }
        ROS_WARN("[CMDRunner] Until Start with timeout limit: %u", RUNNER_TIMEOUT_LIMIT);
        mInternalTimmer = RUNNER_TIMEOUT_LIMIT;
        theCMDQueue.erase(theCMDQueue.begin());
    } catch(...) {
        ROS_ERROR("[CMDRunner] Until Data Invalid");
        clearCMDQueue();
        return false;
    }
    return true;
}

bool CMDRunner::recheckUntilCommand() {
    if (mWaitUntilMode == UNTIL_MODE::ARRWP) {
        // Recheck whether wp list size changed for until command
        CNC::CNCArdupilot* advCNC = dynamic_cast<CNC::CNCArdupilot*>(mpCNC);
        if (!advCNC) {
            ROS_ERROR("This FCU does not support arrwp command !!!");
            return false;
        }
        if (OAC_USE_SETPOINT_ENU) {
            if (mpCNC->getLocalPosition() == mpCNC->getCurrentLocalENUTarget()) {
                resetTimer();
            }
        } else {
            int currentWPListSize = advCNC->getWaypointList().waypoints.size();
            if ((mWaypointListSize != currentWPListSize) || (currentWPListSize == 1 && checkReachLastWP())) {
                //! @note Seems there is a bug in mavros. The last WP will remain in list with SITL
                resetTimer();
            }
        }
    } else if (mWaitUntilMode == UNTIL_MODE::CLRWP) {
        // Recheck whether wp list size changed for until command
        CNC::CNCArdupilot* advCNC = dynamic_cast<CNC::CNCArdupilot*>(mpCNC);
        if (!advCNC) {
            ROS_ERROR("This FCU does not support arrwp command !!!");
            return false;
        }
        if (OAC_USE_SETPOINT_ENU) {
            //! @note Since we only hold one setpoint goal at the moment, so arrive means clear all.
            //! @todo consider supporting local NEU waypoint queue ?
            if (mpCNC->getLocalPosition() == mpCNC->getCurrentLocalENUTarget()) {
                resetTimer();
            }
        } else {
            int currentWPListSize = advCNC->getWaypointList().waypoints.size();
            if ((currentWPListSize == 0) || (currentWPListSize == 1 && checkReachLastWP())) {
                //! @note Seems there is a bug in mavros. The last WP will remain in list with SITL
                resetTimer();
            }
        }
    } else if (mWaitUntilMode == UNTIL_MODE::ALTEQ) {
        // Recheck whether altitude equal to
        float currentAlt = mpCNC->getRelativeAltitude();
        if (abs(currentAlt - mUntilAlt) <= ALT_COMPARE_DIFF_MAX) {
            resetTimer();
        }
    } else if (mWaitUntilMode == UNTIL_MODE::ALTLT) {
        // Recheck whether altitude less than
        float currentAlt = mpCNC->getRelativeAltitude();
        if (currentAlt < mUntilAlt) {
            resetTimer();
        }
    } else if (mWaitUntilMode == UNTIL_MODE::ALTGT) {
        // Recheck whether altitude greater than
        float currentAlt = mpCNC->getRelativeAltitude();
        if (currentAlt > mUntilAlt) {
            resetTimer();
        }
    }
    return true;
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
