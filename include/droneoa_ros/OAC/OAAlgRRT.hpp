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

#ifndef OAC_OAALGRRT_HPP_  // NOLINT
#define OAC_OAALGRRT_HPP_  // NOLINT

#include <droneoa_ros/OAC/BaseAlg.hpp>
#include <droneoa_ros/OAC/OMPLPlanner.hpp>
#include <droneoa_ros/HWI/Utils/LocalPoint.hpp>

namespace OAC {

class OAAlgRRT : public BaseAlg {
    OMPLPlanner mPlanner;
    LocalPoint previousStart;
    LocalPoint previousGoal;
    int32_t mCurrentSolutionRevision;
    // Helper
    void populateCMD_GPS();
    void populateCMD_ENU();
    void populateDATA();
    void setStartPosGPS();
    void setStartPosENU();
    void setGoalPosGPS();
    void setGoalPosENU();

 public:
    explicit OAAlgRRT(CNC::CNCInterface *cnc);
    ~OAAlgRRT() override;
    void init();  // For restart
    /**
     * @brief Collect Data From CNC.
     * Set/Update start position and goal position for the OMPL RRT planner if the change is significant enough.
     * @return false if a valid pointer to cnc is missing
     */
    bool collect() override;  // Collect required sensor data
    /**
     * @brief Generate command queue based on calculated path from OMPL planner.
     * Note: the path only update if there is a change in goal position OR the octomap is updated and causing the
     * previous planned path to be invalid. Data queue is also populated with: algorithm name, confidence,
     * and path cost.
     * @return true all the time
     */
    bool plan() override;  // Return false when get around is impossible
};

}  // namespace OAC

#endif  // OAC_OAALGRRT_HPP_  // NOLINT
