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

namespace OAC {

static const double DIST_COMPARE_DIFF_MAX = 0.2;

class OAAlgRRT : public BaseAlg {
    OMPLPlanner mPlanner;
    Position3D previousStart;
    Position3D previousGoal;
    int32_t mCurrentSolutionRevision;
    // Helper
    void populateCMD_AUTO();
    void populateCMD_MQueue();
    void populateCMD_Setpoint();
    void populateDATA();
 public:
    explicit OAAlgRRT(CNC::CNCInterface *cnc);
    ~OAAlgRRT() override;
    void init();  // For restart
    bool collect() override;  // Collect required sensor data
    bool plan() override;  // Return false when get around is impossible
};

}  // namespace OAC

#endif  // OAC_OAALGRRT_HPP_  // NOLINT
