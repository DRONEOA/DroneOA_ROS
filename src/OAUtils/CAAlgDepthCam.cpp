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
 * Written by Xiao Zhou <x258zhou@edu.uwaterloo.ca>, Nov. 2019
 */

#include <droneoa_ros/OAUtils/CAAlgDepthCam.hpp>

CAAlgDepthCam::CAAlgDepthCam(CNCInterface *cnc, RSCInterface *rsc) : BaseAlg(cnc) {
    init(rsc);
}

CAAlgDepthCam::~CAAlgDepthCam() {}

void CAAlgDepthCam::init(RSCInterface *rsc) {
    rsc_ = rsc;
}  // For restart
bool CAAlgDepthCam::collect() {}  // Collect required sensor data
bool CAAlgDepthCam::plan() {}