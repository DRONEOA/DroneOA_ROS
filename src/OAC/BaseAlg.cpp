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

#include <ros/ros.h>

#include <droneoa_ros/OAC/BaseAlg.hpp>

namespace OAC {

void BaseAlg::init(CNC::CNCInterface *cnc) {
    mpCNC = cnc;
}

BaseAlg::BaseAlg(CNC::CNCInterface *cnc) {
    init(cnc);
}

BaseAlg::~BaseAlg() {
    ROS_INFO("Destroy BaseAlg");
}

Command::CommandQueue BaseAlg::getCommandQueue() {
    return CMDQueue_;
}

Command::DataQueue BaseAlg::getDataQueue() {
    return DATAQueue_;
}

}  // namespace OAC
