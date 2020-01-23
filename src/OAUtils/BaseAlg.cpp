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

#include <droneoa_ros/OAUtils/BaseAlg.hpp>

void BaseAlg::init(CNCInterface *cnc) {
    cnc_ = cnc;
}

BaseAlg::BaseAlg(CNCInterface *cnc) {
    init(cnc);
}

BaseAlg::~BaseAlg() {}

CommandQueue BaseAlg::getCommandQueue() {
    return CMDQueue_;
}

DataQueue BaseAlg::getDataQueue() {
    return DATAQueue_;
}
