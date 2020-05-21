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
 * Written by Bohan Shi, August 2019
 */

#include <droneoa_ros/OAC/OAAlgFGM.hpp>

namespace OAC {

OAAlgFGM::OAAlgFGM(CNC::CNCInterface *cnc, Lidar::LidarGeneric *lidar) : BaseAlg(cnc) {
    init(lidar);
}

void OAAlgFGM::init(Lidar::LidarGeneric *lidar) {
    lidar_ = lidar;
}

OAAlgFGM::~OAAlgFGM() {
    CMDQueue_.clear();
}

bool OAAlgFGM::collect() {
    // Collect data directly from interface as required
    //! @todo WIP
    if (!cnc_) {
        ROS_ERROR("[OAAlgFGM] Missing CNC pointer !!!");
        return false;
    }
    if (!lidar_) {
        ROS_ERROR("[OAAlgFGM] Missing LIDAR pointer !!!");
        return false;
    }
    //! @todo remain false until implemented
    return false;
}

bool OAAlgFGM::plan() {
    CMDQueue_.clear();
    DATAQueue_.clear();
    DATAQueue_.push_back(Command::DataLine(Command::DATA_QUEUE_TYPES::DATA_ALG_NAME, ALG_STR_FGM));
    DATAQueue_.push_back(Command::DataLine(Command::DATA_QUEUE_TYPES::DATA_CONFIDENCE, std::to_string(0.0f)));
    //! @todo remain false until implemented
    return false;
}

}  // namespace OAC
