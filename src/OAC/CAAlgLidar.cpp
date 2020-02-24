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

#include <droneoa_ros/OAC/CAAlgLidar.hpp>

CAAlgLidar::CAAlgLidar(CNCInterface *cnc, LidarInterface *lidar) : BaseAlg(cnc) {
    init(lidar);
}

void CAAlgLidar::init(LidarInterface *lidar) {
    lidar_ = lidar;
    lidarPossibility_ = 0.0;
}

CAAlgLidar::~CAAlgLidar() {
    CMDQueue_.clear();
}

bool CAAlgLidar::collect() {
    // Collect data directly from interface as required
    //! @todo Statue Check e.g. flight mode
    if (!cnc_) {
        ROS_ERROR("[CAAlgLidar] Missing CNC pointer !!!");
        return false;
    }
    if (!lidar_) {
        ROS_ERROR("[CAAlgLidar] Missing LIDAR pointer !!!");
        return false;
    }
    float gSpeed = cnc_->getHUDData().groundspeed;
    lidarThreshold_ = ((gSpeed * gSpeed) / (2 * VEHICLE_MAX_ACCELEATION));  // unit: m
    lidarThreshold_ = lidarThreshold_ < VEHICLE_MIN_SAFE_DISTANCE ? VEHICLE_MIN_SAFE_DISTANCE : lidarThreshold_;
    // Compute Collision Possibility
    std::pair<float, float> closeSector = lidar_->getClosestSectorData();
    if (lidarThreshold_ > closeSector.second) {
        lidarPossibility_ = 1.0;
    } else {
        lidarPossibility_ = 0.0;
    }
    // Debug Prints
#ifdef DEBUG_ALG_COLLISION_LIDAR
    ROS_INFO("[CAAlgLidar] Collect:");
    ROS_INFO("    CNC:   result: groundspeed=%f", gSpeed);
    ROS_INFO("    Lidar: result: %f as range=%f threshold=%f", lidarPossibility_, closeSector.second, lidarThreshold_);
#endif
    return true;
}

bool CAAlgLidar::plan() {
    CMDQueue_.clear();
    DATAQueue_.clear();
    DATAQueue_.push_back(DataLine(DATA_QUEUE_TYPES::DATA_ALG_NAME, ALG_STR_COLLISION_LIDAR));
    if (lidarPossibility_ > 0.75) {
        CMDQueue_.push_back(CommandLine(CMD_QUEUE_TYPES::CMD_CHMOD, FLT_MODE_BRAKE));
        DATAQueue_.push_back(DataLine(DATA_QUEUE_TYPES::DATA_CONFIDENCE, std::to_string(lidarPossibility_)));
    }
    return true;
}
