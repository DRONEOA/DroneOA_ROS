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

CAAlgDepthCam::~CAAlgDepthCam() {
    CMDQueue_.clear();
}

void CAAlgDepthCam::init(RSCInterface *rsc) {
    rsc_ = rsc;
}

float avgInRangeHelper(std::vector<float> source, float min, float max){
    float sum = 0.0f;
    float count = 0.0f;
    for(auto zCoord : source) {
        if(inRange<float>(min, max, zCoord)) {
            sum += zCoord;
            count += 1.0f;
        }
    }
    float avg = sum / count;
    return (std::isnan(avg) ? -1 : avg);
}

bool CAAlgDepthCam::collect() {
    float gSpeed = cnc_->getHUDData().groundspeed;
    camThreshold_ = gSpeed * 200;
    std::vector<float> zCoords = rsc_->pointCloudZCoordsInRange();
    float sum = 0.0f;
    for(auto zCoord : zCoords) {
        sum += zCoord;
    }
    float avg = sum / zCoords.size();
    ROS_INFO("[CAAlgDepthCam] Avg Z Coords: %f", avg);
    return true;
}

bool CAAlgDepthCam::plan() {
    CMDQueue_.clear();
    DATAQueue_.clear();
    DATAQueue_.push_back(
        std::pair<DATA_QUEUE_TYPES, std::string>(DATA_QUEUE_TYPES::DATA_ALG_NAME, ALG_STR_COLLISION_DEPTH));
    if (true) {
        CMDQueue_.push_back(std::pair<CMD_QUEUE_TYPES, std::string>(CMD_QUEUE_TYPES::CMD_CHMOD, FLT_MODE_BRAKE));
        DATAQueue_.push_back(std::pair<DATA_QUEUE_TYPES, std::string>(
            DATA_QUEUE_TYPES::DATA_CONFIDENCE, std::to_string(0)));
    }
    return true;
}