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
#include <droneoa_ros/Utils.hpp>

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
    float a = 50.0f;

    /**
     * The following equation calculates the brack distance of the droone.
     * Credit: High school textbook published by People's Education Press(PEP).
     * Formula: d = (vt^2 - v0^2) / 2a
     * d = braking distance in metres (to be calculated).
     * vt = final speed in m/s.
     * v0 = initial speed in m/s.
     * a = acceleration in m/s^2
     * @TODO test a.
     */
    camThreshold_ =  ((gSpeed * gSpeed) / (2 * a)) * 1000;
    
    if(camThreshold_ < 150.0f) {
        camThreshold_ = 150.0f;
    } // Official documentation said the min dist is 105mm; to be safe, use 150 mm instead.

    std::vector<float> zCoords = rsc_->pointCloudZCoordsInRange();
    float danger = avgInRangeHelper(zCoords, 150.0f, camThreshold_);
    float neutral = avgInRangeHelper(zCoords, camThreshold_, 2*camThreshold_);
    float safe = avgInRangeHelper(zCoords, 2*camThreshold_, 3*camThreshold_);
#ifdef DEBUG_ALG_COLLISION
    ROS_INFO("[CAAlgDepthCam] Avg Z Coords: danger(%f), neutral(%f), safe(%f), threshold=%f", danger, neutral, safe, camThreshold_);
#endif
    
    if(danger != -1) {
        camPossibility_ = 1.0f;
    } else if(neutral != -1) {
        camPossibility_ = scale<float>(neutral, camThreshold_, 2*camThreshold_, 100, 50);
    } else if(safe != -1) {
        camPossibility_ = scale<float>(
            safe,
            2*camThreshold_,
            3*camThreshold_,
            50,
            0);
    } else {
        camPossibility_ = 0.0f;
    }
#ifdef DEBUG_ALG_COLLISION
    ROS_INFO("[CAAlgDepthCam] Possibility: %f", camPossibility_);
#endif
    return true;
}

bool CAAlgDepthCam::plan() {
    CMDQueue_.clear();
    DATAQueue_.clear();
    DATAQueue_.push_back(
        std::pair<DATA_QUEUE_TYPES, std::string>(DATA_QUEUE_TYPES::DATA_ALG_NAME, ALG_STR_COLLISION_DEPTH));
    if (camPossibility_ > 0.5) {
        CMDQueue_.push_back(std::pair<CMD_QUEUE_TYPES, std::string>(CMD_QUEUE_TYPES::CMD_CHMOD, FLT_MODE_BRAKE));
        DATAQueue_.push_back(std::pair<DATA_QUEUE_TYPES, std::string>(
            DATA_QUEUE_TYPES::DATA_CONFIDENCE, std::to_string(camPossibility_)));
    }
    return true;
}
