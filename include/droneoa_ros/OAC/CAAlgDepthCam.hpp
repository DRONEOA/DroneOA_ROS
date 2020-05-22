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

#ifndef OAC_CAALGDEPTHCAM_HPP_  // NOLINT
#define OAC_CAALGDEPTHCAM_HPP_  // NOLINT

#include <droneoa_ros/OAC/BaseAlg.hpp>
#include <droneoa_ros/HWI/RSC.hpp>

namespace OAC {

class CAAlgDepthCam : public BaseAlg {
    Depth::RSC *mpRSC;
    float camThreshold_;
    float camPossibility_;
 public:
    CAAlgDepthCam(CNC::CNCInterface *cnc, Depth::RSC *rsc);
    ~CAAlgDepthCam() override;
    void init(Depth::RSC *rsc);  // For restart
    bool collect() override;  // Collect required sensor data
    bool plan() override;  // Return false when get around is impossible
};

}  // namespace OAC

#endif  // OAC_CAALGDEPTHCAM_HPP_  // NOLINT
