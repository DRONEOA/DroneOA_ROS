/* Copyright (C) 2020 DroneOA Group - All Rights Reserved
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

#ifndef HWI_INTERFACE_DEPTHCAMINTERFACE_HPP_  // NOLINT
#define HWI_INTERFACE_DEPTHCAMINTERFACE_HPP_  // NOLINT

#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

namespace Depth {

class DepthCamInterface {
 public:
    virtual ~DepthCamInterface() {}

    // Init
    virtual void initWatcherThread() = 0;

    // Accessor
    virtual float getMaxRange() = 0;
    virtual float getMinRange() = 0;
    virtual sensor_msgs::Image getDepthImage() = 0;
    virtual sensor_msgs::PointCloud2 getPC2Cloud() = 0;

    // Debug Print
    virtual void printImgInfo() = 0;
};

}  // namespace Depth

#endif  // HWI_INTERFACE_DEPTHCAMINTERFACE_HPP_  // NOLINT
