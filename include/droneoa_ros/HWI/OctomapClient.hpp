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
 */

#ifndef HWI_octomapclient_hpp_  // NOLINT
#define HWI_octomapclient_hpp_  // NOLINT

#include <ros/ros.h>
#include <octomap_msgs/conversions.h>
#include <octomap/octomap.h>

#include <boost/bind.hpp>
#include <boost/thread/thread.hpp>

#include <droneoa_ros/PDN.hpp>

namespace SLAM {

class OctomapClient {
 public:
    OctomapClient(ros::NodeHandle node, ros::Rate rate);
    virtual ~OctomapClient();

    void initWatcherThread();

    void octomap_callback(const octomap_msgs::Octomap::ConstPtr& msg);

    octomap_msgs::Octomap getOctomap();

    void printDebugInfo();

 private:
    ros::NodeHandle mNodeHandle;
    ros::Rate mRate = ros::Rate(GLOBAL_ROS_RATE);

    boost::thread* mpThreadWatchOctomap = nullptr;
    void watchOctomapThread();
    octomap_msgs::Octomap mCurrentMap;
    ros::Subscriber mMapSub;
};

}  // namespace SLAM

#endif  // HWI_octomapclient_hpp_  // NOLINT
