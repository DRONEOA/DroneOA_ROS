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

#include <droneoa_ros/HWI/OctomapClient.hpp>

#include <boost/bind.hpp>
#include <boost/thread/thread.hpp>

namespace SLAM {

OctomapClient::OctomapClient(ros::NodeHandle node, ros::Rate rate) : mNodeHandle(node), mRate(rate) {}

OctomapClient::~OctomapClient() {
    if (mpThreadWatchOctomap) delete mpThreadWatchOctomap;
}

void OctomapClient::initWatcherThread() {
    mpThreadWatchOctomap = new boost::thread(boost::bind(&OctomapClient::watchOctomapThread, this));
}

octomap_msgs::Octomap OctomapClient::getOctomap() {
    return mCurrentMap;
}

void OctomapClient::octomap_callback(const octomap_msgs::Octomap::ConstPtr& msg) {
    mCurrentMap = *msg;
}

void OctomapClient::watchOctomapThread() {
    auto mapSub =
        mNodeHandle.subscribe<octomap_msgs::Octomap>("/octomap_full", 1,
                boost::bind(&OctomapClient::octomap_callback, this, _1));

    while (ros::ok()) {
        ros::spinOnce();
        mRate.sleep();
    }
}

void OctomapClient::printDebugInfo() {
    ROS_INFO("------ Octomap Info Start ------");
    ROS_INFO("> ID:         %s", mCurrentMap.id.c_str());
    ROS_INFO("> Resolution: %f", mCurrentMap.resolution);
    ROS_INFO("> Data Size:  %zu", mCurrentMap.data.size());
    ROS_INFO("------  Octomap Info End  ------");
}

}  // namespace SLAM
