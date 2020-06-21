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

#include <droneoa_ros/OAC/OAAlgGLO.hpp>

namespace OAC {

OAAlgGLO::OAAlgGLO(CNC::CNCInterface *cnc) : BaseAlg(cnc) {
    start3DGLO = mNode.advertise<std_msgs::String>("droneOA/generate3DGLOBAL", 1000);
    init();
}

void OAAlgGLO::init() {
    ROS_INFO("3D_global initialize.");
    // std_msgs::String msg;
    // std::stringstream ss;
    // ss << "initializing 3D-global algo";
    // msg.data == ss.str();
    // start3DGLO.publish(msg);
    thread_watch_path_ = new boost::thread(boost::bind(&OAAlgGLO::watchPathThread, this));
}

OAAlgGLO::~OAAlgGLO() {
    CMDQueue_.clear();
}

bool OAAlgGLO::collect() {
    // Collect data directly from interface as required
    //! @todo WIP
    if (!mpCNC) {
        ROS_ERROR("[OAAlgGLO] Missing CNC pointer !!!");
        return false;
    }
    //! @todo remain false until implemented
    // getData = mNode.subscribe("/result", 1, &OAAlgGLO::pathCallback, this);
    return false;
}

bool OAAlgGLO::plan() {
    CMDQueue_.clear();
    DATAQueue_.clear();
    DATAQueue_.push_back(Command::DataLine(Command::DATA_QUEUE_TYPES::DATA_ALG_NAME, ALG_STR_GLO));
    DATAQueue_.push_back(Command::DataLine(Command::DATA_QUEUE_TYPES::DATA_CONFIDENCE, std::to_string(0.0f)));
    //! @todo remain false until implemented
    return false;
}
void OAAlgGLO::pathCallback(const mavros_msgs::Trajectory& msg) {
    //! @todo a;lsdja kjdklsj o;al
}
void OAAlgGLO::watchPathThread() {
    auto rate = ros::Rate(OAC_REFRESH_FREQ);
    auto gpsFix_sub =
        mNode.subscribe("/droneOA/3Dresult", 1, &OAAlgGLO::pathCallback, this);

    while (ros::ok()) {
        ros::spinOnce();
        rate.sleep();
    }
}
}  // namespace OAC
