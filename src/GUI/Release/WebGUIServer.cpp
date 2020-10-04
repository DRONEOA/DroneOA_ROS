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
 * Written by Bohan Shi <b34shi@uwaterloo.ca>, Oct 2020
 */

#include <std_msgs/String.h>
#include <droneoa_ros/GUI/Release/WebGUIServer.hpp>
#include <droneoa_ros/HWI/base/CNCGeneric.hpp>
#include <droneoa_ros/HWI/Utils/CNCUtils.hpp>

namespace GUI {

WebGUIServer::WebGUIServer(std::string sessionName, CNC::CNCGeneric* cnc) : GUISubscriber(sessionName) {
    mpCNC = cnc;
    mpCNC->GUI::GUISubject::registerGUIPopup(this);
    mWebGUIInfoPub = mNodeHandle.advertise<std_msgs::String>("droneoa/gui_data", 1000);
}

WebGUIServer::~WebGUIServer() {}

std::string WebGUIServer::BuildDataString() {
    geometry_msgs::Vector3 RPY = CNC::CNCUtility::quaternionToRPY(mpCNC->getIMUData().orientation);
    // Post Infomation Package For GUI
    std_msgs::String guiMsg;
    std::stringstream ss;
    ss << mpCNC->getMode() << " "
        << mpCNC->getRelativeAltitude() << " "
        << mpCNC->getBatteryVoltage() << " "
        << mpCNC->getHUDData().climb << " "
        << mpCNC->getHUDData().heading << " "
        << mpCNC->getHUDData().groundspeed << " "
        << mpCNC->getHUDData().throttle << " "
        << RPY.x << " " << RPY.y << " "
        << mpCNC->getHUDData().airspeed;
    return ss.str();
}

void WebGUIServer::UpdateView(GUISubject *subject) {
    std_msgs::String guiMsg;
    guiMsg.data = BuildDataString();
    mWebGUIInfoPub.publish(guiMsg);
}

}  // namespace GUI
