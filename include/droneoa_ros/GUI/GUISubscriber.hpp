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
 * Written by Bohan Shi <b34shi@uwaterloo.ca>, July 2020
 */

#ifndef GUI_SUBSCRIBER_  // NOLINT
#define GUI_SUBSCRIBER_  // NOLINT

#include <string>
#include <droneoa_ros/GUI/GUISubject.hpp>

namespace GUI {

class GUISubscriber {
 public:
    std::string OPENCV_WINDOW_NAME = "";
    explicit GUISubscriber(std::string windowName);
    virtual ~GUISubscriber();
    virtual void UpdateView(GUISubject *subject = nullptr);
};

}  // namespace GUI

#endif  // GUI_SUBSCRIBER_  // NOLINT
