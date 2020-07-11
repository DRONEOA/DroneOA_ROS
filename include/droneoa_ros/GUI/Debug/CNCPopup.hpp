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

#ifndef CNC_DEBUG_CNCPOPUP_  // NOLINT
#define CNC_DEBUG_CNCPOPUP_  // NOLINT

#include <map>
#include <string>
#include <utility>
#include <vector>
#include <opencv2/highgui/highgui.hpp>
#include <droneoa_ros/GUI/GUISubscriber.hpp>

namespace CNC {

class CNCGeneric;

}  // namespace CNC

namespace GUI {

class CNCPopup : public GUISubscriber {
    CNC::CNCGeneric* mpCNC;
    std::pair<int, int> drawBasicStatus(cv::Mat *cncPanel, std::pair<int, int> startPos, float padding);
    std::pair<int, int> drawGPSStatus(cv::Mat *cncPanel, std::pair<int, int> startPos, float padding);
    std::pair<int, int> drawAttitudeStatus(cv::Mat *cncPanel, std::pair<int, int> startPos, float padding);
    std::pair<int, int> drawHUDStatus(cv::Mat *cncPanel, std::pair<int, int> startPos, float padding);
    std::pair<int, int> drawWPStatus(cv::Mat *cncPanel, std::pair<int, int> startPos, float padding);
    std::pair<int, int> drawLocalMissionQueueStatus(cv::Mat *cncPanel, std::pair<int, int> startPos, float padding);
    void drawCNCInfoPanel(cv::Mat *cncPanel);
    void drawText(cv::Mat *targetImg, cv::Point origin, std::string text, double font_scale, int32_t thickness,
        cv::Scalar color = cv::Scalar(94, 206, 165, 255));
    void draw();

 public:
    CNCPopup(std::string windowName, CNC::CNCGeneric* cnc);
    ~CNCPopup();
    void UpdateView(GUISubject *subject) override;
};

}  // namespace GUI

#endif  // CNC_DEBUG_CNCPOPUP_  // NOLINT
