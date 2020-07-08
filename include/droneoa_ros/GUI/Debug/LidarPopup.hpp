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
 * Written by Bohan Shi <b34shi@uwaterloo.ca>, June 2020
 */

#ifndef GUI_DEBUG_LIDARPOPUP_  // NOLINT
#define GUI_DEBUG_LIDARPOPUP_  // NOLINT

#include <map>
#include <string>
#include <utility>
#include <vector>
#include <opencv2/highgui/highgui.hpp>
#include <droneoa_ros/GUI/GUISubscriber.hpp>

namespace Lidar {

class LidarGeneric;

}  // namespace Lidar

namespace CNC {

class CNCArdupilot;

}  // namespace CNC

namespace GUI {

class LidarPopup : public GUISubscriber {
    Lidar::LidarGeneric* mpLidar;
    CNC::CNCArdupilot* mpCNC;
    std::map<float, std::vector<float>> mSectorDataMap;
    std::pair<float, float> mClosestPoint;
    void drawLidarData();
    cv::Mat drawWPList(const cv::Mat & lidarDisk, const cv::Point &center);
    cv::Point getPointFromBearingDistance(const cv::Point &center, float bearing, float range);
    void drawLidarPoint(const cv::Mat &img, const cv::Point &center, float angle, float range, bool isLine = false);

 public:
    LidarPopup(std::string windowName, Lidar::LidarGeneric* lidar, CNC::CNCArdupilot* cnc = nullptr);
    ~LidarPopup();
    void UpdateView(GUISubject *subject);
};

}  // namespace GUI

#endif  // GUI_DEBUG_LIDARPOPUP_  // NOLINT
