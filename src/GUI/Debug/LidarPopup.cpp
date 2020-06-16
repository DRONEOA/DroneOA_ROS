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
 * Written by Bohan Shi <b34shi@edu.uwaterloo.ca>, June 2020
 */

#include <droneoa_ros/GUI/Debug/LidarPopup.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <droneoa_ros/HWI/base/LidarGeneric.hpp>

namespace Lidar {

LidarPopup::LidarPopup(std::string windowName, LidarGeneric* lidar) {
    mpLidar = lidar;
    OPENCV_WINDOW_LIDAR = windowName;
    cv::namedWindow(OPENCV_WINDOW_LIDAR);
    cv::startWindowThread();  //! @todo do we need delay
    mpLidar->registerGUIPopup(this);
}

LidarPopup::~LidarPopup() {
    cv::destroyWindow(OPENCV_WINDOW_LIDAR);
    ROS_INFO("Destroy LidarPopup");
}

void LidarPopup::UpdateView() {
    mSectorDataMap = mpLidar->getSectorDataMap();
    mClosestPoint = mpLidar->getClosestSectorData();
    drawLidarData();
}

void LidarPopup::drawLidarData() {
    int32_t winWidth = 800;
    int32_t winHeight = 800;
    cv::Point center(winWidth / 2, winHeight / 2);
    cv::Mat lidarDisk(winWidth, winHeight, CV_8UC3, cv::Scalar(0, 0, 0));
    // Draw center
    cv::circle(lidarDisk, center, 4, cv::Scalar(200, 0, 0), 2);
    // Draw lidar points
    for (auto it = mSectorDataMap.begin(); it != mSectorDataMap.end(); it++) {
        drawLidarPoint(lidarDisk, center, it->first, (it->second)[0]);
    }
    // Draw closest
    drawLidarPoint(lidarDisk, center, mClosestPoint.first, mClosestPoint.second, true);

    cv::imshow(OPENCV_WINDOW_LIDAR, lidarDisk);
}

void LidarPopup::drawLidarPoint(const cv::Mat &img, const cv::Point &center, float angle, float range, bool isLine) {
    range *= LIDAR_POPUP_SCALE;  // zoom up to draw better radar map
    double angleradians = angle * M_PI / 180.0f;
    double x = range * std::sin(angleradians);
    double y = range * std::cos(angleradians);
    x += center.x;
    y = center.y - y;
    if (isLine) {
        cv::line(img, center, cv::Point(x, y), cv::Scalar(0, 100, 100), 2);
    } else {
        cv::circle(img, cv::Point(x, y), 2, cv::Scalar(0, 0, 200));
    }
}

}  // namespace Lidar
