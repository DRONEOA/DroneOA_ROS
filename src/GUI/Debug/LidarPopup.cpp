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
#include <droneoa_ros/HWI/CNCArdupilot.hpp>
#include <droneoa_ros/HWI/Utils/CNCUtils.hpp>

namespace GUI {

LidarPopup::LidarPopup(std::string windowName, Lidar::LidarGeneric* lidar, CNC::CNCArdupilot* cnc) :
        GUISubscriber(windowName) {
    mpLidar = lidar;
    mpCNC = cnc;
    cv::namedWindow(OPENCV_WINDOW_NAME);
    cv::startWindowThread();
    mpLidar->GUI::GUISubject::registerGUIPopup(this);
    // mpCNC->GUI::GUISubject::registerGUIPopup(this);  //! @todo cause random crash over time. Maybe concurrency issue?
}

LidarPopup::~LidarPopup() {
    cv::destroyWindow(OPENCV_WINDOW_NAME);
    ROS_DEBUG("Destroy LidarPopup");
}

void LidarPopup::UpdateView(GUISubject *subject) {
    if (subject && dynamic_cast<Lidar::LidarGeneric*>(subject)) {
        mSectorDataMap = mpLidar->getSectorDataMap();
        mClosestPoint = mpLidar->getClosestSectorData();
    }
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
    cv::putText(lidarDisk, "Closest Sector: " + std::to_string(mClosestPoint.first) + " Degree  " +
            std::to_string(mClosestPoint.second) + " Meter", cv::Point(20, 20), cv::FONT_HERSHEY_COMPLEX,
            0.5, cv::Scalar(0, 100, 100), 1, 8, 0);

    if (mpCNC) {
        lidarDisk = drawWPList(lidarDisk, center);
    }

    cv::imshow(OPENCV_WINDOW_NAME, lidarDisk);
}

void LidarPopup::drawLidarPoint(const cv::Mat &img, const cv::Point &center, float angle, float range, bool isLine) {
    if (isLine) {
        cv::line(img, center, getPointFromBearingDistance(center, angle, range), cv::Scalar(0, 100, 100), 2);
    } else {
        cv::circle(img, getPointFromBearingDistance(center, angle, range), 2, cv::Scalar(0, 0, 200));
    }
}

cv::Point LidarPopup::getPointFromBearingDistance(const cv::Point &center, float bearing, float range) {
    range *= LIDAR_POPUP_SCALE;  // zoom up to draw better radar map
    double angleradians = bearing * M_PI / 180.0f;
    double x = range * std::sin(angleradians);
    double y = range * std::cos(angleradians);
    x += center.x;
    y = center.y - y;
    return cv::Point(x, y);
}

cv::Mat LidarPopup::drawWPList(const cv::Mat & lidarDisk, const cv::Point &center) {
    GPSPoint currectPos = mpCNC->getCurrentGPSPoint();
    cv::Point previousWPPoint = center;
    int index = 0;
    for (auto wp : mpCNC->getWaypointList().waypoints) {
        GPSPoint wpGPS = GPSPoint(wp.x_lat, wp.y_long, wp.z_alt);
        float bearing = CNC::CNCUtility::getBearing(currectPos, wpGPS);
        float distance = CNC::CNCUtility::getDistanceMeter(currectPos, wpGPS);
        // Draw the waypoint
        cv::Point currentWPPoint = getPointFromBearingDistance(center, bearing, distance);
        cv::line(lidarDisk, previousWPPoint, currentWPPoint, cv::Scalar(0, 255, 0), 3);
        previousWPPoint = currentWPPoint;
        index++;
    }
    return lidarDisk;
}

}  // namespace GUI
