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
 * Written by Bohan Shi <b34shi@edu.uwaterloo.ca>, July 2020
 */

#include <ros/ros.h>
#include <droneoa_ros/GUI/Debug/CNCPopup.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <droneoa_ros/HWI/base/CNCGeneric.hpp>
#include <droneoa_ros/HWI/CNCArdupilot.hpp>
#include <droneoa_ros/HWI/Utils/CNCUtils.hpp>

namespace GUI {

CNCPopup::CNCPopup(std::string windowName, CNC::CNCGeneric* cnc) : GUISubscriber(windowName) {
    mpCNC = cnc;
    cv::namedWindow(OPENCV_WINDOW_NAME);
    cv::startWindowThread();
    mpCNC->GUI::GUISubject::registerGUIPopup(this);
}

CNCPopup::~CNCPopup() {
    cv::destroyWindow(OPENCV_WINDOW_NAME);
    ROS_INFO("Destroy CNCPopup");
}

void CNCPopup::drawText(cv::Mat *targetImg, cv::Point origin, std::string text, double font_scale, int32_t thickness,
        cv::Scalar color) {  // BGR
    int32_t font_face = cv::FONT_HERSHEY_COMPLEX;
    int32_t baseline;
    cv::Size text_size = cv::getTextSize(text, font_face, font_scale, thickness, &baseline);
    cv::Point originTop;
    originTop.x = origin.x;
    originTop.y = origin.y - text_size.height;
    if (originTop.x >= 2) originTop.x -= 2;
    if (originTop.y >= 2) originTop.y -= 2;
    cv::Point pt2;
    pt2.x = originTop.x + text_size.width + 2;
    pt2.y = origin.y + 2;
    cv::rectangle(*targetImg, originTop, pt2, cv::Scalar(0), -1, CV_AA);
    cv::putText(*targetImg, text, origin, font_face, font_scale, color, thickness, 8, 0);
}

void CNCPopup::UpdateView(GUISubject *subject) {
    draw();
}

void CNCPopup::drawCNCInfoPanel(cv::Mat *cncPanel) {
    std::pair<int, int> startPos;
    startPos.first = 20;
    startPos.second = 20;
    int padding = 20;
    drawText(cncPanel, cv::Point(startPos.first, startPos.second), "CNC Debug Info Panel:", 0.8, 1);
    startPos = drawBasicStatus(cncPanel, startPos, padding);
    startPos = drawGPSStatus(cncPanel, startPos, padding);
    startPos = drawAttitudeStatus(cncPanel, startPos, padding);
    startPos = drawHUDStatus(cncPanel, startPos, padding);
    if (dynamic_cast<CNC::CNCArdupilot*>(mpCNC)) {
        startPos = drawWPStatus(cncPanel, startPos, padding);
    }
    //! @todo RC Channel Information
    //! @todo setpoint data
}

std::pair<int, int> CNCPopup::drawBasicStatus(cv::Mat *cncPanel, std::pair<int, int> startPos, float padding) {
    startPos.second += 2 * padding;  // Extra padding for section spacing
    drawText(cncPanel, cv::Point(startPos.first, startPos.second), "=== Basic Status =================", 0.5, 1,
            cv::Scalar(238, 238, 238, 255));
    startPos.second += padding;
    std::string FModeDiaply = "Flight Mode: " + mpCNC->getMode();
    drawText(cncPanel, cv::Point(startPos.first, startPos.second), FModeDiaply, 0.5, 1, cv::Scalar(238, 238, 238, 255));
    std::string ArmedDisplay = "Motors:      ";
    startPos.second += padding;
    if (mpCNC->isArmed()) {
        drawText(cncPanel, cv::Point(startPos.first, startPos.second), ArmedDisplay + " Armed  ", 0.5, 1,
                cv::Scalar(0, 255, 102, 255));
    } else {
        drawText(cncPanel, cv::Point(startPos.first, startPos.second), ArmedDisplay + "DisArmed", 0.5, 1,
                cv::Scalar(0, 0, 255, 255));
    }
    startPos.second += padding;
    std::string BattDisplay = "Batt:        " + std::to_string(mpCNC->getBatteryVoltage()) + "V";
    drawText(cncPanel, cv::Point(startPos.first, startPos.second), BattDisplay, 0.5, 1, cv::Scalar(238, 238, 238, 255));

    return startPos;
}

std::pair<int, int> CNCPopup::drawGPSStatus(cv::Mat *cncPanel, std::pair<int, int> startPos, float padding) {
    startPos.second += 2 * padding;  // Extra padding for section spacing
    drawText(cncPanel, cv::Point(startPos.first, startPos.second), "===  GPS Status  =================", 0.5, 1,
            cv::Scalar(238, 238, 238, 255));
    std::string GPSDisplay = "Current Pos: ";
    startPos.second += padding;
    drawText(cncPanel, cv::Point(startPos.first, startPos.second), GPSDisplay, 0.5, 1, cv::Scalar(238, 238, 238, 255));
    GPSDisplay = "         Lat: " + std::to_string(mpCNC->getCurrentGPSPoint().latitude_);
    startPos.second += padding;
    drawText(cncPanel, cv::Point(startPos.first, startPos.second), GPSDisplay, 0.5, 1, cv::Scalar(238, 238, 238, 255));
    GPSDisplay = "       Long: " + std::to_string(mpCNC->getCurrentGPSPoint().longitude_);
    startPos.second += padding;
    drawText(cncPanel, cv::Point(startPos.first, startPos.second), GPSDisplay, 0.5, 1, cv::Scalar(238, 238, 238, 255));
    GPSDisplay = "         Alt: " + std::to_string(mpCNC->getCurrentGPSPoint().altitude_);
    startPos.second += padding;
    drawText(cncPanel, cv::Point(startPos.first, startPos.second), GPSDisplay, 0.5, 1, cv::Scalar(238, 238, 238, 255));
    return startPos;
}

std::pair<int, int> CNCPopup::drawAttitudeStatus(cv::Mat *cncPanel, std::pair<int, int> startPos, float padding) {
    startPos.second += 2 * padding;  // Extra padding for section spacing
    drawText(cncPanel, cv::Point(startPos.first, startPos.second), "=== Attitude Status ===============", 0.5, 1,
            cv::Scalar(238, 238, 238, 255));
    startPos.second += padding;
    std::string AltitudeDisplay = "Relative Altitude: " + std::to_string(mpCNC->getRelativeAltitude());
    drawText(cncPanel, cv::Point(startPos.first, startPos.second), AltitudeDisplay, 0.5, 1,
            cv::Scalar(238, 238, 238, 255));
    startPos.second += padding;
    geometry_msgs::Vector3 RPY = CNC::CNCUtility::quaternionToRPY(mpCNC->getIMUData().orientation);
    std::string IMUDisplay = "IMU: " + std::to_string(RPY.x*57.3) + " "
                                    + std::to_string(RPY.y*57.3) + " "
                                    + std::to_string(RPY.z*57.3);
    drawText(cncPanel, cv::Point(startPos.first, startPos.second), IMUDisplay, 0.5, 1, cv::Scalar(238, 238, 238, 255));
    return startPos;
}

std::pair<int, int> CNCPopup::drawHUDStatus(cv::Mat *cncPanel, std::pair<int, int> startPos, float padding) {
    startPos.second += 2 * padding;  // Extra padding for section spacing
    drawText(cncPanel, cv::Point(startPos.first, startPos.second), "===  HUD  Data  =================", 0.5, 1,
            cv::Scalar(238, 238, 238, 255));
    std::string HUDDisplay = "Airspeed:        " + std::to_string(mpCNC->getHUDData().airspeed);
    startPos.second += padding;
    drawText(cncPanel, cv::Point(startPos.first, startPos.second), HUDDisplay, 0.5, 1, cv::Scalar(238, 238, 238, 255));
    HUDDisplay = "Altitude:        " + std::to_string(mpCNC->getHUDData().altitude);
    startPos.second += padding;
    drawText(cncPanel, cv::Point(startPos.first, startPos.second), HUDDisplay, 0.5, 1, cv::Scalar(238, 238, 238, 255));
    HUDDisplay = "Climb Rate:     " + std::to_string(mpCNC->getHUDData().climb);
    startPos.second += padding;
    drawText(cncPanel, cv::Point(startPos.first, startPos.second), HUDDisplay, 0.5, 1,
            mpCNC->getHUDData().climb > 0.0 ? cv::Scalar(0, 255, 102, 255) : cv::Scalar(0, 0, 255, 255));
    HUDDisplay = "Ground Speed:  " + std::to_string(mpCNC->getHUDData().groundspeed);
    startPos.second += padding;
    drawText(cncPanel, cv::Point(startPos.first, startPos.second), HUDDisplay, 0.5, 1, cv::Scalar(238, 238, 238, 255));
    HUDDisplay = "Heading:         " + std::to_string(mpCNC->getHUDData().heading);
    startPos.second += padding;
    drawText(cncPanel, cv::Point(startPos.first, startPos.second), HUDDisplay, 0.5, 1, cv::Scalar(238, 238, 238, 255));
    HUDDisplay = "Throttle:        " + std::to_string(mpCNC->getHUDData().throttle);
    startPos.second += padding;
    drawText(cncPanel, cv::Point(startPos.first, startPos.second), HUDDisplay, 0.5, 1, cv::Scalar(238, 238, 238, 255));
    return startPos;
}

std::pair<int, int> CNCPopup::drawWPStatus(cv::Mat *cncPanel, std::pair<int, int> startPos, float padding) {
    CNC::CNCArdupilot* ardupilotPtr = dynamic_cast<CNC::CNCArdupilot*>(mpCNC);
    if (!ardupilotPtr) return startPos;
    // Draw all waypoints
    startPos.second += 2 * padding;  // Extra padding for section spacing
    drawText(cncPanel, cv::Point(startPos.first, startPos.second), "=== Waypoint List ================", 0.5, 1,
            cv::Scalar(238, 238, 238, 255));
    startPos.second += padding;
    std::string localPosDisplay = "Local Position: " + std::to_string(mpCNC->getLocalPosition().pose.position.x) + " "
            + std::to_string(mpCNC->getLocalPosition().pose.position.y) + " "
            + std::to_string(mpCNC->getLocalPosition().pose.position.z);
    drawText(cncPanel, cv::Point(startPos.first, startPos.second), localPosDisplay, 0.5, 1,
            cv::Scalar(238, 238, 238, 255));
    startPos.second += padding;
    drawText(cncPanel, cv::Point(startPos.first, startPos.second), "Idx Current Lat            Long            Alt",
            0.5, 1, cv::Scalar(238, 238, 238, 255));
    mavros_msgs::WaypointList wpList = ardupilotPtr->getWaypointList();
    int index = 0;
    for (auto wp : wpList.waypoints) {
        std::string lineStr = std::to_string(index) + "   N        ";
        if (wp.is_current) lineStr = std::to_string(index) + "   Y        ";
        lineStr += std::to_string(wp.x_lat) + "    " + std::to_string(wp.y_long) + "    " + std::to_string(wp.z_alt);
        startPos.second += padding;
        drawText(cncPanel, cv::Point(startPos.first, startPos.second), lineStr, 0.5, 1, cv::Scalar(238, 238, 238, 255));
        index++;
    }
    return startPos;
}

void CNCPopup::draw() {
    int32_t winWidth = 500;
    int32_t winHeight = 800;
    cv::Mat cncPanel(winHeight, winWidth, CV_8UC3, cv::Scalar(0, 0, 0, 255));

    drawCNCInfoPanel(&cncPanel);

    cv::imshow(OPENCV_WINDOW_NAME, cncPanel);
}

}  // namespace GUI
