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

#include <droneoa_ros/GUI/Debug/RSCPopup.hpp>
#include <droneoa_ros/HWI/RSC.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace GUI {

cv::Point RSCPopup::mDebugMousePos = cv::Point(0, 0);

RSCPopup::RSCPopup(std::string windowName, Depth::RSC* rsc, bool enableViewer) : GUISubscriber(windowName) {
    mpRSC = rsc;
    mIsViewerEnabled = enableViewer;
    cv::startWindowThread();  // DEBUG
    cv::setMouseCallback(OPENCV_WINDOW_NAME, RSCPopup::mouseCallback, NULL);  // DEBUG
    cv::namedWindow(OPENCV_WINDOW_NAME);
    if (mIsViewerEnabled) {
        viewer = new pcl::visualization::CloudViewer("Depth Cloud Viewer");
        if (!thread_pointcloud_viewer_) {
            thread_pointcloud_viewer_ =
                new boost::thread(boost::bind(&RSCPopup::updatePointCloudViewerThread, this));
        }
    }
    mpRSC->GUI::GUISubject::registerGUIPopup(this);
}

RSCPopup::~RSCPopup() {
    if (viewer) {
        delete viewer;
    }
    try {
        cv::destroyWindow(OPENCV_WINDOW_NAME);
    } catch(...) {
        ROS_DEBUG("[RSCPopup] cv::destroyWindow exception");
    }
    if (thread_pointcloud_viewer_) {
        delete thread_pointcloud_viewer_;
    }
}

void RSCPopup::UpdateView(GUISubject *subject) {
    if (subject && dynamic_cast<Depth::DepthCamInterface*>(subject))
    mPclPointCloud = mpRSC->getPCLCloud();
    mDepthFrame = mpRSC->getDepthCVFrame();
    drawDebugOverlay();
}

void RSCPopup::drawDebugOverlay() {
    if (cv::countNonZero(mDepthFrame) < 1) {
        ROS_DEBUG("[RSCPopup] Empty Image Ignored");
        return;
    }

    float centerDist = mDepthFrame.at<float>(mDebugMousePos.y, mDebugMousePos.x);  // Note: row, col order
    std::string centerDistStr = std::to_string(centerDist) + " mm";

    cv::Mat debugImage255;
    std::pair<float, float> range = mpRSC->getRangeFilterSetting();
    if (range.first >= 0) {
        cv::Mat rangedDebugImage = mpRSC->depthImgForDesiredDistanceRange(range.first, range.second, mDepthFrame);
        debugImage255 = getBetterImageDebug(rangedDebugImage);
    } else {
        debugImage255 = getBetterImageDebug(mDepthFrame);
    }

    drawText(debugImage255, cv::Point(20, 20), "Debug Overlay:", 0.5, 1);
    drawText(debugImage255, cv::Point(20, 40), centerDistStr, 0.5, 1);
    cv::line(debugImage255, cv::Point(mDebugMousePos.x - 7, mDebugMousePos.y - 7),
            cv::Point(mDebugMousePos.x + 7, mDebugMousePos.y + 7), cv::Scalar(0xffff), 2);
    cv::line(debugImage255, cv::Point(mDebugMousePos.x - 7, mDebugMousePos.y + 7),
            cv::Point(mDebugMousePos.x + 7, mDebugMousePos.y - 7), cv::Scalar(0xffff), 2);
    cv::imshow(OPENCV_WINDOW_NAME, debugImage255);
}

void RSCPopup::updatePointCloudViewerThread() {
    // boost::shared_lock<boost::shared_mutex> lock(mPointcloudMutex);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(&mPclPointCloud);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
    // PCL Viewer
    while (!viewer->wasStopped()) {
        viewer->showCloud(cloud);
        boost::this_thread::sleep_for(boost::chrono::milliseconds(200));  //! @todo maybe longer interval
    }
}

void RSCPopup::mouseCallback(int32_t event, int32_t x, int32_t y, int32_t flags, void* userdata) {
    if (event == cv::EVENT_LBUTTONDOWN) {
        // std::cout << "Left button of the mouse is clicked - position (" << x << ", " << y << ")" << std::endl;
    } else if (event == cv::EVENT_RBUTTONDOWN) {
        // std::cout << "Right button of the mouse is clicked - position (" << x << ", " << y << ")" << std::endl;
    } else if (event == cv::EVENT_MBUTTONDOWN) {
        // std::cout << "Middle button of the mouse is clicked - position (" << x << ", " << y << ")" << std::endl;
    } else if (event == cv::EVENT_MOUSEMOVE) {
        mDebugMousePos.x = x;
        mDebugMousePos.y = y;
    }
}

void RSCPopup::drawText(cv::Mat targetImg, cv::Point origin, std::string text, double font_scale, int32_t thickness) {
    int32_t font_face = cv::FONT_HERSHEY_COMPLEX;
    int32_t baseline;
    cv::Size text_size = cv::getTextSize(text, font_face, font_scale, thickness, &baseline);
    cv::Point originTop;
    originTop.x = origin.x;
    originTop.y = origin.y - text_size.height;
    cv::Point pt2;
    pt2.x = originTop.x + text_size.width;
    pt2.y = origin.y;
    cv::rectangle(targetImg, originTop, pt2, cv::Scalar(0), -1, cv::LineTypes::LINE_AA);
    cv::putText(targetImg, text, origin, font_face, font_scale, cv::Scalar(0xffff), thickness, 8, 0);
}

// Warning: do not use the ouput as depth data
cv::Mat RSCPopup::getBetterImageDebug(cv::Mat input) {
    double min = 0;
    double max = 12000;
    cv::minMaxIdx(input, &min, &max);
    cv::Mat adjMap;
    cv::convertScaleAbs(input, adjMap, 255 / max);
    return adjMap;
}

}  // namespace GUI
