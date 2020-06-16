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

#ifndef GUI_DEBUG_RSCPOPUP_  // NOLINT
#define GUI_DEBUG_RSCPOPUP_  // NOLINT

#include <pcl/visualization/cloud_viewer.h>

#include <string>

#include <boost/bind.hpp>
#include <boost/thread/thread.hpp>
#include <opencv2/core/core.hpp>

namespace Depth {

class RSC;

class RSCPopup {
    bool mIsViewerEnabled;
    RSC* mpRSC;
    std::string OPENCV_WINDOW_RSC = "";
    // Data
    pcl::PointCloud<pcl::PointXYZRGB> mPclPointCloud;
    cv::Mat mDepthFrame;
    // Popup
    void drawDebugOverlay();
    static cv::Point mDebugMousePos;
    // Viewer
    pcl::visualization::CloudViewer *viewer;
    void updatePointCloudViewerThread();
    boost::thread* thread_pointcloud_viewer_ = nullptr;

 public:
    RSCPopup(std::string windowName, RSC* rsc, bool enableViewer = false);
    ~RSCPopup();
    void UpdateView();

    static void mouseCallback(int32_t event, int32_t x, int32_t y, int32_t flags, void* userdata);
    static void drawText(cv::Mat targetImg, cv::Point origin, std::string text, double font_scale = 1,
            int32_t thickness = 1);  //! @todo move to GUI utility when we have it
    static cv::Mat getBetterImageDebug(cv::Mat input);
};

}  // namespace Depth

#endif  // GUI_DEBUG_RSCPOPUP_  // NOLINT
