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
 * Written by Bohan Shi <b34shi@edu.uwaterloo.ca>, August 2019
 */

#ifndef HWI_RSC_HPP_  // NOLINT
#define HWI_RSC_HPP_  // NOLINT

#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/transforms.h>

#include <string>
#include <vector>

#include <boost/bind.hpp>
#include <boost/thread/thread.hpp>
#include <opencv2/core/core.hpp>

#include <droneoa_ros/PDN.hpp>
#include <droneoa_ros/HWI/interface/DepthCamInterface.hpp>

namespace Depth {

class RSC : public DepthCamInterface {
 public:
    RSC(ros::NodeHandle node, ros::Rate rate);
    virtual ~RSC();

    /***************************************************************************
     * Init
     */
    void initWatcherThread() override;
    /**
     * @brief Change depth data source [USE WITH CAUTION]
     * @param depthSource string of the new depth data source
     */
    void changeDepthSource(std::string depthSource);
    /**
     * @brief Change pointcloud2 data source [USE WITH CAUTION]
     * @param pc2Source string of the new pointcloud2 data source
     */
    void changePC2Source(std::string pc2Source);

    /***************************************************************************
     * Range operation
     */
    cv::Mat depthImgForDesiredDistanceRange(float min, float max, cv::Mat input);
    void setRangeSwitch(bool status);
    void setRange(float min, float max);

    //! @todo(Xiao Zhou): The minimum distance, left for possibility calculation in the future
    std::vector<float> pointCloudZCoordsInRange(
        float width = VEHICLE_BOUNDBOX_WIDTH,
        float height = VEHICLE_BOUNDBOX_HEIGHT,
        float dist = 200.0f);

    /***************************************************************************
     * Accessor
     */
    float getMaxRange() override;  /*!< Unit: m */
    float getMinRange() override;  /*!< Unit: m */
    sensor_msgs::Image getDepthImage() override;
    sensor_msgs::PointCloud2 getPC2Cloud() override;

    /***************************************************************************
     * Callback
     */
    void depthImg_callback(const sensor_msgs::ImageConstPtr& msg);
    void pointcloud_callback(const sensor_msgs::PointCloud2ConstPtr& msg);

    /***************************************************************************
     * Debug
     */
    void printImgInfo() override;
    static void mouseCallback(int32_t event, int32_t x, int32_t y, int32_t flags, void* userdata);
    static void drawText(cv::Mat targetImg, cv::Point origin, std::string text, double font_scale = 1,
            int32_t thickness = 1);
    static cv::Mat getBetterImageDebug(cv::Mat input);

 private:
    ros::NodeHandle mNodeHandle;
    ros::Rate mRate = ros::Rate(GLOBAL_ROS_RATE);
    bool mRangeSwitch = false;

    /***************************************************************************
     * Data
     */
    std::string mCurrentDepthSource;
    std::string mCurrentPCSource;
    sensor_msgs::Image mDepthImage;
    sensor_msgs::PointCloud2 mPointCloud;
    pcl::PointCloud<pcl::PointXYZRGB> mPclPointCloud;
    cv::Mat mDepthFrame;
    float mRangeMin;
    float mRangeMax;

    /***************************************************************************
     * Threads
     */
    boost::thread* mpThreadWatchDepthImg = nullptr;
    boost::thread* mpThreadWatchPointcloud = nullptr;
    void watchDepthImgThread();
    void watchPointCloudThread();
    boost::shared_mutex mDepthImgMutex;
    boost::shared_mutex mPointcloudMutex;

    // Subscriber
    ros::Subscriber mDepthSub;
    ros::Subscriber mPC2Sub;

    // Debug
    void drawDebugOverlay();
    static cv::Point mDebugMousePos;
#ifdef PCL_DEBUG_VIEWER
    pcl::visualization::CloudViewer *viewer;
    void updatePointCloudViewerThread();
    boost::thread* thread_pointcloud_viewer_ = nullptr;
#endif
};

}  // namespace Depth

#endif  // HWI_RSC_HPP_  // NOLINT
