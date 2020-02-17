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

#ifndef INCLUDE_DRONEOA_ROS_RSCINTERFACE_HPP_  // NOLINT
#define INCLUDE_DRONEOA_ROS_RSCINTERFACE_HPP_  // NOLINT

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/transforms.h>
#include <string>
#include <vector>
#include <opencv2/core/core.hpp>
#include <droneoa_ros/PDN.hpp>

#include <boost/bind.hpp>
#include <boost/thread/thread.hpp>


#define ENABLE_POINTCLOUD
// #define PCL_DEBUG_VIEWER
// #define IMG_DEBUG_POPUP

static const char* DEPTH_SOURCE_RSC = "/d435/depth/image_rect_raw";
static const char* DEPTH_SOURCE_UE4 = "/unreal_ros/image_depth";
static const char* PC_SOURCE_RSC = "/d435/depth/color/points";
static const char* PC_SOURCE_UE4 = "/depth_registered/points";

class RSCInterface {
 public:
    RSCInterface();
    virtual ~RSCInterface();
    void init(ros::NodeHandle nh, ros::Rate r);

    cv::Mat depthImgForDesiredDistanceRange(float min, float max, cv::Mat input);
    void setRangeSwitch(bool status);
    void setRange(float min, float max);

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

    //! @todo(Xiao Zhou): The minimum distance, left for possibility calculation in the future
    std::vector<float> pointCloudZCoordsInRange(
        float width = VEHICLE_BOUNDBOX_WIDTH,
        float height = VEHICLE_BOUNDBOX_HEIGHT,
        float dist = 200.0f);

    // Callback
    void depthImg_callback(const sensor_msgs::ImageConstPtr& msg);
    void pointcloud_callback(const sensor_msgs::PointCloud2ConstPtr& msg);

    // Debug Print
    void printImgInfo();
    static void mouseCallback(int event, int x, int y, int flags, void* userdata);
    pcl::PointCloud<pcl::PointXYZRGB> getPointCloud();

 private:
    ros::NodeHandle n;
    ros::Rate r_ = ros::Rate(10.0);
    bool rangeSwitch = false;

    // Data
    std::string currentDepthSource_;
    std::string currentPCSource_;
    sensor_msgs::Image depthImage_;
    sensor_msgs::PointCloud2 pointCloud_;
    pcl::PointCloud<pcl::PointXYZRGB> pcl_pointCloud_;
    cv::Mat depthFrame_;
    float rangeMin;
    float rangeMax;

    // Threads
    boost::thread* thread_watch_depth_img_ = nullptr;
    boost::thread* thread_watch_pointcloud_ = nullptr;
    void watchDepthImgThread();
    void watchPointCloudThread();
    boost::shared_mutex depth_img_mutex;
    boost::shared_mutex pointcloud_mutex;

    // Subscriber
    ros::Subscriber depth_sub_;
    ros::Subscriber pc2_sub_;

    // Debug
    void drawDebugOverlay();
    static cv::Point debugMousePos;
#ifdef PCL_DEBUG_VIEWER
    pcl::visualization::CloudViewer *viewer;
    void updatePointCloudViewerThread();
    boost::thread* thread_pointcloud_viewer_ = nullptr;
#endif
};

#endif  // INCLUDE_DRONEOA_ROS_RSCINTERFACE_HPP_  // NOLINT
