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
 * Written by Bohan Shi <b34shi@edu.uwaterloo.ca>, August 2019
 */

#ifndef OAC_OMPL_OMPLPLANNER_HPP_  // NOLINT
#define OAC_OMPL_OMPLPLANNER_HPP_  // NOLINT

// Octomap
#include <octomap/OcTree.h>
#include <octomap/octomap.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
// OMPL
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>
#include <ompl/geometric/PathSimplifier.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
// FCL
#include <fcl/config.h>
#include <fcl/fcl.h>
#include <fcl/geometry/collision_geometry.h>
#include <fcl/geometry/octree/octree.h>

#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>

#include <mutex>  // NOLINT
#include <memory>

#include <boost/bind.hpp>
#include <boost/thread/thread.hpp>

// #define RRT_DEBUG
// #define RRT_DEBUG_PLANNER
// #define RRT_DEBUG_NOSOLUTION

namespace OAC {

struct Position3D {
    Position3D() : mX(0), mY(0), mZ(0) {}
    Position3D(double x, double y, double z) : mX(x), mY(y), mZ(z) {}
    double mX;
    double mY;
    double mZ;
};

class OMPLPlanner {
 public:
    OMPLPlanner();
    virtual ~OMPLPlanner();
    bool setStartPos(Position3D pos);
    /**
     * Note: Depth Camera is facing -y axis
     */ 
    bool setTargetPos(Position3D pos);
    bool updateMap(std::shared_ptr<fcl::CollisionGeometry<double>> map);
    bool plan();
    bool rePlan();
    bool getIsSolving();
    // Threads Callback
    void Octomap_callback(const octomap_msgs::OctomapConstPtr& msg);
    void Click_callback(const geometry_msgs::PointStampedConstPtr& msg);

 private:
    std::mutex mutex_;
    bool mIsSolving;
    bool replan_flag;
    // OMPL space
    ompl::base::StateSpacePtr mpSpace;
    ompl::base::SpaceInformationPtr mpSpaceInfo;
    // OMPL problem
    ompl::base::ProblemDefinitionPtr mpProblem;
    // OMPL Planner
    ompl::base::PlannerPtr mpPlanner;
    // Solution
    ompl::geometric::PathGeometric *mpPathGeo;
    ompl::geometric::PathGeometric *mpPathSmooth = nullptr;
    ompl::base::PathPtr mPath;
    // FCL
    fcl::CollisionObject<double> *mpTreeObj_obj;
    std::shared_ptr<fcl::CollisionGeometry<double>> mpTreeObj;
    std::shared_ptr<fcl::CollisionGeometry<double>> mpAircraftObject;
    // Helpers
    bool isStateValid(const ompl::base::State *state);
    ompl::base::OptimizationObjectivePtr getPathLengthObjWithCostToGo(const ompl::base::SpaceInformationPtr& si);
    // Threads
    boost::thread* mpThreadWatchOctomap = nullptr;
    void watchOctomapThread();
    // Path Publisher
    ros::NodeHandle n;
    ros::Publisher vis_pub;
    ros::Publisher traj_pub;
};

}  // namespace OAC

#endif  // OAC_OMPL_OMPLPLANNER_HPP_  // NOLINT
