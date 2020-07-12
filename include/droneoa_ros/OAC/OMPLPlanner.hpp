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
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/objectives/StateCostIntegralObjective.h>
// FCL
#include <fcl/config.h>
#include <fcl/fcl.h>
#include <fcl/geometry/collision_geometry.h>
#include <fcl/geometry/octree/octree.h>

#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>

#include <string>
#include <memory>

#include <boost/bind.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>

#include <droneoa_ros/PDN.hpp>
#ifdef ENABLE_SMOOTHER
#include <ompl/geometric/PathSimplifier.h>
#endif

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
    std::string AsString() {
        std::string result = "Position3D: x: " + std::to_string(mX) + " y: " +
                std::to_string(mY) + " z: " + std::to_string(mZ);
        return result;
    }
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
    bool getForcePlanFlag();
    void setForcePlanFlag(bool newflag);
    bool getForceReplanFlag();
    void setForceReplanFlag(bool newflag);
    // Threads Callback
    void Octomap_callback(const octomap_msgs::OctomapConstPtr& msg);
    void Click_callback(const geometry_msgs::PointStampedConstPtr& msg);
    // Helper
    static float getDistBetweenPos3D(const Position3D &pos1, const Position3D& pos2);
    // Clearance Objective
    class ClearanceObjective : public ompl::base::StateCostIntegralObjective {
     public:
        explicit ClearanceObjective(const ompl::base::SpaceInformationPtr& si) :
            ompl::base::StateCostIntegralObjective(si, true) {
        }

        ompl::base::Cost stateCost(const ompl::base::State* s) const {
            return ompl::base::Cost(1 / si_->getStateValidityChecker()->clearance(s));
        }
    };

 private:
    boost::mutex mutex_;
    boost::shared_mutex octomap_mutex_;
    boost::shared_mutex forcePlanFlag_mutex_;
    boost::shared_mutex forceReplanFlag_mutex_;
    bool mIsSolving;
    bool replan_flag;
    bool force_plan_flag;
    bool force_replan_flag;
    // OMPL space
    ompl::base::StateSpacePtr mpSpace;
    ompl::base::SpaceInformationPtr mpSpaceInfo;
    // OMPL problem
    ompl::base::ProblemDefinitionPtr mpProblem;
    // OMPL Planner
    ompl::base::PlannerPtr mpPlanner;
    // Solution
    ompl::geometric::PathGeometric *mpPathSmooth = nullptr;
    ompl::base::PathPtr mPath;
    // FCL
    fcl::CollisionObject<double> *mpTreeObj_obj;
    std::shared_ptr<fcl::CollisionGeometry<double>> mpTreeObj;
    std::shared_ptr<fcl::CollisionGeometry<double>> mpAircraftObject;
    // Helpers
    bool isStateValid(const ompl::base::State *state);
    ompl::base::OptimizationObjectivePtr getPathLengthObjWithCostToGo(const ompl::base::SpaceInformationPtr& si);
    fcl::CollisionObject<double> *getOcTreeCollisionObj();
    // Threads
    boost::thread* mpThreadWatchOctomap = nullptr;
    void RRTMainThread();
    // Path Publisher
    ros::NodeHandle n;
#ifdef ENABLE_SMOOTHER
    ros::Publisher vis_pub;
#endif
    ros::Publisher traj_pub;
};

}  // namespace OAC

#endif  // OAC_OMPL_OMPLPLANNER_HPP_  // NOLINT
