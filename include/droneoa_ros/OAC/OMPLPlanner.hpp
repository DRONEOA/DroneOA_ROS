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
#include <vector>

#include <boost/bind.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>

#include <droneoa_ros/HWI/Utils/LocalPoint.hpp>
#include <droneoa_ros/PDN.hpp>

#ifdef RRT_ENABLE_SMOOTHER
#include <ompl/geometric/PathSimplifier.h>
#endif

// #define RRT_DEBUG
// #define RRT_DEBUG_PLANNER
// #define RRT_DEBUG_NOSOLUTION

namespace OAC {

class OMPLPlanner {
 public:
    OMPLPlanner();
    virtual ~OMPLPlanner();
    /**
     * Note: Depth Camera is facing -y axis
     */ 
    /**
     * @brief Set the Start Position
     * This will NOT trigger a replan
     * @param pos The new start position
     * @return true  
     */
    bool setStartPos(LocalPoint pos);
    /**
     * @brief Set the Target Position
     * This will set the force plan flag and trigger a new path planning with the same new goal position.
     * @param pos The new goal position 
     * @return true 
     */
    bool setTargetPos(LocalPoint pos);
    /**
     * @brief Update the CollisionGeometry of the environment
     * Update the CollisionGeometry of the octomap that is used to validate OMPL state. (Descide whether collide)
     * @param map std::shared_ptr<fcl::CollisionGeometry<double>> The new collisionGeometry of the environment
     * @return true 
     */
    bool updateMap(std::shared_ptr<fcl::CollisionGeometry<double>> map);
    /**
     * @brief Make a path plan based on current setting
     * @return whether a path is found
     */
    bool plan();
    /**
     * @brief Replan if the environment is change and blocked the path. The the path is running out.
     * @return true if noe need to plan OR new plan is performed and a path is found
     * @return false new plan is performed and a path is NOT found
     */
    bool rePlan();
    bool getIsSolving();
    bool getForcePlanFlag();
    /**
     * @brief Request a full plan without checking whether a replan is required
     * @param newflag New flag boolean value
     */
    void setForcePlanFlag(bool newflag);
    bool getForceReplanFlag();
    /**
     * @brief Request a replan, whether a replan is required will be checked
     * @param newflag New flag boolean value
     */
    void setForceReplanFlag(bool newflag);
    /**
     * @brief Get the Path And Revision Number of the Path
     * @param results the results container will carry the resulting path out. Each waypoint is a LocalPoint.
     * @return int32_t revision number of the path (increament on each success plan action)
     */
    int32_t getPathAndRevision(std::vector<LocalPoint> *results);
    double getPathCost();
    bool isSolutionExist();
    // Threads Callback
    void Octomap_callback(const octomap_msgs::OctomapConstPtr& msg);
    void Click_callback(const geometry_msgs::PointStampedConstPtr& msg);
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
    boost::shared_mutex solution_mutex_;
    boost::shared_mutex solution_revision_mutex_;
    bool mSolutionExist;
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
    int32_t mSolutionRevision;
    ompl::geometric::PathGeometric *mpPathSmooth = nullptr;
    ompl::base::PathPtr mPath;
    ompl::base::Cost mCostOfPath;
    // FCL
    fcl::CollisionObject<double> *mpTreeObj_obj;
    std::shared_ptr<fcl::CollisionGeometry<double>> mpTreeObj;
    std::shared_ptr<fcl::CollisionGeometry<double>> mpAircraftObject;
    // Helpers
    bool isStateValid(const ompl::base::State *state);
    ompl::base::OptimizationObjectivePtr getPathLengthObjWithCostToGo(const ompl::base::SpaceInformationPtr& si);
    fcl::CollisionObject<double> *getOcTreeCollisionObj();
    void setPathCost(bool pathExist, ompl::base::Cost cost);
    void setNewPathAndRevision(ompl::geometric::PathGeometric *newPath);
    // Threads
    boost::thread* mpThreadWatchOctomap = nullptr;
    void RRTMainThread();
    // Path Publisher
    ros::NodeHandle n;
#ifdef RRT_ENABLE_SMOOTHER
    ros::Publisher vis_pub;
#endif
    ros::Publisher traj_pub;
};

}  // namespace OAC

#endif  // OAC_OMPL_OMPLPLANNER_HPP_  // NOLINT
