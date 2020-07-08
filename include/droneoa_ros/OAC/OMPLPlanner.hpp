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
// FCL
#include <fcl/config.h>
#include <fcl/fcl.h>
#include <fcl/geometry/collision_geometry.h>
#include <fcl/geometry/octree/octree.h>

#include <mutex>  // NOLINT
#include <memory>

#include <boost/bind.hpp>
#include <boost/thread/thread.hpp>

namespace OAC {

struct Position3D {
    Position3D() : x(0), y(0), z(0) {}
    double x;
    double y;
    double z;
};

class OMPLPlanner {
 public:
    OMPLPlanner();
    virtual ~OMPLPlanner();
    bool setStartPos(Position3D pos);
    bool setTargetPos(Position3D pos);
    bool setOctomap(octomap::OcTree* dataTree);
    bool plan();
    bool getIsSolving();
    // Threads Callback
    void Octomap_callback(const octomap_msgs::OctomapConstPtr& msg);

 private:
    std::mutex mutex_;
    bool mIsSolving;
    // OMPL space
    ompl::base::StateSpacePtr mpSpace;
    ompl::base::SpaceInformationPtr mpSpaceInfo;
    // OMPL problem
    ompl::base::ProblemDefinitionPtr mpProblem;
    // OMPL Planner
    ompl::base::PlannerPtr mpPlanner;
    // Solution
    ompl::geometric::PathGeometric *mpPathGeo;
    ompl::base::PathPtr mPath;
    // FCL
    std::shared_ptr<fcl::CollisionObject<double>> mpTreeObj;
    std::shared_ptr<fcl::CollisionObject<double>> mpAircraftObject;
    // Helpers
    bool isStateValid(const ompl::base::State *state);
    // Threads
    boost::thread* mpThreadWatchOctomap = nullptr;
    void watchOctomapThread();
};

}  // namespace OAC

#endif  // OAC_OMPL_OMPLPLANNER_HPP_  // NOLINT
