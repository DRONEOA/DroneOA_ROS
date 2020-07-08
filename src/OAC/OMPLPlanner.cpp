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
 */

// Reference: https://github.com/ArduPilot/OctomapPlanner

#include <ros/ros.h>
#include <droneoa_ros/OAC/OMPLPlanner.hpp>
#include <droneoa_ros/PDN.hpp>

namespace OAC {

OMPLPlanner::OMPLPlanner() : mIsSolving(false) {
    // Aircraft collision body
    mpAircraftObject = std::make_shared<fcl::CollisionObject<double>>(std::shared_ptr<fcl::CollisionGeometry<double>>(
            new fcl::Box<double>(1.5, 1.5, 1.5)));
    // Init Space
    mpSpace = ompl::base::StateSpacePtr(new ompl::base::RealVectorStateSpace(3));
    ompl::base::ScopedState<ompl::base::RealVectorStateSpace> start(mpSpace);
    start->values[0] = 0;
    start->values[1] = 0;
    start->values[2] = 0;
    ompl::base::ScopedState<ompl::base::RealVectorStateSpace> goal(mpSpace);
    goal->values[0] = 0;
    goal->values[1] = 0;
    goal->values[2] = 0;
    // Init Bound
    ompl::base::RealVectorBounds bounds(3);
    bounds.setLow(0, -10);
    bounds.setHigh(0, 10);
    bounds.setLow(1, -10);
    bounds.setHigh(1,  10);
    bounds.setLow(2, 0.5);
    bounds.setHigh(2, 3.5);
    mpSpace->as<ompl::base::RealVectorStateSpace>()->setBounds(bounds);
    // Init SPace Info
    mpSpaceInfo = ompl::base::SpaceInformationPtr(new ompl::base::SpaceInformation(mpSpace));
    mpSpaceInfo->setStateValidityChecker(std::bind(&OMPLPlanner::isStateValid, this, std::placeholders::_1));
    // Init Problem
    mpProblem = ompl::base::ProblemDefinitionPtr(new ompl::base::ProblemDefinition(mpSpaceInfo));
    mpProblem->setStartAndGoalStates(start, goal);
    // mpProblem->setOptimizationObjective(Planner::getPathLengthObjWithCostToGo(mpSpaceInfo));
    // Init Planner
    mpPlanner = ompl::base::PlannerPtr(new ompl::geometric::InformedRRTstar(mpSpaceInfo));
    mpPlanner->setProblemDefinition(mpProblem);
    mpPlanner->setup();

    mpThreadWatchOctomap = new boost::thread(boost::bind(&OMPLPlanner::watchOctomapThread, this));
}

OMPLPlanner::~OMPLPlanner() {
    if (mpThreadWatchOctomap) {
        mpThreadWatchOctomap->join();
        delete mpThreadWatchOctomap;
    }
}

void OMPLPlanner::watchOctomapThread() {
    ros::Rate rt(GLOBAL_ROS_RATE);
    auto node = boost::make_shared<ros::NodeHandle>();
    auto octomap_sub =
        node->subscribe<octomap_msgs::Octomap>("/octomap_full", 1,
                boost::bind(&OMPLPlanner::Octomap_callback, this, _1));
    while (ros::ok()) {
        ros::spinOnce();
        rt.sleep();
    }
}

void OMPLPlanner::Octomap_callback(const octomap_msgs::OctomapConstPtr& msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    octomap::AbstractOcTree* tree = octomap_msgs::msgToMap(*msg);
    setOctomap(dynamic_cast<octomap::OcTree*>(tree));
}

bool OMPLPlanner::getIsSolving() {
    return mIsSolving;
}

bool OMPLPlanner::setOctomap(octomap::OcTree* dataTree) {
    //! @todo save data to local
    //! @todo post process
    fcl::OcTree<double>* tree = new fcl::OcTree<double>(std::shared_ptr<const octomap::OcTree>(dataTree));
    std::shared_ptr<fcl::CollisionGeometry<double>> tmpTree = std::shared_ptr<fcl::CollisionGeometry<double>>(tree);
    mpTreeObj = std::make_shared<fcl::CollisionObject<double>>(tmpTree);
    return true;
}

bool OMPLPlanner::setStartPos(Position3D pos) {
    ompl::base::ScopedState<ompl::base::RealVectorStateSpace> start(mpSpace);
    start->values[0] = pos.x;
    start->values[1] = pos.y;
    start->values[2] = pos.z;
    ompl::base::State *state = mpSpace->allocState();
    state->as<ompl::base::RealVectorStateSpace::StateType>()->values = start->values;
    if (isStateValid(state)) {
        mpProblem->clearStartStates();
        mpProblem->addStartState(start);
        return true;
    } else {
        return false;
    }
}

bool OMPLPlanner::setTargetPos(Position3D pos) {
    ompl::base::ScopedState<ompl::base::RealVectorStateSpace> target(mpSpace);
    target->values[0] = pos.x;
    target->values[1] = pos.y;
    target->values[2] = pos.z;
    mpProblem->clearGoal();
    mpProblem->setGoalState(target);
    ompl::base::State *state = mpSpace->allocState();
    state->as<ompl::base::RealVectorStateSpace::StateType>()->values = target->values;
    if (isStateValid(state)) {
        return true;
    } else {
        return false;
    }
}

bool OMPLPlanner::plan() {
    // Attempt 4 secs
    mIsSolving = true;
    ompl::base::PlannerStatus solved = mpPlanner->solve(4);
    if (solved) {
        ompl::base::PathPtr path = mpProblem->getSolutionPath();
        mpPathGeo = mpProblem->getSolutionPath()->as<ompl::geometric::PathGeometric>();
        mpPathGeo->printAsMatrix(std::cout);
        mIsSolving = false;
        return true;
    }
    mIsSolving = false;
    return false;
}

bool OMPLPlanner::isStateValid(const ompl::base::State *state) {
    const ompl::base::RealVectorStateSpace::StateType *pos = state->as<ompl::base::RealVectorStateSpace::StateType>();

    //! @todo Check whether will collide
    // called on each grid check, so be fast :)
    fcl::Vector3<double> translation(pos->values[0], pos->values[1], pos->values[2]);
    mpAircraftObject->setTranslation(translation);
    fcl::CollisionRequest<double> requestType(1, false, 1, false);
    fcl::CollisionResult<double> collisionResult;
    fcl::collide(mpAircraftObject.get(), mpTreeObj.get(), requestType, collisionResult);

    return (!collisionResult.isCollision());
}

}  // namespace OAC
