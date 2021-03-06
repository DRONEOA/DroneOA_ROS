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

// Main Idea From Reference: https://github.com/ArduPilot/OctomapPlanner

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <iostream>
#include <droneoa_ros/OAC/OMPLPlanner.hpp>

namespace OAC {

bool OMPLPlanner::updateMap(std::shared_ptr<fcl::CollisionGeometry<double>> map) {
    boost::upgrade_lock<boost::shared_mutex> lock(octomap_mutex_);
    boost::upgrade_to_unique_lock<boost::shared_mutex> uniqueLock(lock);
    mpTreeObj = map;
    fcl::CollisionObject<double> *treeObj = new fcl::CollisionObject<double>(mpTreeObj);
    if (mpTreeObj_obj) delete mpTreeObj_obj;
    mpTreeObj_obj = treeObj;
    return true;
}

fcl::CollisionObject<double> *OMPLPlanner::getOcTreeCollisionObj() {
    boost::shared_lock<boost::shared_mutex> lock(octomap_mutex_);
    return mpTreeObj_obj;
}

bool OMPLPlanner::setStartPos(LocalPoint pos) {
    ompl::base::ScopedState<ompl::base::SE3StateSpace> start(mpSpace);
    start->setXYZ(pos.mX, pos.mY, pos.mZ);
    start->as<ompl::base::SO3StateSpace::StateType>(1)->setIdentity();
    mpProblem->clearStartStates();
    mpProblem->addStartState(start);
#ifdef RRT_DEBUG
    ROS_DEBUG("[RRT] Set Start Pos: %lf %lf %lf", pos.mX, pos.mY, pos.mZ);
#endif
    return true;
}

bool OMPLPlanner::setTargetPos(LocalPoint pos) {
    ompl::base::ScopedState<ompl::base::SE3StateSpace> goal(mpSpace);
    goal->setXYZ(pos.mX, pos.mY, pos.mZ);
    goal->as<ompl::base::SO3StateSpace::StateType>(1)->setIdentity();
    mpProblem->clearGoal();
    mpProblem->setGoalState(goal);
#ifdef RRT_DEBUG
    ROS_DEBUG("[RRT] Set Goal Pos: %lf %lf %lf", pos.mX, pos.mY, pos.mZ);
#endif
    setForcePlanFlag(true);
    return true;
}

bool OMPLPlanner::getForcePlanFlag() {
    boost::shared_lock<boost::shared_mutex> lock(forcePlanFlag_mutex_);
    return force_plan_flag;
}

void OMPLPlanner::setForcePlanFlag(bool newflag) {
    boost::upgrade_lock<boost::shared_mutex> lock(forcePlanFlag_mutex_);
    boost::upgrade_to_unique_lock<boost::shared_mutex> uniqueLock(lock);
    force_plan_flag = newflag;
}

bool OMPLPlanner::getForceReplanFlag() {
    boost::shared_lock<boost::shared_mutex> lock(forceReplanFlag_mutex_);
    return force_replan_flag;
}

void OMPLPlanner::setForceReplanFlag(bool newflag) {
    boost::upgrade_lock<boost::shared_mutex> lock(forceReplanFlag_mutex_);
    boost::upgrade_to_unique_lock<boost::shared_mutex> uniqueLock(lock);
    force_replan_flag = newflag;
}

OMPLPlanner::OMPLPlanner() : mSolutionExist(true), replan_flag(false), force_replan_flag(false), mSolutionRevision(0) {
    // Collision box of the drone
    mpAircraftObject = std::shared_ptr<fcl::CollisionGeometry<double>>(new fcl::Box<double>(
            static_cast<double>(VEHICLE_BOUNDBOX_WIDTH) / 1000.0f,
            static_cast<double>(VEHICLE_BOUNDBOX_LENGTH) / 1000.0f,
            static_cast<double>(VEHICLE_BOUNDBOX_HEIGHT) / 1000.0f));
    // Octomap tree and it's resolution
    fcl::OcTree<double>* tree =
            new fcl::OcTree<double>(std::shared_ptr<const octomap::OcTree>(new octomap::OcTree(OCTOMAP_RESOLUTION)));
    mpTreeObj = std::shared_ptr<fcl::CollisionGeometry<double>>(tree);
    // State space of the solution
    mpSpace = ompl::base::StateSpacePtr(new ompl::base::SE3StateSpace());
    // create a start state
    ompl::base::ScopedState<ompl::base::SE3StateSpace> start(mpSpace);
    // create a goal state
    ompl::base::ScopedState<ompl::base::SE3StateSpace> goal(mpSpace);
    // set the bounds for the R^3 part of SE(3)
    // Boundary of the search space
    //! @todo Add a set function
    ompl::base::RealVectorBounds bounds(3);
    bounds.setLow(0, -RRT_MAX_PLANNING_DISTANCE);
    bounds.setHigh(0, RRT_MAX_PLANNING_DISTANCE);
    bounds.setLow(1, -RRT_MAX_PLANNING_DISTANCE);
    bounds.setHigh(1, RRT_MAX_PLANNING_DISTANCE);
    bounds.setLow(2, RRT_MIN_PLANNING_HEIGHT);
    bounds.setHigh(2, RRT_MAX_PLANNING_HEIGHT);
    mpSpace->as<ompl::base::SE3StateSpace>()->setBounds(bounds);
    // construct an instance of  space information from this state space
    mpSpaceInfo = ompl::base::SpaceInformationPtr(new ompl::base::SpaceInformation(mpSpace));
    start->setXYZ(0, 0, 0);
    start->as<ompl::base::SO3StateSpace::StateType>(1)->setIdentity();
    goal->setXYZ(0, -1, 1);
    goal->as<ompl::base::SO3StateSpace::StateType>(1)->setIdentity();
    // set state validity checking for this space
    mpSpaceInfo->setStateValidityChecker(std::bind(&OMPLPlanner::isStateValid, this, std::placeholders::_1));
    // create a problem instance
    mpProblem = ompl::base::ProblemDefinitionPtr(new ompl::base::ProblemDefinition(mpSpaceInfo));
    // set the start and goal states
    mpProblem->setStartAndGoalStates(start, goal);
    // set Optimizattion objective
    mpProblem->setOptimizationObjective(OMPLPlanner::getPathLengthObjWithCostToGo(mpSpaceInfo));
    // Debug Level Settinf
#if !defined(RRT_DEBUG) && !defined(RRT_DEBUG_PLANNER)
    ompl::msg::setLogLevel(ompl::msg::LOG_NONE);
#endif
    ROS_DEBUG("[RRT] Planner Initialized");

    // Init Octomap Watcher
    mpThreadWatchOctomap = new boost::thread(boost::bind(&OMPLPlanner::RRTMainThread, this));
    // Setup Publisher
#ifdef RRT_ENABLE_SMOOTHER
    vis_pub = n.advertise<nav_msgs::Path>("visualization_marker", 0);
#endif
    traj_pub = n.advertise<nav_msgs::Path>("waypoints", 1);
}

OMPLPlanner::~OMPLPlanner() {
    if (mpThreadWatchOctomap) {
        mpThreadWatchOctomap->join();
        delete mpThreadWatchOctomap;
    }
    if (mpTreeObj_obj) delete mpTreeObj_obj;
    if (mpPathSmooth) delete mpPathSmooth;
}

bool OMPLPlanner::rePlan() {
    if (!mpPathSmooth) {
        return plan();
    }
#ifdef RRT_DEBUG
    std::cout << "Total Points:" << mpPathSmooth->getStateCount() << std::endl;
#endif
    if (mpPathSmooth->getStateCount() <= 2) {
        return plan();
    } else {
        for (std::size_t idx = 0; idx < mpPathSmooth->getStateCount(); idx++) {
            if (!replan_flag) {
                replan_flag = !isStateValid(mpPathSmooth->getState(idx));
            } else {
                break;
            }
        }
        if (replan_flag) {
            return plan();
        } else {
        #ifdef RRT_DEBUG
            std::cout << "Replanning not required" << std::endl;
        #endif
        }
    }
    return true;
}

void OMPLPlanner::RRTMainThread() {
    ros::Rate rt(GLOBAL_ROS_RATE);
    auto node = boost::make_shared<ros::NodeHandle>();
    auto octomap_sub =
        node->subscribe<octomap_msgs::Octomap>("/octomap_binary", 1,
                boost::bind(&OMPLPlanner::Octomap_callback, this, _1));
    auto goal_sub =
        node->subscribe<geometry_msgs::PointStamped>("/clicked_point", 1,
                boost::bind(&OMPLPlanner::Click_callback, this, _1));

    while (ros::ok()) {
        if (getForcePlanFlag()) {
            setForcePlanFlag(false);
            setForceReplanFlag(false);
            plan();
        }
        if (getForceReplanFlag()) {
            setForceReplanFlag(false);
            rePlan();
        }
        ros::spinOnce();
        rt.sleep();
    }
}

void OMPLPlanner::Octomap_callback(const octomap_msgs::OctomapConstPtr& msg) {
    boost::lock_guard<boost::mutex> lock(mutex_);
    octomap::OcTree* tree_oct = dynamic_cast<octomap::OcTree*>(octomap_msgs::msgToMap(*msg));
    fcl::OcTree<double>* tree = new fcl::OcTree<double>(std::shared_ptr<const octomap::OcTree>(tree_oct));
    updateMap(std::shared_ptr<fcl::CollisionGeometry<double>>(tree));
    setForceReplanFlag(true);
}

void OMPLPlanner::Click_callback(const geometry_msgs::PointStampedConstPtr& msg) {
#ifdef RRT_DEBUG
    std::cout << "x: " << (*msg).point.x << " y: " << (*msg).point.y << " z: " << (*msg).point.z << std::endl;
#endif
}

bool OMPLPlanner::plan() {
    /**************************************
     * Setup Planner
     **************************************/
    // create a planner for the defined space
    ompl::geometric::InformedRRTstar* rrt = new ompl::geometric::InformedRRTstar(mpSpaceInfo);
    // RRT range (Path resolution)
    rrt->setRange(2.0);
    ompl::base::PlannerPtr plan(rrt);
    // set the problem we are trying to solve for the planner
    plan->setProblemDefinition(mpProblem);
    // perform setup steps for the planner
    plan->setup();
#ifdef RRT_DEBUG_PLANNER
    // print the settings for this space
    mpSpaceInfo->printSettings(std::cout);
    std::cout << "problem setting\n";
    // print the problem settings
    mpProblem->print(std::cout);
#endif
    /**************************************
     * Setup PlannerSolve
     **************************************/
    // attempt to solve the problem within one second of planning time
#ifdef RRT_DEBUG_PLANNER
    std::cout << "START SOLVING" << std::endl;
#endif
    ompl::base::PlannerStatus solved = plan->solve(1);
#ifdef RRT_DEBUG_PLANNER
    std::cout << "END SOLVING" << std::endl;
#endif
    if (solved) {
        /**************************************
         * Solution Found :)
         **************************************/
        // get the goal representation from the problem definition (not the same as the goal state)
        // and inquire about the found path
    #ifdef RRT_DEBUG_PLANNER
        std::cout << "Found solution:" << std::endl;
    #endif
        ompl::base::PathPtr path = mpProblem->getSolutionPath();
        ompl::geometric::PathGeometric* pth = mpProblem->getSolutionPath()->as<ompl::geometric::PathGeometric>();
        setPathCost(true, rrt->bestCost());
    #ifdef RRT_DEBUG_PLANNER
        pth->printAsMatrix(std::cout);
    #endif
        // print the path to screen
        // path->print(std::cout);
        /**************************************
         * Convert Path to nav_msgs::Path
         **************************************/
        nav_msgs::Path msg;
            msg.header.stamp = ros::Time::now();
            msg.header.frame_id = "world";
            for (std::size_t path_idx = 0; path_idx < pth->getStateCount(); path_idx++) {
                const ompl::base::SE3StateSpace::StateType *se3state =
                        pth->getState(path_idx)->as<ompl::base::SE3StateSpace::StateType>();
                // extract the first component of the state and cast it to what we expect
                const ompl::base::RealVectorStateSpace::StateType *pos =
                        se3state->as<ompl::base::RealVectorStateSpace::StateType>(0);
                // extract the second component of the state and cast it to what we expect
                const ompl::base::SO3StateSpace::StateType *rot =
                        se3state->as<ompl::base::SO3StateSpace::StateType>(1);
                geometry_msgs::PoseStamped pose;
                // pose.header.frame_id = "/world" // @todo ???
                pose.pose.position.x = pos->values[0];
                pose.pose.position.y = pos->values[1];
                pose.pose.position.z = pos->values[2];
                pose.pose.orientation.x = rot->x;
                pose.pose.orientation.y = rot->y;
                pose.pose.orientation.z = rot->z;
                pose.pose.orientation.w = rot->w;
                msg.poses.push_back(pose);
            }
            traj_pub.publish(msg);
            /**************************************
             * Path smoothing using bspline
             **************************************/
            // Optimize / Smooth the path
        #ifdef RRT_ENABLE_SMOOTHER
            ompl::geometric::PathSimplifier* pathBSpline = new ompl::geometric::PathSimplifier(mpSpaceInfo);
        #endif
            setNewPathAndRevision(new ompl::geometric::PathGeometric(
                    dynamic_cast<const ompl::geometric::PathGeometric&>(*mpProblem->getSolutionPath())));
        #ifdef RRT_ENABLE_SMOOTHER
            pathBSpline->smoothBSpline(*mpPathSmooth, 3);
            // Publish path as markers
            nav_msgs::Path smooth_msg;
            smooth_msg.header.stamp = ros::Time::now();
            smooth_msg.header.frame_id = "world";
            for (std::size_t idx = 0; idx < mpPathSmooth->getStateCount(); idx++) {
                // cast the abstract state type to the type we expect
                const ompl::base::SE3StateSpace::StateType *se3state =
                        mpPathSmooth->getState(idx)->as<ompl::base::SE3StateSpace::StateType>();
                // extract the first component of the state and cast it to what we expect
                const ompl::base::RealVectorStateSpace::StateType *pos =
                        se3state->as<ompl::base::RealVectorStateSpace::StateType>(0);
                // extract the second component of the state and cast it to what we expect
                const ompl::base::SO3StateSpace::StateType *rot =
                        se3state->as<ompl::base::SO3StateSpace::StateType>(1);
                geometry_msgs::PoseStamped point;
                // pose.header.frame_id = "/world" // @todo ???
                point.pose.position.x = pos->values[0];
                point.pose.position.y = pos->values[1];
                point.pose.position.z = pos->values[2];
                point.pose.orientation.x = rot->x;
                point.pose.orientation.y = rot->y;
                point.pose.orientation.z = rot->z;
                point.pose.orientation.w = rot->w;
                smooth_msg.poses.push_back(point);
            #ifdef RRT_DEBUG_PLANNER
                std::cout << "Published marker: " << idx << std::endl;
            #endif
            }
            vis_pub.publish(smooth_msg);
            delete pathBSpline;
        #endif
            // ros::Duration(0.1).sleep();
            // Clear memory
            mpProblem->clearSolutionPaths();
            replan_flag = false;
    } else {
        /**************************************
         * NO Solution Found :(
         **************************************/
    #ifdef RRT_DEBUG_NOSOLUTION
        ROS_ERROR("[RRT] No solution found");
    #endif
        setPathCost(false, ompl::base::Cost(std::numeric_limits<double>::infinity()));
        return false;
    }
    return true;
}

bool OMPLPlanner::isStateValid(const ompl::base::State *state) {
    // cast the abstract state type to the type we expect
    const ompl::base::SE3StateSpace::StateType *se3state = state->as<ompl::base::SE3StateSpace::StateType>();
    // extract the first component of the state and cast it to what we expect
    const ompl::base::RealVectorStateSpace::StateType *pos =
            se3state->as<ompl::base::RealVectorStateSpace::StateType>(0);
    // extract the second component of the state and cast it to what we expect
    const ompl::base::SO3StateSpace::StateType *rot = se3state->as<ompl::base::SO3StateSpace::StateType>(1);


    fcl::CollisionObject<double> aircraftObject(mpAircraftObject);
    // check validity of state defined by pos & rot
    fcl::Vector3<double> translation(pos->values[0], pos->values[1], pos->values[2]);
    // std::cout << "[DEBUG] " << translation << std::endl;
    fcl::Quaternion<double> rotation(rot->w, rot->x, rot->y, rot->z);
    aircraftObject.setQuatRotation(rotation);
    aircraftObject.setTranslation(translation);
    fcl::CollisionRequest<double> requestType(1, false, 1, false);
    fcl::CollisionResult<double> collisionResult;
    fcl::collide(&aircraftObject, getOcTreeCollisionObj(), requestType, collisionResult);
    return(!collisionResult.isCollision());
}

ompl::base::OptimizationObjectivePtr OMPLPlanner::getPathLengthObjWithCostToGo(
        const ompl::base::SpaceInformationPtr& si) {
    //! @todo Need a balanced Optimization Objective
    // // Option 1 Bug?
    // ompl::base::OptimizationObjectivePtr lengthObj(new ompl::base::PathLengthOptimizationObjective(si));
    // ompl::base::OptimizationObjectivePtr clearObj(new ClearanceObjective(si));
    // return 10.0*lengthObj + clearObj;
    // Option 2
    ompl::base::OptimizationObjectivePtr obj(new ompl::base::PathLengthOptimizationObjective(si));
    obj->setCostThreshold(ompl::base::Cost(1.51));
    return obj;
    // // Option 3
    // ompl::base::OptimizationObjectivePtr obj(new ompl::base::PathLengthOptimizationObjective(si));
    // obj->setCostToGoHeuristic(&ompl::base::goalRegionCostToGo);
    // return obj;
}

int32_t OMPLPlanner::getPathAndRevision(std::vector<LocalPoint> *results) {
    //! @todo do we need read write lock for mpPathSmooth?
    //! @todo consider using callback
    boost::shared_lock<boost::shared_mutex> lock(solution_revision_mutex_);
    if (!mpPathSmooth) {
        return mSolutionRevision;
    }
    for (std::size_t idx = 0; idx < mpPathSmooth->getStateCount(); idx++) {
        // cast the abstract state type to the type we expect
        const ompl::base::SE3StateSpace::StateType *se3state =
                mpPathSmooth->getState(idx)->as<ompl::base::SE3StateSpace::StateType>();
        // extract the first component of the state and cast it to what we expect
        const ompl::base::RealVectorStateSpace::StateType *pos =
                se3state->as<ompl::base::RealVectorStateSpace::StateType>(0);
        results->push_back(LocalPoint(pos->values[0], pos->values[1], pos->values[2]));
    }
    // for (auto wp : *results) {
    //     std::cout << wp.AsString() << std::endl;
    // }
    return mSolutionRevision;
}

void OMPLPlanner::setNewPathAndRevision(ompl::geometric::PathGeometric *newPath) {
    boost::upgrade_lock<boost::shared_mutex> lock(solution_revision_mutex_);
    boost::upgrade_to_unique_lock<boost::shared_mutex> uniqueLock(lock);
    if (mpPathSmooth) delete mpPathSmooth;
    mpPathSmooth = newPath;
    mSolutionRevision++;
}

double OMPLPlanner::getPathCost() {
    boost::shared_lock<boost::shared_mutex> lock(solution_mutex_);
    return mCostOfPath.value();
}

bool OMPLPlanner::isSolutionExist() {
    boost::shared_lock<boost::shared_mutex> lock(solution_mutex_);
    return mSolutionExist;
}

void OMPLPlanner::setPathCost(bool pathExist, ompl::base::Cost cost) {
    boost::upgrade_lock<boost::shared_mutex> lock(solution_mutex_);
    boost::upgrade_to_unique_lock<boost::shared_mutex> uniqueLock(lock);
    mSolutionExist = pathExist;
    mCostOfPath = cost;
}

}  // namespace OAC
