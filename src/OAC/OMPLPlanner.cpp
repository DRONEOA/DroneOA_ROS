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
#include <droneoa_ros/PDN.hpp>


namespace OAC {

bool OMPLPlanner::updateMap(std::shared_ptr<fcl::CollisionGeometry<double>> map) {
    mpTreeObj = map;
    fcl::CollisionObject<double> *treeObj = new fcl::CollisionObject<double>(mpTreeObj);
    mpTreeObj_obj = treeObj;
    return true;
}

bool OMPLPlanner::setStartPos(Position3D pos) {
    ompl::base::ScopedState<ompl::base::SE3StateSpace> start(mpSpace);
    start->setXYZ(pos.mX, pos.mY, pos.mZ);
    start->as<ompl::base::SO3StateSpace::StateType>(1)->setIdentity();
    mpProblem->clearStartStates();
    mpProblem->addStartState(start);
    ROS_WARN("[RRT] Set Start Pos: %lf %lf %lf", pos.mX, pos.mY, pos.mZ);
    return true;  //! @todo do we need replan?
}

bool OMPLPlanner::setTargetPos(Position3D pos) {
    ompl::base::ScopedState<ompl::base::SE3StateSpace> goal(mpSpace);
    goal->setXYZ(pos.mX, pos.mY, pos.mZ);
    goal->as<ompl::base::SO3StateSpace::StateType>(1)->setIdentity();
    mpProblem->clearGoal();
    mpProblem->setGoalState(goal);
    ROS_WARN("[RRT] Set Goal Pos: %lf %lf %lf", pos.mX, pos.mY, pos.mZ);
    return plan();
}

OMPLPlanner::OMPLPlanner() : mIsSolving(false), replan_flag(false) {
    //四旋翼的障碍物几何形状
    mpAircraftObject = std::shared_ptr<fcl::CollisionGeometry<double>>(new fcl::Box<double>(0.8, 0.8, 0.3));
    //分辨率参数设置
    fcl::OcTree<double>* tree =
            new fcl::OcTree<double>(std::shared_ptr<const octomap::OcTree>(new octomap::OcTree(0.15)));
    mpTreeObj = std::shared_ptr<fcl::CollisionGeometry<double>>(tree);
    //解的状态空间
    mpSpace = ompl::base::StateSpacePtr(new ompl::base::SE3StateSpace());
    // create a start state
    ompl::base::ScopedState<ompl::base::SE3StateSpace> start(mpSpace);
    // create a goal state
    ompl::base::ScopedState<ompl::base::SE3StateSpace> goal(mpSpace);
    // set the bounds for the R^3 part of SE(3)
    // Boundary of the search space
    //! @todo Add a set function
    ompl::base::RealVectorBounds bounds(3);
    bounds.setLow(0, -10);
    bounds.setHigh(0, 10);
    bounds.setLow(1, -10);
    bounds.setHigh(1, 10);
    bounds.setLow(2, 0);
    bounds.setHigh(2, 5);
    mpSpace->as<ompl::base::SE3StateSpace>()->setBounds(bounds);
    // construct an instance of  space information from this state space
    mpSpaceInfo = ompl::base::SpaceInformationPtr(new ompl::base::SpaceInformation(mpSpace));
    start->setXYZ(0, 0, 0);
    start->as<ompl::base::SO3StateSpace::StateType>(1)->setIdentity();
    // start.random();
    goal->setXYZ(0, -1, 1);
    goal->as<ompl::base::SO3StateSpace::StateType>(1)->setIdentity();
    // goal.random();
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
    ROS_WARN("[RRT] Planner Initialized");

    // Init Octomap Watcher
    mpThreadWatchOctomap = new boost::thread(boost::bind(&OMPLPlanner::watchOctomapThread, this));
    // Setup Publisher
    vis_pub = n.advertise<nav_msgs::Path>("visualization_marker", 0);
    traj_pub = n.advertise<nav_msgs::Path>("waypoints", 1);
}

OMPLPlanner::~OMPLPlanner() {
    if (mpThreadWatchOctomap) {
        mpThreadWatchOctomap->join();
        delete mpThreadWatchOctomap;
    }
    if (mpTreeObj_obj) delete mpTreeObj_obj;
}

bool OMPLPlanner::rePlan() {
    if (!mpPathSmooth) {
        return plan();
    }
#ifdef RRT_DEBUG
    std::cout << "Total Points:" << mpPathSmooth->getStateCount() << std::endl;
#endif
    if (mpPathSmooth->getStateCount() <= 2) {
        plan();
    } else {
        for (std::size_t idx = 0; idx < mpPathSmooth->getStateCount(); idx++) {
            if (!replan_flag) {
                replan_flag = !isStateValid(mpPathSmooth->getState(idx));
            } else {
                break;
            }
        }
        if (replan_flag) {
            plan();
        } else {
        #ifdef RRT_DEBUG
            std::cout << "Replanning not required" << std::endl;
        #endif
        }
    }
}

void OMPLPlanner::watchOctomapThread() {
    ros::Rate rt(GLOBAL_ROS_RATE);
    auto node = boost::make_shared<ros::NodeHandle>();
    auto octomap_sub =
        node->subscribe<octomap_msgs::Octomap>("/octomap_binary", 1,
                boost::bind(&OMPLPlanner::Octomap_callback, this, _1));
    auto goal_sub =
        node->subscribe<geometry_msgs::PointStamped>("/clicked_point", 1,
                boost::bind(&OMPLPlanner::Click_callback, this, _1));
    while (ros::ok()) {
        ros::spinOnce();
        rt.sleep();
    }
}

void OMPLPlanner::Octomap_callback(const octomap_msgs::OctomapConstPtr& msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    octomap::OcTree* tree_oct = dynamic_cast<octomap::OcTree*>(octomap_msgs::msgToMap(*msg));
    fcl::OcTree<double>* tree = new fcl::OcTree<double>(std::shared_ptr<const octomap::OcTree>(tree_oct));
    updateMap(std::shared_ptr<fcl::CollisionGeometry<double>>(tree));
    rePlan();
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
    rrt->setRange(1.0);
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
        // get the goal representation from the problem definition (not the same as the goal state)
        // and inquire about the found path
    #ifdef RRT_DEBUG_PLANNER
        std::cout << "Found solution:" << std::endl;
    #endif
        ompl::base::PathPtr path = mpProblem->getSolutionPath();
        ompl::geometric::PathGeometric* pth = mpProblem->getSolutionPath()->as<ompl::geometric::PathGeometric>();
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
            ompl::geometric::PathSimplifier* pathBSpline = new ompl::geometric::PathSimplifier(mpSpaceInfo);
            mpPathSmooth = new ompl::geometric::PathGeometric(dynamic_cast<const ompl::geometric::PathGeometric&>(
                    *mpProblem->getSolutionPath()));
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
            // ros::Duration(0.1).sleep();
            // Clear memory
            mpProblem->clearSolutionPaths();
            replan_flag = false;
    } else {
    #ifdef RRT_DEBUG_NOSOLUTION
        ROS_ERROR("[RRT] No solution found");
    #endif
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
    fcl::collide(&aircraftObject, mpTreeObj_obj, requestType, collisionResult);
    return(!collisionResult.isCollision());
}

ompl::base::OptimizationObjectivePtr OMPLPlanner::getPathLengthObjWithCostToGo(
        const ompl::base::SpaceInformationPtr& si) {
    //! @todo Need a balanced Optimization Objective
    // // Option 1
    // ob::OptimizationObjectivePtr lengthObj(new ob::PathLengthOptimizationObjective(si));
    // ob::OptimizationObjectivePtr clearObj(new ClearanceObjective(si));
    // return 10.0*lengthObj + clearObj;
    // Option 2
    ompl::base::OptimizationObjectivePtr obj(new ompl::base::PathLengthOptimizationObjective(si));
    // obj->setCostThreshold(ob::Cost(1.51));
    // // Option 3
    // ob::OptimizationObjectivePtr obj(new ob::PathLengthOptimizationObjective(si));
    // obj->setCostToGoHeuristic(&ob::goalRegionCostToGo);
    return obj;
}

}  // namespace OAC
