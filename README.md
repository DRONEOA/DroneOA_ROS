# ROS Package For Drone Obstacle Avoidance Project

[![pipeline status](http://tuotuogzs.ddns.net/droneoa/droneoa_ros/badges/master/pipeline.svg)](http://tuotuogzs.ddns.net/droneoa/droneoa_ros/commits/master)

## Introduction

### Hardware
- [OA450](http://tuotuogzs.ddns.net/droneoa/droneoa_ros/wikis/Test-Vehicle-OA450)
- OA650

### Software Package Component
Required ROS packages / Library:
- mavros
- realsense ros
- opencv

Modules:
- CNCInterface [WIP]
    - Direct command and control interface communicate with mavros and give command to the flight controller unit via mavlink
    - Supported Commands:
        - SetMode
        - Takeoff
        - Landing
        - Waypoint Goto (relative / global)
        - Yaw Goto
        - Yaw Control
        - Watcher (Altitude, Position, Battery, State, Velocity, Orientation)
- SensorInterface [WIP]
    - Lidar data pre-processing (LidarInterface)
        - Generating degree sector
        - Simple data filtering
    - Camera data pre-processing (RSCInterface)
        - Distance Filtering
        - Flight Path Filtering
    - "Black Box" logging
- AIInterface [WIP]
    - Coming soon
- OAController [Stage 1](http://tuotuogzs.ddns.net/droneoa/droneoa_ros/wikis/Stage-1-Collision-Avoidance-Flow)
    - Main controller
    - Switch between OA modes
    - Application level fail safety handling [Stage 1 Fail Safety](http://tuotuogzs.ddns.net/droneoa/droneoa_ros/wikis/Stage-1-Fail-Safety-Graph)
    - Mission queue
- Utils [WIP]
- Unittests

## How To [DEV ONLY]

### Environment Setup
- [Setup Development Environment](ENVSetup.md)

### Use With SITL
- [Launch File README](launch/README.md)
- [SITL With Gazebo *NEW](http://tuotuogzs.ddns.net/droneoa/droneoa_ros/wikis/SITL%20With%20Gazebo)

### Use With Real Vehicle
- [Set Up Jetson OBC](http://tuotuogzs.ddns.net/droneoa/jetson-nano-obc-setup)
- [Launch File README](launch/README.md)

## Credit
DroneOA Group 2019