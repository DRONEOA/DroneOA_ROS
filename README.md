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

### Use With SITL
- [Setup Ardupilot Development Environment](http://tuotuogzs.ddns.net/shibohan/arducopter/wikis/Environment-Setup)
- [How To Run With SITL](http://tuotuogzs.ddns.net/droneoa/droneoa_ros/wikis/Launch%20In%20SITL)
- [SITL With Gazebo *NEW](http://tuotuogzs.ddns.net/droneoa/droneoa_ros/wikis/SITL%20With%20Gazebo)

### Use With Real Vehicle
- [Set Up Jetson OBC](http://tuotuogzs.ddns.net/droneoa/jetson-nano-obc-setup)
- [How To Run On Real Vehicle](http://tuotuogzs.ddns.net/droneoa/droneoa_ros/wikis/Run-With-Real-Vehicle)

## Credit
DroneOA Group 2019