# ROS Package For Drone Obstacle Avoidance Project

[![pipeline status](http://tuotuogzs.ddns.net/droneoa/droneoa_ros/badges/master/pipeline.svg)](http://tuotuogzs.ddns.net/droneoa/droneoa_ros/commits/master)

## Introduction

### Hardware
- **OA07A** Mid Range Development Platform
    - Designed for indoor and outdoor test environment
    - OBC: Jetson nano
    - Depth Camera: D435i / D435
    - Laser Scanner: RPLidar A1 / YDLidar X2L
    - VOI: t265

- **OA07S** Short Range Low-Cost Development Platform [WIP]
    - Designed for indoor and outdoor test environment
    - OBC: Jetson nano
    - Depth Camera: SR305
    - Laser Scanner: YDLidar X2L
    - VOI: t265 [Optional]

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
    - Sensor data pre-processing (LSInterface)
    - Camera data pre-processing (RSCInterface)
    - "Black Box" logging
- AIInterface [WIP]
    - Coming soon
- OAController [WIP]
    - Main controller
    - Switch between OA modes
    - Application level fail safety handling
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