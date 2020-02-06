# ROS Package For Drone Obstacle Avoidance Project

[![pipeline status](http://tuotuogzs.ddns.net/droneoa/droneoa_ros/badges/master/pipeline.svg)](http://tuotuogzs.ddns.net/droneoa/droneoa_ros/commits/master)

## Introduction
@TODO

### Test Hardware

#### Test Platform:
- [OA450](http://tuotuogzs.ddns.net/droneoa/droneoa_ros/wikis/Test-Vehicle-OA450)
- OA650 (WIP)

#### Supported Sensors:
- Lidar:
    - YDLidar X2L
    - RPLidar A1 (WIP)
- Depth Camera:
    - D435(i)
- Normal Camera

#### Supported OBCs:
- Jetson nano
- Raspberry Pi 4 (Planned)

### Software Package Component
#### Required ROS packages / Library:
- mavros
- realsense ros
- opencv
- PCI library
- see `.gitlab-ci.yml` for more dependencies

#### Modules:
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
- AIInterface [Planned]
- OAController [Stage 1](http://tuotuogzs.ddns.net/droneoa/droneoa_ros/wikis/Stage-1-Collision-Avoidance-Flow)
    - Main controller
    - Switch between OA modes
    - Application level fail safety handling [Stage 1 Fail Safety](http://tuotuogzs.ddns.net/droneoa/droneoa_ros/wikis/Stage-1-Fail-Safety-Graph)
    - Mission queue
- Utils
- Unittests

## How To [DEV ONLY]

### Environment Setup
- [Setup Development Environment](ENVSetup.md)
- Setup Script [WIP]

### Launch With SITL
- [Launch File README](launch/README.md)
- [SITL With UE4](http://tuotuogzs.ddns.net/droneoa/droneoa_ros_ue4_simulator)
- [SITL With Gazebo (Discontinued)](http://tuotuogzs.ddns.net/droneoa/droneoa_ros/wikis/SITL%20With%20Gazebo)

### Launch With Real Vehicle
- [Set Up Jetson OBC](http://tuotuogzs.ddns.net/droneoa/jetson-nano-obc-setup)
- [Launch File README](launch/README.md)

## Credit
DroneOA Group 2019 :palm_tree: