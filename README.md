# ROS Package For Drone Obstacle Avoidance Project

[![pipeline status](https://gitlab.tuotuogzs.com/droneoa/droneoa_ros/badges/master/pipeline.svg)](https://gitlab.tuotuogzs.com/droneoa/droneoa_ros/commits/master)

## Introduction
Drones are used in an unprecedented number of ways since the advent, with millions of people using it for agriculture, journalism, photography, courier services, etc. Seeing this opportunity, we propose to develop some solutions that make drones safer. And make drone obstacle avoidance development easier.

**OASIS**, short for `Obstacle Avoidance with Spontaneous Itinerancy Strategies`, is a software package we have been working on to tackle this target. It provides a unified interface (common API for sensors and flight controllers) and tool kit (simulator support and debug tools) for an automatic obstacle avoidance platform, powered by multiple strategies (algorithm) running simultaneously. For example, both LIDAR and traditional camera with AI can be used to determine collision probabilities and/or path independently, which are then synthesized to provide a final decision.

We plan to get official support in Ardupilot community, and possibly develop a mobile ground control station as a side project. The challenge we face was mainly the strict requirement of the correctness of the specification and implementation due to the safety-critical nature of the project.

### Current Progress:
- Stage 1: Simple Collision Avoidance (stop when a collision is possible and waiting for human input), and basic tool kit. This stage is designed for hardware-software interface development & testing and system integration to verify feasibility. [DONE]
- Stage 2: Add in 2D environment path planning [In progress]
- Stage 3: Add in 3D environment path planning [In progress]
- Stage 4: Add more dev friendly features and addon installation support [Planned]

### Test Hardware

#### Test Platform:
- [OA450](https://gitlab.tuotuogzs.com/droneoa/droneoa_ros/wikis/Test-Vehicle-OA450)
- OA650 (WIP)

#### Supported Sensors:
- Lidar:
    - YDLidar X2L
    - RPLidar Lineup (WIP)
- Depth Camera:
    - D435(i) local only
    - D435(I) + T265 local/global
- Normal Camera (Planned)

#### Supported OBCs:
- Jetson nano
- Raspberry Pi 4 (Planned)
- Up board 1st gen
- Most other x86 system with at least 1 usb/serial port

### Software Package Component
#### Required ROS packages / Library:
- mavros
- realsense ros
- opencv
- PCI library
- see `.gitlab-ci.yml` for more dependencies

#### Modules:
- CNCInterface
    - Direct command and control interface communicate with mavros and give command to the flight controller unit via mavlink
    - Supported Commands:
        - SetMode
        - Takeoff
        - Landing
        - Waypoint Goto (relative / global)
        - Yaw Goto
        - Yaw Control
        - Watcher (Altitude, Position, Battery, State, Velocity, Orientation)
        - more...
- SensorInterface [WIP]
    - Lidar data pre-processing (LidarInterface)
        - Generating degree sector
        - Simple data filtering
    - Camera data pre-processing (RSCInterface)
        - Distance Filtering
        - Flight Path HUD [Planned]
    - Octomap pre-processing (WIP)
- AIInterface [Planned Low Priority]
- OAController [Stage 1](https://gitlab.tuotuogzs.com/droneoa/droneoa_ros/wikis/Stage-1-Collision-Avoidance-Flow)
    - Main controller
    - Switch between OA modes
    - Cycle control
    - Application level fail safety handling [Stage 1 Fail Safety](https://gitlab.tuotuogzs.com/droneoa/droneoa_ros/wikis/Stage-1-Fail-Safety-Graph)
    - Mission queue
- Console Manager
    - Process and parse console command
    - Direct control the vehicle
    - Process and execute command queue (with time relationship) [Planned]
- GUI
    - JS based
    - Through TCP bridge and console manager
- Addon Manager [Planned Requested]
    - Install new app / algorithm using URL
    - Auto pull and compile new modules
- Utils
- Unittests

### How to:
- Checkout the official [Wiki](http://droneoa.tuotuogzs.net/droneoa_gitbook) 

### Environment Setup
- [Setup Development Environment](ENVSetup.md)
- [Setup Script](https://gitlab.tuotuogzs.com/droneoa/droneoa_ros/-/tree/master/scripts/setup)

### Launch With SITL
- [Launch File README](launch/README.md)
- [SITL With UE4](https://gitlab.tuotuogzs.com/droneoa/droneoa_ros_ue4_simulator)
- [SITL With Gazebo (Discontinued)](https://gitlab.tuotuogzs.com/droneoa/droneoa_ros/wikis/SITL%20With%20Gazebo)

### Launch With Real Vehicle
- [Set Up Jetson OBC](https://gitlab.tuotuogzs.com/droneoa/jetson-nano-obc-setup)
- [Launch File README](launch/README.md)

## Credit
DroneOA Group 2019 :palm_tree:
