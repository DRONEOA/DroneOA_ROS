# ROS Package For Drone Obstacle Avoidance Project

## Introduction

### Hardware
- **OA07A** Mid Range Development Platform
    - Designed for indoor and outdoor test environment
    - OBC: Jetson nano
    - Depth Camera: D435i
    - Laser Scanner: RPLidar A1
    - VOI: t265

- **OA07S** Short Range Low-Cost Development Platform [WIP]
    - Designed for indoor and outdoor test environment
    - OBC: Jetson nano
    - Depth Camera: SR305
    - VOI: t265 [Optional]

### Software Package Component
Required ROS packages:
- mavros

Modules:
- CNCInterface [WIP]
    - Direct command and control interface communicate with mavros and give command to the flight controller unit via mavlink
    - Supported Commands:
        - SetMode
        - Takeoff
        - Landing
        - Waypoint Guided
        - Yaw Control
        - Watcher [WIP]
- SensorInterface [WIP]
    - Sensor data pre-processing
    - Camera data pre-processing
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
- [SITL With Gazebo](http://tuotuogzs.ddns.net/droneoa/droneoa_ros/wikis/SITL%20With%20Gazebo)

### Use With Real Vehicle
- [Set Up Jetson OBC](http://tuotuogzs.ddns.net/droneoa/jetson-nano-obc-setup)
- [How To Run On Real Vehicle](http://tuotuogzs.ddns.net/droneoa/droneoa_ros/wikis/Run-With-Real-Vehicle)

## Credit