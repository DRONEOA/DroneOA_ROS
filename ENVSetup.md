# Environment Setup Guide

This guide will help you setup build environment for Ardupilot and SITL simulation environment for ArduCopter.

## Part 1. Ardupilot
### Build Environment

Linux/Ubuntu users can install with apt :
```shell
sudo  apt-get  update
sudo  apt-get  install  git
sudo  apt-get  install  gitk  git-gui
```
Clone Ardupilot git repository from Github:
```shell
git clone https://github.com/ArduPilot/ardupilot
cd ardupilot
git submodule update --init --recursive
```
Install some required packages:
```shell
sudo Tools/environment_install/install-prereqs-ubuntu.sh -y
```
Reload the path (log-out and log-in to make permanent):
```shell
. ~/.profile
```
Install Pymavlink:
```shell
sudo apt-get install libxml2-dev libxslt-dev python-dev
sudo -H pip2 install -U future lxml
sudo -H pip2 install -U pymavlink
```

### Build Ardupilot

Build Ardupilot for CubeBlack (Used in OA project):
```shell
make CubeBlack
```

### Install MAVProxy

First, a few pre-requisite packages need to be installed:
```shell
sudo apt-get install python3-dev python3-opencv python3-pip python3-matplotlib
```
Then download and install MAVProxy via Pypi:
```shell
sudo -H pip install MAVProxy
```

### Setup SITL Simulation

#### Start SITL simulator
To start the simulator first change directory to the vehicle directory. For example, for the multicopter code change to  **ardupilot/ArduCopter**:
```shell
cd ~
cd ardupilot/ArduCopter
```
Then start the simulator using  **sim_vehicle.py**. The first time you run it you should use the -w option to wipe the virtual EEPROM and load the right default parameters for your vehicle.
```shell
sim_vehicle.py -w
```
After the default parameters are loaded you can start the simulator normally. First kill the sim_vehicle.py you are running using Ctrl-C. Then:
```shell
sim_vehicle.py --console --map
```

### Updating MAVProxy and pymavlink

```shell
sudo -H pip install --upgrade pymavlink MAVProxy --user
```

### FlightGear 3D View
Install FlightGear from the terminal:
```shell
sudo apt-get install flightgear
```
Start up a FlightGear instance
```shell
/ardupilot/Tools/autotest/fg_quad_view.sh
```
Start SITL in the terminal in the normal way. In this case we’re specifying the start location as San Francisco airport (KSFO) as this is an interesting airport with lots to see:
```shell
sim_vehicle.py -L KSFO
```

## Part 2. Setup ROS

### Install ROS Melodic
Follow official guide:
[Install ROS](http://wiki.ros.org/melodic/Installation)

### Install Dependencies
Mavros:
```shell
sudo apt-get install ros-melodic-mavros ros-melodic-mavros-extras
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
chmod a+x install_geographiclib_datasets.sh
./install_geographiclib_datasets.sh

sudo apt-get install ros-melodic-rqt ros-melodic-rqt-common-plugins ros-melodic-rqt-robot-plugins
```

Catkin Tools:
```shell
sudo apt-get install python-catkin-tools
```

Install opencv tools
```shell
sudo apt-get install ros-melodic-perception
```

Checkout [.gitlab-ci.yml](http://tuotuogzs.ddns.net/droneoa/droneoa_ros/blob/master/.gitlab-ci.yml) if still missing dependencies

## Part 3. Setup Realsense

### Install Realsense SDK2.0
Follow this official guide:
[Install Realsense SDK](https://www.intelrealsense.com/developers/)

### Install Realsense ROS:
**Note:** Rmember to install all dependencies in official *.travis.yml* file
[Install realsense-ros](https://github.com/IntelRealSense/realsense-ros/blob/development/README.md)

### Launch Camera:
Single camera:
```shell
roslaunch realsense2_camera rs_camera.launch
```
T265 Camera:
```shell
roslaunch realsense2_camera rs_t265.launch
```
Multiple camera:
```shell
roslaunch realsense2_camera rs_multiple_devices.launch serial_no_camera1:=<serial number of the first camera> serial_no_camera2:=<serial number of the second camera>
```
Check serial port:
```shell
rs-enumerate-devices | grep Serial
```
**Note:** If you have issue with D435i camera, please use a older version firmware:
[Firmware Page](https://dev.intelrealsense.com/docs/firmware-releases)

## Part 4. Start SITL Development

### Setup Workspace
First, create a catkin workspace OR use existing catkin workspace (If you have realsense-ros setup).

**ONLY** do this if you don't have a catkin workspace, otherwise use the existing one
```shell
mkdir -p ardupilot_ws/src
cd ardupilot_ws
catkin init
cd src
```
**Note:** if you have issue "catkin not found":
```shell
cd ~
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

**ONLY** do this if you don't them already
```shell
git clone https://github.com/IntelRealSense/realsense-ros.git
git clone http://tuotuogzs.ddns.net/droneoa/ydlidar-x2l-local.git
git clone http://tuotuogzs.ddns.net/droneoa/droneoa_ros.git
git clone https://github.com/hoangthien94/vision_to_mavros.git
```

Build the node with catkin_make:
```shell
cd ..
catkin_make
cd src
```
**Note:** use this command if you use vscode ros plugin
```shell
catkin_make -DCMAKE_EXPORT_COMPILE_COMMANDS=Yes
```
source you ``setup.bash``
```shell
source <WORKSPACE_PATH>/devel/setup.bash
```
You can also add this to the end of ``~/.bashrc``

### Use With SITL
- [Launch File README](launch/README.md)
- [SITL With Gazebo](http://tuotuogzs.ddns.net/droneoa/droneoa_ros/wikis/SITL%20With%20Gazebo)

### Use With Real Vehicle
- [Set Up Jetson OBC](http://tuotuogzs.ddns.net/droneoa/jetson-nano-obc-setup)
- [Launch File README](launch/README.md)

