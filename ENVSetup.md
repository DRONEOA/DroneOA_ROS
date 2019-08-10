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
pip install --upgrade pymavlink MAVProxy --user
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

### Setup Workspace
Follow this local wiki page:
[Setup For SITL](http://tuotuogzs.ddns.net/droneoa/droneoa_ros/wikis/Launch%20In%20SITL)

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
