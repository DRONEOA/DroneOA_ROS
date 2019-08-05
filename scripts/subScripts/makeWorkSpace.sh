#!/bin/bash
# Make workspace
mkdir -p ~/ardupilot_ws/src
cd ~/ardupilot_ws
catkin init
cd src
# Clone the repo
git clone http://tuotuogzs.ddns.net/droneoa/droneoa_ros.git
cd ..
catkin_make
cd src
