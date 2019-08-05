#!/bin/bash
# Start mavros and connect
mkdir ~/ardupilot_ws/src/launch
cd ~/ardupilot_ws/src/launch
roscp mavros apm.launch apm.launch
roslaunch apm.launch