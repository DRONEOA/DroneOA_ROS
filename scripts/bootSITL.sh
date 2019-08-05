#!/bin/bash
# A Simple Shell Script To Start SITL and ROS nodes
# Make workspace
./subScripts/makeWorkSpace.sh
# Start a SITL ArduCopter
gnome-terminal -t “SITLInstance” -x bash -c “sh ./subScripts/launchSITLInstance.sh;exec bash;”
# Start mavros node
sleep 10
gnome-terminal -t “Mavros” -x bash -c “sh ./launchConnectionProfile.sh;exec bash;”
# Start droneoa node
sleep 25
gnome-terminal -t “DroneOA” -x bash -c “sh ./launchDroneOANode.sh;exec bash;”