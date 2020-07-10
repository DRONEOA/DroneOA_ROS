#!/bin/bash
mkdir RRTInstallDownload
cd RRTInstallDownload
echo "---------- Install OMPL ----------"
wget https://ompl.kavrakilab.org/install-ompl-ubuntu.sh
sudo chmod u+x install-ompl-ubuntu.sh
./install-ompl-ubuntu.sh -a
sudo cp -r /usr/local/include/ompl-1.5/ompl /usr/local/include/ompl
# sudo rm -r /usr/local/include/ompl-1.5
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list'
wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
sudo apt-get update
sudo apt-get install ros-melodic-ompl -y
sudo apt-get install libompl-dev -y
sudo apt-get install ros-melodic-fcl-catkin -y
echo "----------  All Done  -----------"
