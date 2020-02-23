#!/bin/bash
echo "---------- $0 start ----------"
set -e
set -x
if [ $EUID == 0 ]; then
    echo "Please do not run this script as root; don't sudo it!"
    exit 1
fi

sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt update
sudo apt install ros-melodic-desktop-full -y
sudo rm /etc/ros/rosdep/sources.list.d/* || true # clear the directory before init
sudo rosdep init
rosdep update

BASH_FILE=~/.bashrc
ZSH_FILE=~/.zshrc
if test -f "$BASH_FILE"; then
	echo "$BASH_FILE exist"
	echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
	source ~/.bashrc
fi
if test -f "$ZSH_FILE"; then
	echo "$ZSH_FILE exists"
	echo "source /opt/ros/melodic/setup.zsh" >> ~/.zshrc
	source ~/.zshrc
fi
sudo apt install python-rosinstall python-rosinstall-generator python-wstool build-essential -y

sudo apt install git gitk -y
cd ~
mkdir ardupilot_ws
cd ardupilot_ws
mkdir src
cd src
git clone http://gitlab.tuotuogzs.com/droneoa/droneoa_ros.git

cd ~/ardupilot_ws
echo 'deb http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo xenial main' | sudo tee /etc/apt/sources.list.d/realsense-public.list
sudo apt-key add ./src/droneoa_ros/scripts/realsenseKey
sudo apt-get update
sudo apt-get install librealsense2-dkms --allow-unauthenticated -y
sudo apt-get install librealsense2-dev --allow-unauthenticated -y
sudo apt-get install librealsense2-utils -y
sudo apt-get install ros-melodic-cv-bridge -y
sudo apt-get install ros-melodic-image-transport
sudo apt-get install ros-melodic-tf -y
sudo apt-get install ros-melodic-diagnostic-updater -y
sudo apt-get install ros-melodic-ddynamic-reconfigure -y
sudo apt-get install ros-melodic-nodelet -y
sudo apt-get install ros-melodic-perception -y
sudo apt-get install ros-melodic-pcl-ros -y
sudo apt-get install ros-melodic-pcl-conversions -y
sudo apt-get install ros-melodic-mavros -y
cd src
git clone https://github.com/IntelRealSense/realsense-ros.git
git clone http://gitlab.tuotuogzs.com/droneoa/ydlidar-x2l-local.git
cd realsense-ros/
git checkout `git tag | sort -V | grep -P "^\d+\.\d+\.\d+" | tail -1`

wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
sudo chmod +x ./install_geographiclib_datasets.sh
sudo ./install_geographiclib_datasets.sh


cd ~
git clone https://github.com/ArduPilot/ardupilot
cd ardupilot
git submodule update --init --recursive
sudo chown -R $USER ~/ardupilot
sudo chown -R $USER ~/ardupilot_ws
Tools/environment_install/install-prereqs-ubuntu.sh -y
. ~/.profile
sudo apt-get install libxml2-dev libxslt-dev python-dev -y
sudo -H pip2 install -U future lxml
sudo -H pip2 install -U pymavlink
sudo apt-get install python3-dev python3-opencv python3-pip python3-matplotlib -y
sudo -H pip install MAVProxy
make sitl



