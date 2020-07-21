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
sudo apt install python-rosdep
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
echo "Please pick you workspace root directory(absolute path): press ENTER to continue"
echo ""
read absolutePath
mkdir -p $absolutePath/ardupilot_ws/src
cd $absolutePath/ardupilot_ws/src
git clone http://gitlab.tuotuogzs.com/droneoa/droneoa_ros.git

echo "----- Install Realsense Library"
echo 'deb http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo xenial main' | sudo tee /etc/apt/sources.list.d/realsense-public.list
sudo apt-key add ./droneoa_ros/scripts/ci/realsenseKey
sudo apt-get update
sudo apt-get install librealsense2-dkms --allow-unauthenticated -y
sudo apt-get install librealsense2-dev --allow-unauthenticated -y
sudo apt-get install librealsense2-utils -y
sudo apt-get install libreadline7 libreadline-dev -y
echo "----- Install ROS Dependencies"
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
sudo apt-get install liboctomap-dev -y
sudo apt-get install ros-melodic-octomap -y
sudo apt-get install ros-melodic-octomap-server -y
sudo apt-get install ros-melodic-octomap-rviz-plugins -y
echo "----- Clone Dependency Repos"
git clone https://github.com/IntelRealSense/realsense-ros.git
git clone https://gitlab.tuotuogzs.com/droneoa/ydlidar-x2l-local.git
cd realsense-ros/
git checkout `git tag | sort -V | grep -P "^\d+\.\d+\.\d+" | tail -1`

echo "----- Install Geographiclib"
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
sudo chmod +x ./install_geographiclib_datasets.sh
sudo ./install_geographiclib_datasets.sh

echo "----- Build OMPL"
cd $absolutePath
bash ./RRT_script.sh

echo "----- Build Ardupilot"
cd $absolutePath
git clone -b Copter-4.0.3 https://github.com/ArduPilot/ardupilot
cd ardupilot
git submodule update --init --recursive
sudo chown -R $USER $absolutePath/ardupilot
sudo chown -R $USER $absolutePath/ardupilot_ws
find . -name install-prereqs-ubuntu.sh -exec /bin/bash -x {} -y \;
. ~/.profile
sudo apt-get install libxml2-dev libxslt-dev python-dev -y
sudo -H pip2 install -U future lxml
sudo -H pip2 install -U pymavlink
sudo apt-get install python3-dev python3-opencv python3-pip python3-matplotlib -y
sudo -H pip install -Iv MAVProxy==1.8.18
pip install MAVProxy==1.8.18
./waf configure --board sitl
. ~/.profile
cd ArduCopter
sim_vehicle.py -w
