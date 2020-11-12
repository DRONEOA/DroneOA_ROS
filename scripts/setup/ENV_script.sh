#!/bin/bash
echo "---------- $0 start ----------"
set -e
# set -x  # Expands variables and prints a little + sign before the line
if [ $EUID == 0 ]; then
    echo "Please do not run this script as root; don't sudo it!"
    exit 1
fi

# Determine which ROS dist to install based on OC version
ROSDIST="melodic"
REALSENSE_BRANCH=2.2.17
DRONEOA_BRANCH=master
if [[ $(lsb_release -rs) == "18.04" ]]; then
    echo "Ubuntu 18.04 - using melodic"
	ROSDIST="melodic"
elif [[ $(lsb_release -rs) == "20.04" ]]; then
	echo "Ubuntu 20.04 - using noetic"
	ROSDIST="noetic"
else
    echo "Unsupported OS Dist !!!"
	exit 1
fi
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt update
sudo apt install ros-${ROSDIST}-desktop-full -y
sudo rm /etc/ros/rosdep/sources.list.d/* || true # clear the directory before init

## Install rosdep
if [[ $ROSDIST == "melodic" ]]; then
	sudo apt install python-rosdep
elif [[ $ROSDIST == "noetic" ]]; then
	sudo apt install python3-rosdep
else
    echo "Unsupported OS Dist !!!"
	exit 1
fi

## Init rosdep
sudo rosdep init
rosdep update

## Source ROS
BASH_FILE=~/.bashrc
ZSH_FILE=~/.zshrc
if test -f "$BASH_FILE"; then
	echo "$BASH_FILE exist"
	echo "source /opt/ros/${ROSDIST}/setup.bash" >> ~/.bashrc
	source ~/.bashrc
fi
if test -f "$ZSH_FILE"; then
	echo "$ZSH_FILE exists"
	echo "source /opt/ros/${ROSDIST}/setup.zsh" >> ~/.zshrc
	source ~/.zshrc
fi

## Install other ROS tools
if [[ $ROSDIST == "melodic" ]]; then
	sudo apt install python-rosinstall python-rosinstall-generator python-wstool build-essential -y
elif [[ $ROSDIST == "noetic" ]]; then
	sudo apt install python3-rosinstall python3-rosinstall-generator python3-wstool build-essential -y
else
    echo "Unsupported OS Dist !!!"
	exit 1
fi

sudo apt install git gitk -y

# Construct Main Workspace
cd ~
echo -e "\033[0;33m ===== Customize Workspace ===== \033[0m"
echo " == File Structure Example == "
echo " - <Workspace root directory>"
echo "   - <Ardupilot firmware>"
echo "   - <ROS workspace>"
echo "     - src"
echo "       - droneoa_ros"
echo "       - realsense-ros"
echo "       - ydlidar_ros/ydlidar-x2l-local"
echo " == File Structure Example =="
read -p "Pick workspace root directory[absolute path](default: $HOME/Workspace):" ROOT_WS_PATH
read -p "Pick ROS workspace name(default: ardupilot_ws):" ROS_WS_NAME
ROOT_WS_PATH=${ROOT_WS_PATH:-"$HOME/Workspace"}
ROS_WS_NAME=${ROS_WS_NAME:-"ardupilot_ws"}
echo -e "\033[0;33m =====  Start Building WS  ===== \033[0m"
mkdir -p $ROOT_WS_PATH/$ROS_WS_NAME/src
cd $ROOT_WS_PATH/$ROS_WS_NAME/src
git clone -b $DRONEOA_BRANCH http://gitlab.tuotuogzs.com/droneoa/droneoa_ros.git

## Install Realsense Lib
echo "----- Install Realsense Library"
echo 'deb http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo xenial main' | sudo tee /etc/apt/sources.list.d/realsense-public.list
sudo apt-key add ./droneoa_ros/scripts/ci/realsenseKey
sudo apt-get update
sudo apt-get install librealsense2-dkms --allow-unauthenticated -y
sudo apt-get install librealsense2-dev --allow-unauthenticated -y
sudo apt-get install librealsense2-utils -y
if [[ $ROSDIST == "melodic" ]]; then
	sudo apt-get install libreadline7 libreadline-dev -y
elif [[ $ROSDIST == "noetic" ]]; then
	sudo apt-get install libreadline8 libreadline-dev -y
fi

## Install other ROS Dependencies
echo "----- Install ROS Dependencies"
sudo apt-get install ros-${ROSDIST}-cv-bridge -y
sudo apt-get install ros-${ROSDIST}-image-transport -y
sudo apt-get install ros-${ROSDIST}-tf -y
sudo apt-get install ros-${ROSDIST}-diagnostic-updater -y
sudo apt-get install ros-${ROSDIST}-ddynamic-reconfigure -y
sudo apt-get install ros-${ROSDIST}-nodelet -y
sudo apt-get install ros-${ROSDIST}-perception -y
sudo apt-get install ros-${ROSDIST}-pcl-ros -y
sudo apt-get install ros-${ROSDIST}-pcl-conversions -y
sudo apt-get install ros-${ROSDIST}-mavros -y
sudo apt-get install liboctomap-dev -y
sudo apt-get install ros-${ROSDIST}-octomap -y
sudo apt-get install ros-${ROSDIST}-octomap-server -y
if [[ $ROSDIST == "melodic" ]]; then
	sudo apt-get install ros-${ROSDIST}-octomap-rviz-plugins -y
fi

## Clone Dependency Repos
echo "----- Clone Dependency Repos"
git clone https://github.com/IntelRealSense/realsense-ros.git
if [[ $ROSDIST == "melodic" ]]; then
	git clone https://gitlab.tuotuogzs.com/droneoa/ydlidar-x2l-local.git
elif [[ $ROSDIST == "noetic" ]]; then
	git clone https://github.com/YDLIDAR/ydlidar_ros.git
fi
cd realsense-ros/
git checkout $REALSENSE_BRANCH
## Install Geographiclib
echo "----- Install Geographiclib"
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
sudo chmod +x ./install_geographiclib_datasets.sh
sudo ./install_geographiclib_datasets.sh

# Setup OMPL
echo "----- Build OMPL"
cd $ROOT_WS_PATH/$ROS_WS_NAME/src/droneoa_ros/scripts/setup
bash ./RRT_script.sh

# Try to build ROS workspace
echo "----- Build ROS Worksapce"
cd $ROOT_WS_PATH/$ROS_WS_NAME
catkin_make

## Source the workspace
if test -f "$BASH_FILE"; then
	echo "$BASH_FILE exist"
	`echo "source $ROOT_WS_PATH/$ROS_WS_NAME/devel/setup.bash" >> ~/.bashrc`
	source ~/.bashrc
fi
if test -f "$ZSH_FILE"; then
	echo "$ZSH_FILE exists"
	echo `echo "source $ROOT_WS_PATH/$ROS_WS_NAME/devel/setup.zsh" >> ~/.zshrc`
	source ~/.zshrc
fi

# Setup Ardupilot Workspace
## [TODO] ensure correct python version; Simplfy;
echo "----- Build Ardupilot"
cd $ROOT_WS_PATH
git clone -b Copter-4.0.3 https://github.com/ArduPilot/ardupilot
cd ardupilot
git submodule update --init --recursive
sudo chown -R $USER $ROOT_WS_PATH/ardupilot
sudo chown -R $USER $ROOT_WS_PATH/$ROS_WS_NAME
find . -name install-prereqs-ubuntu.sh -exec /bin/bash -x {} -y \;
. ~/.profile
sudo apt-get install libxml2-dev libxslt-dev python-dev -y
if [[ $ROSDIST == "noetic" ]]; then
	curl https://bootstrap.pypa.io/get-pip.py --output get-pip.py
	sudo python2 get-pip.py
fi
sudo -H pip2 install -U future lxml
sudo -H pip2 install -U pymavlink
sudo apt-get install python3-dev python3-opencv python3-pip python3-matplotlib -y
sudo -H pip install -Iv MAVProxy==1.8.18
pip install MAVProxy==1.8.18
./waf configure --board sitl
. ~/.profile

## Attempt to launch a SITL instance, which will trigger a build
cd ArduCopter
sim_vehicle.py -w
