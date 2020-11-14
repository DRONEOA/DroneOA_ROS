#!/bin/bash
echo "---------- $0 start ----------"

if [ $EUID == 0 ]; then
    echo "Please do not run this script as root; don't sudo it!"
    exit 1
fi

##################################################
#  Define Steps As Functions
##################################################

# Determine which ROS dist to install based on OC version
ROSDIST=""
REALSENSE_BRANCH=2.2.17
DRONEOA_BRANCH=master
ARDUPILOT_BRANCH=Copter-4.0.3
MAVPROXY_VERSION=1.8.18
START_ABS_PATH=$(pwd)

function determine_os_version()
{
    if [[ $(lsb_release -rs) == "18.04" ]]; then
        echo "Ubuntu 18.04 - using melodic"
		ROSDIST="melodic"
    elif [[ $(lsb_release -rs) == "20.04" ]]; then
		echo "Ubuntu 20.04 - using noetic"
		ROSDIST="noetic"
    else
        echo "Unsupported OS Dist !!!"
    fi
}

# Install ROS
function install_ros()
{(
	set -e
	echo "----- Install ROS Desktop -----"
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
		return 1
	fi

	## Init rosdep
	sudo rosdep init
	rosdep update
)}

## Source ROS
BASH_FILE=~/.bashrc
ZSH_FILE=~/.zshrc
function source_ros()
{
	if test -f "$BASH_FILE"; then
		echo "$BASH_FILE exist"
		if grep -Fxq "source /opt/ros/${ROSDIST}/setup.bash" $BASH_FILE
		then
			echo "ROS Source Already Exist"
		else
			echo "ROS Source Added"
			echo "source /opt/ros/${ROSDIST}/setup.bash" >> ~/.bashrc
		fi
		source ~/.bashrc
	fi
	if test -f "$ZSH_FILE"; then
		echo "$ZSH_FILE exists"
		if grep -Fxq "source /opt/ros/${ROSDIST}/setup.zsh" $ZSH_FILE
		then
			echo "ROS Source Already Exist"
		else
			echo "ROS Source Added"
			echo "source /opt/ros/${ROSDIST}/setup.zsh" >> ~/.zshrc
		fi
		source ~/.zshrc
	fi
}

## Install ROS Tools
function install_ros_tools()
{(
	set -e
	echo "----- Install ROS Tools: ${ROSDIST} -----"
	if [[ $ROSDIST == "melodic" ]]; then
	sudo apt install python-rosinstall python-rosinstall-generator python-wstool build-essential -y
	elif [[ $ROSDIST == "noetic" ]]; then
		sudo apt install python3-rosinstall python3-rosinstall-generator python3-wstool build-essential -y
	else
		echo "Unsupported OS Dist !!!"
		exit 1
	fi
	sudo apt install git gitk -y
)}

## Install other ROS Dependencies
function install_ros_dependencies()
{(
	set -e
	echo "----- Install ROS Dependencies: ${ROSDIST} -----"
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
)}

# Construct Workspace
ROOT_WS_PATH=""
ROS_WS_NAME=""
function construct_workspace()
{
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
	cd $START_ABS_PATH
}

# Print WS confirmation [debug]
function ws_confirmation()
{
	echo " == File Structure Conf == "
	echo " - $ROOT_WS_PATH"
	echo "   - ardupilot"
	echo "   - $ROS_WS_NAME"
	echo "     - src"
	echo "       - droneoa_ros"
	echo "       - realsense-ros"
	echo "       - ydlidar_ros/ydlidar-x2l-local"
	echo " == File Structure Conf =="
}

# Clone repos
function clone_repos()
{(
	set -e
	echo "----- Clone Required Repos -----"
	cd $ROOT_WS_PATH/$ROS_WS_NAME/src
	if [ ! -d "droneoa_ros" ]; then
		git clone -b $DRONEOA_BRANCH http://gitlab.tuotuogzs.com/droneoa/droneoa_ros.git
	else
		echo "Droneoa_ros folder already exist"
	fi
	if [ ! -d "realsense-ros" ]; then
		git clone https://github.com/IntelRealSense/realsense-ros.git
	else
		echo "Realsense-ros folder already exist"
	fi
	if [[ $ROSDIST == "melodic" ]]; then
		if [ ! -d "ydlidar-x2l-local" ]; then
			git clone https://gitlab.tuotuogzs.com/droneoa/ydlidar-x2l-local.git
		else
			echo "Ydlidar-x2l-local folder already exist"
		fi
	elif [[ $ROSDIST == "noetic" ]]; then
		if [ ! -d "ydlidar_ros" ]; then
			git clone https://github.com/YDLIDAR/ydlidar_ros.git
		else
			echo "Ydlidar_ros folder already exist"
		fi
	fi
	cd realsense-ros/
	git checkout $REALSENSE_BRANCH
	cd $START_ABS_PATH
)}

# Install Realsense Lib
function install_realsense()
{(
	#! @todo Support build from source
	set -e
	echo "----- Install Realsense Library -----"
	echo 'deb http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo xenial main' | sudo tee /etc/apt/sources.list.d/realsense-public.list
	sudo apt-key add realsenseKey
	sudo apt-get update
	sudo apt-get install librealsense2-dkms --allow-unauthenticated -y
	sudo apt-get install librealsense2-dev --allow-unauthenticated -y
	sudo apt-get install librealsense2-utils -y
	if [[ $ROSDIST == "melodic" ]]; then
		sudo apt-get install libreadline7 libreadline-dev -y
	elif [[ $ROSDIST == "noetic" ]]; then
		sudo apt-get install libreadline8 libreadline-dev -y
	fi
)}

# Install Geographiclib
function install_geographiclib()
{(
	set -e
	echo "----- Install Geographiclib -----"
	rm install_geographiclib_datasets.sh || true
	wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
	sudo chmod +x ./install_geographiclib_datasets.sh
	sudo ./install_geographiclib_datasets.sh
)}

# Install OMPL Library
function install_ompl()
{(
	set -e
	echo "----- Build & Install OMPL -----"
	bash ./RRT_script.sh
)}

# Build ROS Workspace
function build_ros_ws()
{(
	set -e
	cd $ROOT_WS_PATH/$ROS_WS_NAME
	catkin_make
	cd $START_ABS_PATH
)}

# Source ROS Workspace
function source_ros_ws()
{
	if test -f "$BASH_FILE"; then
		echo "$BASH_FILE exist"
		if grep -Fxq "source $ROOT_WS_PATH/$ROS_WS_NAME/devel/setup.bash" $BASH_FILE
		then
			echo "ROS Source Already Exist"
		else
			echo "ROS Source Added"
			echo "source $ROOT_WS_PATH/$ROS_WS_NAME/devel/setup.bash" >> ~/.bashrc
		fi
		source ~/.bashrc
	fi
	if test -f "$ZSH_FILE"; then
		echo "$ZSH_FILE exists"
		if grep -Fxq "source $ROOT_WS_PATH/$ROS_WS_NAME/devel/setup.zsh" $ZSH_FILE
		then
			echo "ROS Source Already Exist"
		else
			echo "ROS Source Added"
			echo "source $ROOT_WS_PATH/$ROS_WS_NAME/devel/setup.zsh" >> ~/.zshrc
		fi
		source ~/.zshrc
	fi
}

# Build Ardupilot
function build_ardupilot()
{(
	set -e
	echo "----- Build Ardupilot -----"
	cd $ROOT_WS_PATH
	if [ ! -d "ardupilot" ]; then
		git clone -b $ARDUPILOT_BRANCH https://github.com/ArduPilot/ardupilot
	else
		echo "Ardupilot folder already exist"
	fi
	cd ardupilot
	git submodule update --init --recursive
	sudo chown -R $USER $ROOT_WS_PATH/ardupilot
	sudo chown -R $USER $ROOT_WS_PATH/$ROS_WS_NAME
	find . -name install-prereqs-ubuntu.sh -exec /bin/bash -x {} -y \;
	sudo apt-get install libxml2-dev libxslt-dev python-dev -y
	if [[ $ROSDIST == "noetic" ]]; then
		curl https://bootstrap.pypa.io/get-pip.py --output get-pip.py
		sudo python2 get-pip.py
	fi
	sudo -H pip2 install -U future lxml
	sudo -H pip2 install -U pymavlink
	sudo apt-get install python3-dev python3-opencv python3-pip python3-matplotlib -y
	sudo -H pip install -Iv MAVProxy==$MAVPROXY_VERSION
	pip install MAVProxy==$MAVPROXY_VERSION
	## Setup SITL
	./waf configure --board sitl
	. ~/.profile || true
	## Attemp build and launch a Copter SITL instance
	cd ArduCopter
	sim_vehicle.py -w
	cd $START_ABS_PATH
)}

##################################################
#  Main, run the sequence
##################################################

# Check OS Version
determine_os_version
if [[ $ROSDIST == "" ]]
then
	echo "Error Detecting ROS Version To Use"
	return 1
fi
echo "ROSDIST: $ROSDIST"

# Build Workspace
construct_workspace
echo "ROOT_WS_PATH $ROOT_WS_PATH"
echo "ROS_WS_NAME $ROS_WS_NAME"
ws_confirmation # DEBUG

# Install ROS
install_ros
install_ros_tools
source_ros
install_ros_dependencies

# Install Realsense
install_realsense

# Install Geographiclib
install_geographiclib

# Install OMPL
install_ompl

# Build ROS Workspace Attempt
clone_repos
build_ros_ws
source_ros_ws

# Prepare Ardupilot SITL
build_ardupilot

# Return to where we start
cd $START_ABS_PATH
