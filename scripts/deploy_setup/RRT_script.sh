#!/bin/bash
mkdir RRTInstallDownload
cd RRTInstallDownload
ROSDIST="melodic"
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
echo "---------- Install OMPL ----------"
sudo apt-get install libompl-dev ompl-demos -y
wget https://ompl.kavrakilab.org/install-ompl-ubuntu.sh
sudo chmod u+x install-ompl-ubuntu.sh
./install-ompl-ubuntu.sh
sudo cp -r /usr/local/include/ompl-1.5/ompl /usr/local/include/ompl
# sudo rm -r /usr/local/include/ompl-1.5
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list'
wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
sudo apt-get update
sudo apt-get install ros-${ROSDIST}-ompl -y
echo "---------- Install FCL ----------"
sudo apt install libccd-dev -y
sudo apt install liboctomap-dev -y
sudo apt install ros-${ROSDIST}-octomap -y
sudo apt install ros-${ROSDIST}-octomap-server -y
sudo apt install ros-${ROSDIST}-octomap-msgs -y
git clone https://github.com/flexible-collision-library/fcl.git
cd fcl
git checkout 0.6.1
mkdir build
cd build
cmake ..
sudo make install
echo "----------  All Done  -----------"
