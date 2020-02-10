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
git clone http://tuotuogzs.ddns.net/droneoa/ydlidar-x2l-local.git
cd realsense-ros/
git checkout `git tag | sort -V | grep -P "^\d+\.\d+\.\d+" | tail -1`
cd ..
cd ..
