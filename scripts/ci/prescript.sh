echo 'deb http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo xenial main' | tee /etc/apt/sources.list.d/realsense-public.list
apt-key add ./src/droneoa_ros/scripts/realsenseKey
apt-get update
apt-get install librealsense2-dkms --allow-unauthenticated -y
apt-get install librealsense2-dev --allow-unauthenticated -y
apt-get install ros-melodic-cv-bridge -y
apt-get install ros-melodic-image-transport
apt-get install ros-melodic-tf -y
apt-get install ros-melodic-diagnostic-updater -y
apt-get install ros-melodic-ddynamic-reconfigure -y
apt-get install ros-melodic-nodelet -y
apt-get install ros-melodic-perception -y
apt-get install ros-melodic-pcl-ros -y
apt-get install ros-melodic-pcl-conversions -y
cd src
git clone https://github.com/IntelRealSense/realsense-ros.git
git clone http://gitlab.tuotuogzs.com/droneoa/ydlidar-x2l-local.git
cd realsense-ros/
git checkout `git tag | sort -V | grep -P "^\d+\.\d+\.\d+" | tail -1`
cd ..
cd ..
