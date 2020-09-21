apt-key adv --keyserver keys.gnupg.net --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
echo 'deb http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo bionic main' | tee /etc/apt/sources.list.d/realsense-public.list
apt-get update
apt-get install librealsense2-dkms --allow-unauthenticated -y
apt-get install librealsense2-dev --allow-unauthenticated -y
apt-get install libreadline7 libreadline-dev -y
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
git checkout 2.2.17
cd ..
cd ..
