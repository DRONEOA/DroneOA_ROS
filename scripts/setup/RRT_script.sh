#!/bin/bash
mkdir RRTInstallDownload
cd RRTInstallDownload
echo "---------- Install OMPL ----------"
wget https://ompl.kavrakilab.org/install-ompl-ubuntu.sh
sudo chmod u+x install-ompl-ubuntu.sh
./install-ompl-ubuntu.sh
sudo cp -r /usr/local/include/ompl-1.5/ompl /usr/local/include/ompl
sudo rm -r /usr/local/include/ompl-1.5
echo "---------- Install FCL ----------"
git clone https://github.com/flexible-collision-library/fcl.git
cd fcl
git checkout 0.6.1
mkdir build
cd build
cmake ..
sudo make install
echo "----------  All Done  -----------"
