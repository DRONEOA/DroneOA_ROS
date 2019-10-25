cd ~
git clone https://github.com/ArduPilot/ardupilot
cd ardupilot
git submodule update --init --recursive
sudo Tools/environment_install/install-prereqs-ubuntu.sh -y
. ~/.profile
sudo apt-get install libxml2-dev libxslt-dev python-dev -y
sudo -H pip2 install -U future lxml
sudo -H pip2 install -U pymavlink
sudo apt-get install python3-dev python3-opencv python3-pip python3-matplotlib -y
sudo -H pip install MAVProxy
make CubeBlack
