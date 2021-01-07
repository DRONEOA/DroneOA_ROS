#!/bin/bash
echo "---------- $0 start ----------"
set -e

function print_usage()
{
    echo "Usage: $0 MODE"
    echo "    MODE: sitl; ngn; gps; ue4"
    exit 1
}

if [ "$#" -ne 1 ]; then
    print_usage
fi

MODE=$1
CONF_NAME="droneoa_sitl.yml"

if [ "$MODE" = "sitl" ]; then
    CONF_NAME="droneoa_sitl.yml"
elif [ "$MODE" = "ngn" ]; then
    CONF_NAME="droneoa_ngn.yml"
elif [ "$MODE" = "gps" ]; then
    CONF_NAME="droneoa_gps.yml"
elif [ "$MODE" = "ue4" ]; then
    CONF_NAME="droneoa_ue4.yml"
else
    print_usage
fi

# Find package and ROS workspace path
MAIN_PKT_PATH=$(rospack find droneoa_ros)
cd $MAIN_PKT_PATH
cd ..
ROS_WORKSPACE_PATH=$(pwd)

echo "Config Path: $MAIN_PKT_PATH/scripts/tmux/$CONF_NAME"

#install catmux
sudo apt install tmux
pip3 install --user catmux
pip3 install rospkg

# Launch with catmux
catmux_create_session $MAIN_PKT_PATH/scripts/tmux/$CONF_NAME
