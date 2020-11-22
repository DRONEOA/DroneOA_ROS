#!/bin/bash

## Find Workspace path
MAIN_PKT_PATH=$(rospack find droneoa_ros)
cd $MAIN_PKT_PATH
cd ..
cd ..
WORKSPACE_PATH=$(pwd)

## Read Package Name
PACKAGE_NAME=$1

## Launch
echo "Launch APP: $PACKAGE_NAME"
cd src
cd $PACKAGE_NAME
FILE=./start.sh
if [ -f "$FILE" ]; then
    echo "Launch Using Script..."
    bash ./start.sh
else
    echo "Launch Using Launch File..."
    roslaunch ./launch/start.launch
fi

## Determine whether success
retval=$?
echo "Launch Exit with code: $retval"
exit $retval
