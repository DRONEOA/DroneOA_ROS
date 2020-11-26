#!/bin/bash

## Find Workspace path
MAIN_PKT_PATH=$(rospack find droneoa_ros)
cd $MAIN_PKT_PATH
cd ..
cd ..
WORKSPACE_PATH=$(pwd)

## Read Package Name
PACKAGE_NAME=$1

## Shutdown
echo "Shutdown APP: $PACKAGE_NAME"
cd src
cd $PACKAGE_NAME
FILE=./shutdown.sh
if [ -f "$FILE" ]; then
    echo "Shutdown Using Script..."
    bash ./shutdown.sh
else
    echo "Cannot Find Shutdown Script"
    exit -1
fi

## Determine whether success
retval=$?
echo "Shutdown Exit with code: $retval"
exit $retval
