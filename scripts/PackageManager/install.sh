#!/bin/bash

## Find Workspace path
MAIN_PKT_PATH=$(rospack find droneoa_ros)
cd $MAIN_PKT_PATH
cd ..
cd ..
WORKSPACE_PATH=$(pwd)

## Read Package Name
PACKAGE_NAME=$1

## Read Input URL
PACKAGE_URL=$2

echo "DroneOA Path: $MAIN_PKT_PATH"
echo "Workspace Path: $WORKSPACE_PATH"
echo "Package URL: $PACKAGE_URL"

## Pull package with git
cd src
git clone $PACKAGE_URL $PACKAGE_NAME

## Run Package's install script
cd $PACKAGE_NAME
# cd "$(basename $PACKAGE_URL .git)"  # Find Folder Name
FILE=./install.sh
if [ -f "$FILE" ]; then
    echo "RUN Package Installation Script..."
    bash ./install.sh
fi

retval=$?
echo "Installation Exit with code: $retval"
exit $retval
