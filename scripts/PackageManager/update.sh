#!/bin/bash

## Find Workspace path
MAIN_PKT_PATH=$(rospack find droneoa_ros)
cd $MAIN_PKT_PATH
cd ..
cd ..
WORKSPACE_PATH=$(pwd)

## Read Package Name
PACKAGE_NAME=$1

## Pull Latest
echo "Update Package: $PACKAGE_NAME"
cd src/$PACKAGE_NAME
git pull

echo "Update Exit with code: $retval"
exit $retval
