#!/bin/bash

## Find Workspace path
MAIN_PKT_PATH=$(rospack find droneoa_ros)
cd $MAIN_PKT_PATH
cd ..
cd ..
WORKSPACE_PATH=$(pwd)

## Read Package Name
PACKAGE_NAME=$1

## Delete Package locally
echo "Uninstall Package: $PACKAGE_NAME"
rm -r src/$PACKAGE_NAME

echo "Uninstall Exit with code: $retval"
exit $retval
