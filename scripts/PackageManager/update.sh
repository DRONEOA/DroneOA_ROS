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

## If specified branch
if [ -z "$2" ]
then
    echo "Checkout Branch: $(git branch --show-current)"
else
    echo "Checkout Branch: $2"
    cd ./$PACKAGE_NAME
    git checkout $2
    git pull
fi

retval=$?
echo "Update Exit with code: $retval"
exit $retval
