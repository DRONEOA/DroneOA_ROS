#!/bin/bash

## Find Workspace path
MAIN_PKT_PATH=$(rospack find droneoa_ros)
cd $MAIN_PKT_PATH
cd ..
cd ..
echo "Rebuild All Packages"
catkin_make
retval=$?
echo "Rebuild Exit with code: $retval"
exit $retval
