cmake_minimum_required(VERSION 2.8.3)
project(droneoa_ros)

# Set target C++ standard and required compiler features
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++14" COMPILER_SUPPORTS_CXX14)
if(COMPILER_SUPPORTS_CXX14)
    set(CMAKE_CXX_STANDARD 14)
else()
    set(CMAKE_CXX_STANDARD 11)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  nodelet
  mavros
  roscpp
  rospy
  std_msgs
  sensor_msgs
  cv_bridge
  message_generation
)

find_package( OpenCV REQUIRED )
find_package( PCL REQUIRED )
find_package( ompl REQUIRED )
find_package( fcl REQUIRED )
find_package( octomap REQUIRED )
find_package(PkgConfig REQUIRED)
pkg_check_modules(JSONCPP jsoncpp)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  DataPoolPair.msg
  DataPoolSync.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  CheckGetNewInput.srv
  GetDPDataAsString.srv
  GetDPData.srv
  SetDPData.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES droneoa_ros
  CATKIN_DEPENDS nodelet mavros roscpp rospy std_msgs sensor_msgs cv_bridge message_runtime
#  DEPENDS system_lib
  DEPENDS OMPL
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${FCL_INCLUDE_DIRS}
  ${ompl_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${JSONCPP_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(droneoa_ros_libutils
  src/Utils/GeneralUtils.cpp
  src/Utils/DataPool.cpp
  src/Utils/DataPoolClient.cpp
  src/Utils/DataPoolServer.cpp
  src/Utils/DataPoolSubscriber.cpp
  src/HWI/Utils/CNCUtils.cpp
  src/HWI/Utils/GPSPoint.cpp
  src/HWI/Utils/LocalPoint.cpp
  src/HWI/Utils/Position3D.cpp
  src/HWI/Utils/LidarUtils.cpp
  src/Utils/JsonUtils.cpp
  src/Utils/ConfChangedListener.cpp
)

add_dependencies(droneoa_ros_libutils droneoa_ros_generate_messages_cpp)

add_library(droneoa_ros_liboac
  src/OAC/BaseAlg.cpp
  src/OAC/CAAlgLidar.cpp
  src/OAC/CAAlgDepthCam.cpp
  src/OAC/OAAlgFGM.cpp
  src/OAC/OAAlgRRT.cpp
  src/OAC/CMDParser.cpp
  src/OAC/CMDRunner.cpp
  src/OAC/Command.cpp
  src/OAC/OAC.cpp
  src/OAC/OMPLPlanner.cpp
)

add_dependencies(droneoa_ros_liboac droneoa_ros_generate_messages_cpp)

add_library(droneoa_ros_libhwi
  src/HWI/base/CNCGeneric.cpp
  src/HWI/base/LidarGeneric.cpp
  src/HWI/CNCArdupilot.cpp
  src/HWI/LidarYDLidar.cpp
  src/HWI/RSC.cpp
  src/HWI/ConsoleInputManager.cpp
)

add_dependencies(droneoa_ros_libhwi droneoa_ros_generate_messages_cpp)

add_library(droneoa_ros_libgui
  src/GUI/Release/WebGUIServer.cpp
  src/GUI/Release/UE4SIMServer.cpp
  src/GUI/Debug/LidarPopup.cpp
  src/GUI/Debug/RSCPopup.cpp
  src/GUI/Debug/CNCPopup.cpp
  src/GUI/GUISubscriber.cpp
  src/GUI/GUISubject.cpp
)

add_dependencies(droneoa_ros_libgui droneoa_ros_generate_messages_cpp)

add_library(droneoa_ros_libpm
  src/PackageManager/PMCommandParser.cpp
  src/Utils/GeneralUtils.cpp
)

add_dependencies(droneoa_ros_libpm droneoa_ros_generate_messages_cpp)

add_library(droneoa_ros_libconsole
  src/ConsoleService/ConsoleService.cpp
  src/Utils/GeneralUtils.cpp
)

add_dependencies(droneoa_ros_libconsole droneoa_ros_generate_messages_cpp)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/droneoa_ros_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")
add_executable(droneoa_ros
  src/main.cpp
)

add_executable(droneoa_ros_package_manager
  src/PackageManager/PackageManagerNode.cpp
)

add_executable(droneoa_ros_console_server
  src/ConsoleService/ConsoleServiceNode.cpp
)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(droneoa_ros
  droneoa_ros_libhwi
  droneoa_ros_libutils
  droneoa_ros_liboac
  droneoa_ros_libgui
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
  ${FCL_LIBRARIES}
  ${OMPL_LIBRARIES}
  ${OCTOMAP_LIBRARIES}
  readline
  ${JSONCPP_LIBRARIES}
)

target_link_libraries(droneoa_ros_package_manager
  droneoa_ros_libpm
  ${catkin_LIBRARIES}
  readline
)

target_link_libraries(droneoa_ros_console_server
  droneoa_ros_libconsole
  ${catkin_LIBRARIES}
  readline
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_lib
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

install(
	TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_libhwi ${PROJECT_NAME}_libutils ${PROJECT_NAME}_liboac ${PROJECT_NAME}_libgui
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_droneoa_ros.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

# Utils Unittests
add_executable(droneoa_ros_utiltest
  test/utils_unittest/utils_unittest.cpp
  src/Utils/GeneralUtils.cpp
  src/HWI/Utils/CNCUtils.cpp
  src/HWI/Utils/GPSPoint.cpp
  src/HWI/Utils/LocalPoint.cpp
  src/HWI/Utils/Position3D.cpp
)

target_link_libraries(droneoa_ros_utiltest ${catkin_LIBRARIES} gtest)

# Command Unittests
add_executable(droneoa_ros_commandtest
  test/command_unittest/test_main.cpp
  test/command_unittest/command_tests.cpp
  test/command_unittest/test_suite.cpp
  src/OAC/Command.cpp
  src/Utils/GeneralUtils.cpp
  src/HWI/Utils/GPSPoint.cpp
  src/HWI/Utils/LocalPoint.cpp
  src/HWI/Utils/Position3D.cpp
  src/HWI/Utils/CNCUtils.cpp
  src/GUI/GUISubscriber.cpp
  src/GUI/GUISubject.cpp
)

target_link_libraries(droneoa_ros_commandtest
  ${catkin_LIBRARIES}
  gtest
  gmock
)
