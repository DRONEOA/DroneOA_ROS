# How To Use Launch File:

## With Real Vehicle:

### Config realsense camera port number
Find out realsense camera serial number:
```shell
rs-enumerate-devices
```
Record ``Serial Number`` for both t265 tracking camera anf d435 depth camera.

Modify ``component/rs_camera_t265_d435.launch`` file with correct serial number:
```xml
<arg name="serial_no_camera1"    			default="909212110178"/> 			<!-- Note: Replace with actual serial number -->
<arg name="serial_no_camera2"    			default="841512070232"/> 			<!-- Note: Replace with actual serial number -->
```
### Launch Dependency Nodes
```shell
roslaunch step1.launch
```
### Launch Main Nodes
```shell
roslaunch StartCore.launch
```
This will launch `console service` and `package manager`, more instructions will be displayed in the terminal launch message

For example: use `pm launch main` to launch the main drone control node

Note: If you are performing a test flight, you may want to open above nodes in 2 seperate ssh session


## With SITL:
If you DO NOT use t265 tracking camera for navigation data, skip to **Launch SITL Instance**

### Config realsense camera port number
Find out realsense camera serial number:
```shell
rs-enumerate-devices
```
Record ``Serial Number`` for both t265 tracking camera anf d435 depth camera.

Modify ``component/rs_camera_t265_d435.launch`` file with correct serial number:
```xml
<arg name="serial_no_camera1"    			default="909212110178"/> 			<!-- Note: Replace with actual serial number -->
<arg name="serial_no_camera2"    			default="841512070232"/> 			<!-- Note: Replace with actual serial number -->
```
### Launch SITL Instance
```shell
sim_vehicle.py -v ArduCopter --console --map
```
### Launch Dependency Nodes
Enter lauch file folder:
```shell
cd <workspace>/src/droneoa_ros/launch
```
If use t265 tracking camera for nav data:
```shell
roslaunch ./sitl/step1SITL_t265.launch
```
else:
```shell
roslaunch ./sitl/step1SITL.launch
```
### Launch Main Nodes
Enter lauch file folder:
```shell
cd <workspace>/src/droneoa_ros/launch
```
Launch the node in separate terminal to allow dedicated console input:
```shell
roslaunch StartCore.launch
```
This will launch `console service` and `package manager`, more instructions will be displayed in the terminal launch message

For example: use `pm launch main` to launch the main drone control node

## FAQs

Q: I cannot launch after doing a sudo upgrade. The error contains something like 
```shell
API version mismatch: librealsense.so was compiled with API version 2.XX.0 but the application was compiled with 2.XX.0
```
A: You likely need to update the ``realsence-ros`` repo.
* Enter ``realsence-ros``
* Checkout the ``developer`` branch if you did not do so
* Perform git pull.
* Rebuild all packages by ``catkin_make`` in you workspace root
