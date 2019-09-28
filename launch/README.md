# How To Use Launch File:

## With Real Vehicle:

### Config realsense camera port number
Find out realsense camera serial number:
```shell
rs-enumerate-devices
```
Record ``Serial Number`` for both t265 tracking camera anf d435 depth camera.

Modify ``rs_camera_t265_d435.launch`` file with correct serial number:
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
roslaunch step2.launch
```

Note: If you are performing a test flight, you may want to open above nodes in 2 seperate ssh session


## With SITL:
If you DO NOT use t265 tracking camera for navigation data, skip to **Launch SITL Instance**

### Config realsense camera port number
Find out realsense camera serial number:
```shell
rs-enumerate-devices
```
Record ``Serial Number`` for both t265 tracking camera anf d435 depth camera.

Modify ``rs_camera_t265_d435.launch`` file with correct serial number:
```xml
<arg name="serial_no_camera1"    			default="909212110178"/> 			<!-- Note: Replace with actual serial number -->
<arg name="serial_no_camera2"    			default="841512070232"/> 			<!-- Note: Replace with actual serial number -->
```
### Launch SITL Instance
```shell
sim_vehicle.py -v ArduCopter --console --map
```
### Launch Dependency Nodes
If use t265 tracking camera for nav data:
```shell
roslaunch step1SITL_t265.launch
```
else:
```shell
roslaunch step1SITL.launch
```
### Launch Main Nodes
```shell
roslaunch step2.launch
```