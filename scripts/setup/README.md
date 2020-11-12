# DroneOA Setup Script

## How to use the environment setup script
```shell
cd droneoa_ros/scripts/setup
chmod +x *
./ENV_script.sh
```
### After The Installation [Important]
Go to the generated ROS workspace: `<User Worspace Path>/<User ROS Workspace Name>`
1. Try to **rebuild** the workspace: `catkin_make clean; catkin_make`
    - If `catkin_make` is not found, reopen terminal OR `source ~/.bashrc` / `source ~/.zshrc`
2. Run `. ~/.profile` to allow use command `sim_vehicle.py` anywhere.
3. Check the installation is complete:
    - Open a terminal with 3 tabs, then try the following in order.
    - Tab1: `sim_vehicle.py -v ArduCopter`
    - Tab2:
      - Navigate to droneoa_ros package path: `roscd dronsoa_ros`
        - If command not found, manually navigate to `<User Worspace Path>/<User ROS Workspace Name>/src/dronsoa_ros`
      - Enter SITL launch dependencies folder: `cd launch/sitl`
      - Run `roslaunch ./step1SITL.launch`
    - Tab3:
      - Navigate to droneoa_ros package path: `roscd dronsoa_ros`
        - If command not found, manually navigate to `<User Worspace Path>/<User ROS Workspace Name>/src/dronsoa_ros`
      - Enter launch folder: `cd launch`
      - Run `roslaunch ./step2.launch`

## How to start an ArduCopter instance
For the first time:
```shell
cd ~
cd <Ardupilot_Path>/ArduCopter
sim_vehicle.py -w
```
After that:
```shell
sim_vehicle.py -v ArduCopter
```
You can add `--console` for mavproxy console, and `--map` for satellite map view

## FAQ
### sim_vehicle.py command not found:
Solutions:
- Run `. ~/.profile`
- If still not found, Rebooting / Sign out your computer.
- If still not found, you can find this script here: `<Ardupilot_Path>/Tools/autotest/sim_vehicle.py`
