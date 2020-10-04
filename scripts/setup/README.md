# DroneOA Setup Script

## How to use the environment setup script
```shell
cd droneoa_ros/scripts/setup
chmod +x *
./ENV_script.sh
```
### After The Installation [Important]
Go to the generated ROS workspace: `<Input Worspace Path>/ardupilot_ws`
1. Try to **build** the workspace: `catkin_make`
2. After a successful build. **Source** the setup bash/zsh:
    - For zsh user: `echo "source <Input Worspace Path>/ardupilot_ws/devel/setup.zsh" >> ~/.zshrc`
    - For bash user: `echo "source <Input Worspace Path>/ardupilot_ws/devel/setup.bash" >> ~/.bashrc`
    - Reopen terminal OR `source ~/.bashrc` / `source ~/.zshrc`

In addition:
1. Run `. ~/.profile` to allow use command `sim_vehicle.py` anywhere.

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
- Rebooting / Sign out your computer.
- If still not found, you can find this script here: `<Ardupilot_Path>/Tools/autotest/sim_vehicle.py`
### Asking for repo path in the environment setup script
The path is absolute path, run `pwd` in the desired Workspace folder to check the absolute path
