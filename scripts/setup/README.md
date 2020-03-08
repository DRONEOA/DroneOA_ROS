# DroneOA Setup Script

## How to use the environment setup script
```shell
cd droneoa_ros/scripts/setup
chmod +x *
./ENV_script.sh
```
## How to start an ArduCopter instance
```shell
cd ~
cd <Ardupilot_Path>/ArduCopter
sim_vehicle.py -w
```
## FAQ
### sim_vehicle.py command not found:
Solutions:
- Rebooting / Sign out your computer.
- If still not found, you can find this script here: `<Ardupilot_Path>/Tools/autotest/sim_vehicle.py`
### Asking for repo path in the environment setup script
The path is absolute path
