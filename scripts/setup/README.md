# DroneOA Setup Script

## How to use the script(s)
```shell
cd droneoa_ros/scripts/setup
chmod +x *
./test_script.sh
./SITL_setup.sh
```

## FAQ
### sim_vehicle.py command not found:
Please do this:
(for bash)
```shell
	echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
	source ~/.bashrc
```
Or
(for zsh)
```shell
	echo "source /opt/ros/melodic/setup.zsh" >> ~/.zshrc
	source ~/.zshrc
```

