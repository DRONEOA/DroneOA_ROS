# DroneOA Setup Script

## How to use the environment setup script

**With full repo:**
```shell
cd droneoa_ros/scripts/deploy_setup
chmod +x *
source ./deploy_ardupilot.sh
```

**Note:** with release script zip, simply
```shell
cd /release/deploy_setup
chmod +x *
source ./deploy_ardupilot.sh
```

### After The Installation [Important]
Go to the generated ROS workspace: `<User Worspace Path>/<User ROS Workspace Name>`
1. Check the installation is complete through the summary table
2. Try to **rebuild** the workspace: `catkin_make clean; catkin_make`
    - If `catkin_make` is not found, reopen terminal OR `source ~/.bashrc` / `source ~/.zshrc`

## Read The Wiki
Checkout the official wiki for user instructions:
- http://droneoa.tuotuogzs.net/droneoa_gitbook/

## FAQ
### System program problem detected
Solution:
- This is normal
- You can disable this warning by modifying `enabled=0` in this file `sudo nano /etc/default/apport`
