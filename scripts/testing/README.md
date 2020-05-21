# DroneOA Testing Script

## Test with UE4 SIM
1. **Start UE4 SIM** (click the start button in the UE4 Editor)
   - Note: This will start both a "SITL instance" and "simple_udp bridge"
2. **Wait** for the connection indicator in UE4 to be **Soild Green**
   - Should usually take around 15s
   - Note: do not close any popups unless on shutdown
3. **Execute the `sim_ue4.bash` script**
   - Note: This will start basic dependencies, "ROS TCP bridge", "dapthImg to PointCloud2 convertor" and "DroneOA main node"
