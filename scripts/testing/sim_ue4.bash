#!/bin/bash
killbg() {
        for p in "${pids[@]}" ; do
                kill "$p";
        done
}
trap killbg EXIT
cd ../../launch
pids=()
# sim_vehicle doesn't work in background mode. Need to find a solution later
# sim_vehicle.py -v ArduCopter &
# pids+=($!)
# sleep 15
roslaunch ./step1SITLUE4.launch > step1log.txt  &
pids+=($!)
sleep 10
echo "UDP python"
python ../scripts/simple_UDP.py & 
pids+=($!)
sleep infinity
