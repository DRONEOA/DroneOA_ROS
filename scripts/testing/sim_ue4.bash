#!/bin/bash
killbg() {
        for p in "${pids[@]}" ; do
                kill "$p";
        done
}
trap killbg EXIT
cd ../../launch
pids=()
# sim_vehicle will be started by UE4 SIM
echo ">>> Starting dependency nodes ......"
roslaunch ../../launch/sitl/step1SITLUE4.launch > step1log.txt  &
pids+=($!)
echo ">>> 10s interval <<<"
sleep 10
echo ">>> Starting main node ......"
roslaunch ../../launch/step2.launch > step1log.txt  &
pids+=($!)
echo ">>> Waiting for shutdown signal <<<"
sleep infinity
