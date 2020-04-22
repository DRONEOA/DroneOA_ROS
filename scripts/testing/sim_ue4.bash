#!/bin/bash
killbg() {
        for p in "${pids[@]}" ; do
                kill "$p";
        done
}
trap killbg EXIT
pids=()
# sim_vehicle will be started by UE4 SIM
echo ">>> Starting UDP ......"
python ./simple_UDP.py & 
pids+=($!)
echo ">>> Starting dependency nodes ......"
cd ../../launch/sitl
roslaunch ./step1SITLUE4.launch > step1log.txt  &
pids+=($!)
echo ">>> 10s interval <<<"
sleep 10
echo "==================================="
echo ">>>>>     If need octomap     <<<<<"
echo ">>>>>   Start UE4 SIM ASAP    <<<<<"
echo "==================================="
sleep 5
echo "==================================="
echo ">>>>> You Can Start Main Node <<<<<"
echo "==================================="
# cd ..
# roslaunch ./step2.launch > step1log.txt  &
# pids+=($!)
echo ">>> Waiting for shutdown signal <<<"
sleep infinity
