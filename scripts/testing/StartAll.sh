#!/bin/bash

## Use tmux, if not installed, install
sudo apt install tmux -y

## Split 2 window
tmux new-session -d bash
tmux split-window -h bash
#sends keys to first and second terminals
tmux send -t 0:0.0 "cd ../../launch/sitl ; roslaunch ./step1SITL.launch" C-m
tmux send -t 0:0.1 "cd ../../launch ; roslaunch ./StartCore.launch" C-m
tmux -2 attach-session -d

