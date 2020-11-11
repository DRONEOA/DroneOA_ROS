#!/bin/bash

split-window -h

tmux select-window -t 0
ls

tmux select-window -t 1
pwd
