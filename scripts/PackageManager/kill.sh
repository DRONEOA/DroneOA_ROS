#!/bin/bash

## Node name to kill
NODE_NAME=$1

## Check if node running
if ! pgrep -x $NODE_NAME > /dev/null
then
    echo "Node Not Running / Not Exist"
    exit -1
fi

## Find node pid
NODE_PID=$(pgrep -x $NODE_NAME)

## Kill
kill $NODE_PID
