#!/bin/bash
roscore &
sleep 10 # give roscore enough time to start
./rover_onboard_node_launch.sh localhost $1
