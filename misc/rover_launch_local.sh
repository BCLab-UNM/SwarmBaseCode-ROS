#!bin/bash
roscore &
./rover_onboard_node_launch.sh localhost $1
