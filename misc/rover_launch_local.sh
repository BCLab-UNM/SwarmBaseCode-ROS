#!/bin/bash
export SWARMATHON_APP_ROOT="$(catkin locate)"
if [ -z "$SWARMATHON_APP_ROOT" ]; then SWARMATHON_APP_ROOT=$(dirname "$0"); fi
roscore &
$SWARMATHON_APP_ROOT/misc/rover_onboard_node_launch.sh localhost $1
