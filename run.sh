#!/bin/bash
previous_gazebo_model_path=${GAZEBO_MODEL_PATH}
previous_gazebo_plugin_path=${GAZEBO_PLUGIN_PATH}
export SWARMATHON_APP_ROOT="$(catkin locate)"
if [ -z "$SWARMATHON_APP_ROOT" ]; then SWARMATHON_APP_ROOT=$(dirname "$0"); fi
echo "Running from: $PWD with repo root: $SWARMATHON_APP_ROOT"
export GAZEBO_MODEL_PATH="$SWARMATHON_APP_ROOT/simulation/models"
export GAZEBO_PLUGIN_PATH="$SWARMATHON_APP_ROOT/build/gazebo_plugins"
source "$SWARMATHON_APP_ROOT/devel/setup.bash"
echo Cleaning up ROS and Gazebo Processes

#Delete the rqt cache - can take 24 hours for changes in the UI
# to show up otherwise
rm ~/.config/ros.org/rqt_gui.ini

$SWARMATHON_APP_ROOT/cleanup.sh
echo Killing rosmaster
pkill rosmaster
echo Killing roscore
pkill roscore
roscore &
sleep 2
roslaunch rosbridge_server rosbridge_websocket.launch &
rqt -s rqt_rover_gui
# The rover program cleans up after itself but if there is a crash this helps to make sure there are no leftovers
echo Cleaning up ROS and Gazebo Processes
rosnode kill -a 
echo Killing rosmaster
pkill rosmaster
echo Killing roscore
pkill roscore
$SWARMATHON_APP_ROOT/cleanup.sh
# Restore previous environment
export GAZEBO_MODEL_PATH=$previous_gazebo_model_path
export GAZEBO_PLUGIN_PATH=$previous_gazebo_plugin_path
