#!/bin/bash
source ~/rover_workspace/devel/setup.bash
echo Cleaning up ROS and Gazebo Processes
./cleanup.sh
echo Killing rosmaster
pkill rosmaster
echo Killing roscore
pkill roscore
roscore &
sleep 2
rqt -s rqt_rover_gui
# The rover program cleans up after itself but if there is a crash this helps to make sure there are no leftovers
echo Cleaning up ROS and Gazebo Processes
rosnode kill -a 
echo Killing rosmaster
pkill rosmaster
echo Killing roscore
pkill roscore
./cleanup.sh
