#!/bin/bash
source ~/rover_workspace/devel/setup.bash
echo Cleaning up ROS and Gazebo Processes
src/rover_driver_gazebo_launch/cleanup.sh
echo Killing rosmaster
pkill rosmaster
echo Killing roscore
pkill roscore
roscore &
sleep 2
rqt -s rqt_rover_gui
echo Cleaning up ROS and Gazebo Processes
echo Killing rosmaster
pkill rosmaster
echo Killing roscore
pkill roscore
src/rover_driver_gazebo_launch/cleanup.sh
