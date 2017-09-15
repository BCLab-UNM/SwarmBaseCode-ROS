#!/bin/bash

if [ -z "$1" ]
then
    export ROS_MASTER_URI=http://$(echo $SSH_CLIENT | cut -d' ' -f 1):11311
else
    export ROS_MASTER_URI=http://$1:11311
fi

if udevadm info /dev/ttyACM0 | grep -q Leonardo; then
  swarmie_dev=/dev/ttyACM0
  ublox_dev=/dev/ttyACM1
else
  swarmie_dev=/dev/ttyACM1
  ublox_dev=/dev/ttyACM0
fi

launchfile=~/rover_workspace/launch/swarmie.launch
if [ -f ./launch/swarmie.launch ]; then
    launchfile=./launch/swarmie.launch
fi

roslaunch $launchfile name:=$(hostname) simulation:=False swarmie_dev:=$swarmie_dev ublox_dev:=$ublox_dev
