#!/bin/bash
echo "running pkill on old rosnodes"
pkill behaviours_JUAV
pkill diagnostics

source "../devel/setup.bash"
export GAZEBO_MODEL_PATH="../simulation/models"
export GAZEBO_PLUGIN_PATH="../build/gazebo_plugins"

#Point to ROS master on the network
echo "point to ROS master on the network"
if [ -z "$1" ]
then
    echo "Usage: ./rover_onboard_node_launch.sh master_hostname"
    exit 1
else
    export ROS_MASTER_URI=http://$1:11311
fi


if [ -z "$2" ]
then
    echo "No name given starting with current machine hostname: $HOSTNAME"
    name=$HOSTNAME
else
    echo "Name given starting as: $2"
    echo "Starting as simulated JUAV"
    name=$2
fi

#Set prefix to fully qualify transforms for each robot
echo "set prefix to fully qualify transforms for each robot: $name"
rosparam set tf_prefix $name


#Startup ROS packages/processes

echo "Running JUAV launch file"
roslaunch ../launch/JUAV.launch name:=$name &


#Wait for user input to terminate processes
while true; do
    echo "Close all driver processes. [q]"
    read choice;

    if [ "$choice" == "q" ];then
	rosnode kill $name\_BEHAVIOUR
	rosnode kill $name\_DIAGNOSTICS

	exit 1
    fi
done
