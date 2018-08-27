#!/bin/bash

#Point to ROS master on the network
echo "point to ROS master on the network"
if [ -z "$1" ]
then
    echo "Usage: ./rover_onboard_node_launch.sh master_hostname"
    exit 1
else
    export ROS_MASTER_URI=http://$1:11311

fi



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


#rosrun gazebo_ros spawn_model -sdf -file SwarmBaseCode-ROS/simulation/models/iris_with_standoffs_demo/model.sdf -model 1
