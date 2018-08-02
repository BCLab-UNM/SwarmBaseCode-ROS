#!/bin/bash
echo "running pkill on old rosnodes"
pkill behaviours_JUAV

source "../devel/setup.bash"
export GAZEBO_MODEL_PATH="../simulation/models"
export GAZEBO_PLUGIN_PATH="../build/gazebo_plugins"

#Point to ROS master on the network
echo "point to ROS master on the network"
if [ -z "$1" ]
then
    echo "Usage: ./rover_onboard_node_launch.sh master_hostname calibration_location"
    exit 1
else
    export ROS_MASTER_URI=http://$1:11311

fi


#Set prefix to fully qualify transforms for each robot
echo "set prefix to fully qualify transforms for each robot: $HOSTNAME"
rosparam set tf_prefix $HOSTNAME



#Startup ROS packages/processes

echo "rosrun behaviours"
nohup > logs/$HOSTNAME"_behaviours_log.txt" rosrun behaviours JUAV_behaviours &

echo "rosrun diagnostics"
nohup > logs/$HOSTNAME"_diagnostics_log.txt" rosrun diagnostics diagnostics &

#roslaunch "mavros.launch"


#Wait for user input to terminate processes
while true; do
    echo "Close all driver processes. [q]"
    read choice;

    if [ "$choice" == "q" ];then
	rosnode kill $HOSTNAME\_BEHAVIOUR

	exit 1
    fi
done
