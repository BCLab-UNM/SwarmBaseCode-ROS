#!/bin/bash

# start_robots.sh [--prefix <prefix directory>] [--team-prefix <team prefix>] <calibration file name> <team name> <robot list>

launch_delay=10
OPTSTRING=`getopt -l prefix:,team-prefix: -- $0 $@`

if [ $? != 0 ]
then
    echo "bad options"
    exit 1
fi

eval set -- "$OPTSTRING"

prefix=""
while true
do
    case "$1" in
        --prefix ) prefix="$2"; shift 2 ;;
        --team-prefix ) team_prefix="$2"; shift 2 ;;
        -- ) shift ; args="$@";  break ;;
    esac
done

cal_file=$1
team=$2
shift 2
robots="$@"

if [ -z $prefix ]
then
    path=$team_prefix$team
else
    path=$prefix/$team_prefix$team
fi

master=`hostname`
# start the robots
for robot in $robots
do
    echo "starting on $robot"
    echo
	#ssh and run script from rover --WORKS
	gnome-terminal --tab -x bash -c "echo -n -e '\033]0;$robot\007';
		ssh -t swarmie@$robot 'echo 'Running $robot';
        cd $path/misc;
        . /opt/ros/kinetic/setup.bash;
		./rover_onboard_node_launch.sh $master $cal_file;
		exit 1;
		exit 1;
		/bin/bash;' exec $SHELL"

    #   ssh swarmie@$robot ". /opt/ros/kinetic/setup.bash && cd $path/misc && nohup ./rover_onboard_node_launch.sh $master $cal_file &"
    if [ $? != 0 ]
    then
        echo "ERROR: Failed to start ROS nodes on $robot"
    else
        sleep $launch_delay
    fi
    
    echo
    echo "---"
    echo
done
