#!/bin/bash

OPTION=$1

roverpass=KSC-2018
rover=swarmie
roverIP="r02"

#GOALS
#SSH and begin ssh protocol back to workstation
#Transfer Files to Rovers

#steps
# 1.) ssh into requested rover
# 2.) navigate and trigger the rover node script


#find users current IP and store it
userIP=$(ifconfig  | grep 'inet addr:'| grep -v '127.0.0.1' | cut -d: -f2 | awk '{ print $1}')
hostName=$(hostname)

#No Connection
#-------------------------------------------------------------
if [ -z "$userIP" ]; then
	echo "No Current Internet Connection Found"
	echo "Cannot SSH into rover without IP Address!"
	exit 1
fi

#No Command Given
#-------------------------------------------------------------
if [ -z $OPTION ]; then
	echo "-------------------------------------------------------------"
	echo "Please run this script with the following commands:"
	echo "-G {repository name} {branch} to pull and transfer workspace to swarmie(s)"
	echo "-F for duplicating current workspace and transferring to swarmie(s)"
	echo "-R to run current code on all swarmie(s)"
	echo "-------------------------------------------------------------"
	exit 1
fi

clear

echo "Your network info:  $hostName@$userIP"

#check OPTION given
#------------------------------------------------------------

#GitHub Selection
#------------------------------------------------------------
if [ $OPTION == "-G" ]; then

	#if not everything is filled out
	if [ $# -ne 3 ]; then
    		echo "Usage -G: {repository name} {branch}"
	    	exit 1
	fi

	cd ~

	if [ -z "$(ls -A $2/src/ublox/)" ]; then
  		echo "The ublox submodule is missing."
  		exit 1
	fi

	if [ -z "$(ls -A $2/src/apriltags_ros/)" ]; then
  		echo "The apriltags submodule is missing."
  		exit 1
	fi

	#ping -q -w2 $3 > /dev/null
	#if [ $? -eq 0 ]; then
	#echo "Found $2"
	#cd ~/SwarmBaseCode-ROS/

	#Tell User Option

	echo "Retrieving and Transferring Code from GitHub:  "
	echo ""
	echo "dir: $2 branch: $3"
	echo "-------------------------------------------------------------"
	echo ""

	gnome-terminal -x bash -c "cd $2;
		echo 'Checking out branch $3';
		git checkout $3;
		sleep 2;
		echo 'Pulling latest build on branch $3';
		git pull;
		sleep 2;
		echo 'Compiling $2 branch $3...';
		catkin build;
		sleep 2;
		cd -;
		echo 'Packing up the repository... ';
		tar czf $2.tgz $2;
		sleep 2; 
		exit 1; 
		/bin/bash;' exec $SHELL"

	if [ true ]; then
		while(true); do
		read -p "Rover Name/IP To Start (Type 'done' to finish):  " roverIP
			if [ "$roverIP" =  "done" ]; then
				break	
			fi

			echo "-------------------------------------------------------------"
			echo "Uploading to $roverIP"
			echo ""
			echo "Copying compressed repository to swarmie at $roverIP"
			scp $2.tgz swarmie@$roverIP:~
			sleep 2
			echo "Unpacking repository $2 and branch $3 on swarmie at $roverIP"
			ssh swarmie@$roverIP "tar xzf $2.tgz"
			echo "Starting ROS nodes on swarmie at $3 with master at $hostName"

			#ssh and run script from rover --WORKS
			gnome-terminal -x bash -c "ssh -t $rover@$roverIP 'cd SwarmBaseCode-ROS/misc;./rover_onboard_node_launch.sh $hostName;
				exit 1;
				/bin/bash;' 
				exec $SHELL"

			sleep 10

		done
	fi

	exit 1
fi

#Transfer Local Copy
#-------------------------------------------------------------
if [ $OPTION == "-F" ]; then
	echo "Copying and transferring current folder to swarmie(s)"
	echo ""
	echo "dir: $2"
	echo "-------------------------------------------------------------"
	echo ""


	#if not everything is filled out
	if [ $# -ne 2 ]; then
    		echo "Usage -F: {folder name}"
	    	exit 1
	fi

	gnome-terminal -x bash -c "cd ~/$2;
		echo 'Compiling $2 branch $3...';
		catkin build;
		sleep 2;
		cd ~;
		echo 'Packing up the repository... ';
		tar czf $2.tgz $2;
		sleep 2; 
		exit 1; 
		/bin/bash;' exec $SHELL"

	if [ true ]; then
		while(true); do
		read -p "Rover Name/IP To Start (Type 'done' to finish):  " roverIP
			if [ "$roverIP" =  "done" ]; then
				break	
			fi

			echo "-------------------------------------------------------------"
			echo "Uploading to $roverIP"
			echo ""
			echo "Copying compressed repository to swarmie at $roverIP"
			scp $2.tgz swarmie@$roverIP:~
			sleep 2
			echo "Unpacking repository $2 on swarmie at $roverIP"
			gnome-terminal -x bash -c "ssh -t swarmie@$roverIP 'echo 'Unpacking $2.tgz ...';
				tar xzf $2.tgz;
				sleep 2;			
				echo 'Starting ROS nodes on swarmie at $roverIP with master at $hostName';
				sleep 2;
				cd $2/misc/;
				./rover_onboard_node_launch.sh $hostName;
				exit 1;
				/bin/bash;' exec $SHELL"
				
			echo "Starting ROS nodes on swarmie at $3 with master at $hostName"
			echo ""
			#ssh and run script from rover --WORKS
			#gnome-terminal -x bash -c "ssh -t swarmie@$roverIP 'cd SwarmBaseCode-ROS/misc;./rover_onboard_node_launch.sh 					$hostName;exit 1;/bin/bash;' exec $SHELL"

			sleep 10


		done	
	fi
fi

#Just Run
#-------------------------------------------------------------
if [ $OPTION == "-R" ]; then
	echo "Running current code on all swarmie(s)"
	echo "-------------------------------------------------------------"
	echo ""

	if [ true ]; then
		while(true); do
		read -p "Rover Name/IP To Start (Type 'done' to finish):  " roverIP
			if [ "$roverIP" =  "done" ]; then
				break	
			fi

		echo "-------------------------------------------------------------"
		echo "Starting $roverIP"

		#ssh and run script from rover --WORKS
		gnome-terminal -x bash -c "ssh -t $rover@$roverIP 'cd SwarmBaseCode-ROS/misc;./rover_onboard_node_launch.sh $hostName;exit 1;/bin/bash;' exec $SHELL"

		sleep 10

		done
	fi
fi
