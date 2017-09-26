#!/bin/bash

OPTION=$1

roverpass=""
roverIP=""

cd ..
dirPath="$(pwd)"
dirName="$(basename `pwd`)"

#GOALS
#SSH and begin ssh protocol back to workstation
#Transfer Files to Rovers

#steps
# 1.) ssh into requested rover
# 2.) navigate and trigger the rover node script


#find users current IP and store it
userIP=$(ifconfig  | grep 'inet addr:'| grep -v '127.0.0.1' | cut -d: -f2 | awk '{ print $1}')
hostName=$(hostname)

echo "$dirPath"
echo "$dirName"

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
	echo "-G {branch} pulls and transfers workspace to swarmie(s)"
	echo "-F duplicates current workspace and transfers to swarmie(s)"
	echo "-R to run current code on all swarmie(s)"
	echo "-------------------------------------------------------------"
	exit 1
fi

clear

echo "Your network info:  $hostName@$userIP"

#check OPTION given
#------------------------------------------------------------

while(true); do

#GitHub Selection
#------------------------------------------------------------
if [ $OPTION == "-G" ]; then

	branch=$2

	#if not everything is filled out
	if [ "$branch"="" ]; then
		echo ""
    		read -p "Branch?:  " branch
		echo ""
	fi

	cd ~

	if [ -z "$(ls -A $dirPath/src/ublox/)" ]; then
  		echo "The ublox submodule is missing."
  		exit 1
	fi

	if [ -z "$(ls -A $dirPath/src/apriltags_ros/)" ]; then
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
	echo "Type 'done' to finish, '-F' or '-R' to change your option, or 'branch' to change branches"
	echo "dir: $dirPath branch: $branch"
	echo "-------------------------------------------------------------"
	echo ""

	gnome-terminal -x bash -c "cd $dirPath;
		echo 'Checking out branch $3';
		git checkout $2;
		sleep 2;
		echo 'Pulling latest build on branch $2';
		git pull;
		sleep 2;
		echo 'Compiling $dirName branch $2...';
		catkin build;
		sleep 2;
		cd -;
		echo 'Packing up the repository... ';
		tar czf $dirName.tgz $dirName;
		sleep 2; 
		exit 1; 
		/bin/bash;' exec $SHELL"

	while(true); do
		read -p "Rover Name/IP To Start:  " roverIP
			if [ "$roverIP" =  "done" ]; then
				exit 1
			elif [ "$roverIP" = "-R" ]; then
				OPTION="-R"
				echo ""
				echo ""
				break	
			elif [ "$roverIP" = "-F" ]; then
				OPTION="-F"
				echo ""
				echo ""
				break
			elif [ "$roverIP" = "branch" ]; then
				branch=""
				echo ""
				echo ""
				break
			fi
		#If rover is on the network
		ping -q -w2 $roverIP > /dev/null
		
		if [ $? -eq 0 ]; then

			echo "-------------------------------------------------------------"
			echo "Uploading to $roverIP"
			echo ""
			echo "Copying compressed repository to swarmie at $roverIP"
			scp $dirName.tgz swarmie@$roverIP:~
			sleep 2
			echo "Unpacking repository $dirName and branch $2 on swarmie at $roverIP"
			gnome-terminal -x bash -c "ssh -t swarmie@$roverIP 'echo 'Unpacking $dirName.tgz ...';
				tar xzf $dirName.tgz;
				sleep 2;			
				echo 'Starting ROS nodes on swarmie at $roverIP with master at $hostName';
				sleep 2;
				cd $dirName/misc/;
				./rover_onboard_node_launch.sh $hostName;
				exit 1;
				/bin/bash;' exec $SHELL"

			echo "Starting ROS nodes on swarmie at $roverIP with master at $hostName"
			echo ""

			sleep 10

		#if not on the network
		else
			echo "$roverIP was not found on the network.  Check IP and network settings!"
			echo "-------------------------------------------------------------"	
			sleep 4
			echo ""
		fi

		done

fi

#Transfer Local Copy
#-------------------------------------------------------------
if [ $OPTION == "-F" ]; then

	cd ~

	if [ -z "$(ls -A $dirPath/src/ublox/)" ]; then
  		echo "The ublox submodule is missing."
  		exit 1
	fi

	if [ -z "$(ls -A $dirPath/src/apriltags_ros/)" ]; then
  		echo "The apriltags submodule is missing."
  		exit 1
	fi

	echo "Copying and transferring current folder to swarmie(s)"
	echo ""
	echo "Type 'done' to finish or '-G' or '-R' to change your option"
	echo ""
	echo "dir: $dirPath"
	echo "-------------------------------------------------------------"
	echo ""

	gnome-terminal -x bash -c "cd $dirPath;
		echo 'Compiling $dirName';
		catkin build;
		sleep 2;
		cd ~;
		echo 'Packing up the repository... ';
		tar czf $dirName.tgz $dirName;
		sleep 2; 
		exit 1; 
		/bin/bash;' exec $SHELL"

	while(true); do
		read -p "Rover Name/IP To Start:  " roverIP
			if [ "$roverIP" =  "done" ]; then
				exit 1
			elif [ "$roverIP" = "-G" ]; then
				OPTION="-G"
				echo ""
				echo ""
				break	
			elif [ "$roverIP" = "-R" ]; then
				OPTION="-R"
				echo ""
				echo ""
				break
			fi

			#If rover is on the network
			ping -q -w2 $roverIP > /dev/null
	
			if [ $? -eq 0 ]; then

				cd ~
				echo "Found $roverIP"
				echo "-------------------------------------------------------------"
				echo "Uploading to $roverIP"
				echo ""

				#Pack and send

				echo "Copying compressed repository to swarmie at $roverIP"
				scp $dirName.tgz swarmie@$roverIP:~
				sleep 2

				#unpack and run
				echo "Unpacking repository $dirName on swarmie at $roverIP"
				gnome-terminal -x bash -c "ssh -t swarmie@$roverIP 'echo 'Unpacking $dirName.tgz ...';
					tar xzf $dirName.tgz;
					sleep 2;			
					echo 'Starting ROS nodes on swarmie at $roverIP with master at $hostName';
					sleep 2;
					cd $dirName/misc/;
					./rover_onboard_node_launch.sh $hostName;
					exit 1;
					/bin/bash;' exec $SHELL"
			
				echo "Starting ROS nodes on swarmie at $3 with master at $hostName"
				echo ""
				sleep 10

			#if not on the network
			else
				echo "$roverIP was not found on the network.  Check IP and network settings!"
				echo "-------------------------------------------------------------"	
				sleep 4
				echo ""
			fi

	done	

fi

#Just Run
#-------------------------------------------------------------
if [ $OPTION == "-R" ]; then
	echo "Running current code on all swarmie(s)"
	echo ""
	echo "Type 'done' to finish or '-G' or '-F' to change your option"
	echo "-------------------------------------------------------------"
	echo ""

	while(true); do
		read -p "Rover Name/IP To Start:  " roverIP
			if [ "$roverIP" =  "done" ]; then
				exit 1
			elif [ "$roverIP" = "-G" ]; then
				OPTION="-G"
				echo ""
				echo ""
				break	
			elif [ "$roverIP" = "-F" ]; then
				OPTION="-F"
				echo ""
				echo ""
				break
			fi

		echo "$OPTION"

		#If rover is on the network
		ping -q -w2 $roverIP > /dev/null
		if [ $? -eq 0 ]; then
			echo "-------------------------------------------------------------"
			echo "Starting $roverIP"

			#ssh and run script from rover --WORKS
			gnome-terminal -x bash -c "ssh -t $rover@$roverIP 'cd SwarmBaseCode-ROS/misc;./rover_onboard_node_launch.sh $hostName;exit 1;/bin/bash;' exec $SHELL"

			sleep 10

		#if not on the network
		else
			echo "$roverIP was not found on the network.  Check IP and network settings!"
			echo "-------------------------------------------------------------"	
			sleep 4
			echo ""
		fi

	done
fi

done
