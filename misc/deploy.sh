#!/bin/bash

OPTION=$1

roverpass=""
roverIP=""

cd ..
dirPath="$(pwd)"
dirName="$(basename `pwd`)"

#Functions
#--------------------------------------------------------------------------

Check()
{

	cd ~

	if [ -z "$(ls -A $dirPath/src/ublox/)" ]; then
  		echo "The ublox submodule is missing."
  		exit 1
	fi

	if [ -z "$(ls -A $dirPath/src/apriltags_ros/)" ]; then
  		echo "The apriltags submodule is missing."
  		exit 1
	fi
}

PullGit_Pack()
{
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
		tar czf $dirName.tgz $dirPath;
		exit 1; 
		/bin/bash;' exec $SHELL"
}

Pack()
{
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
}

Transfer()
{

	echo "Copying compressed repository to swarmie at $roverIP"
	scp $dirName.tgz swarmie@$roverIP:~
}

Unpack_Run()
{
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
}

Run()
{

	#ssh and run script from rover --WORKS
	gnome-terminal -x bash -c "ssh -t swarmie@$roverIP 'cd SwarmBaseCode-ROS/misc;
		./rover_onboard_node_launch.sh $hostName;
		exit 1;
		exit 1;
		/bin/bash;' exec $SHELL"
}

DispOpt()
{
	clear

	echo "Your network info:  $hostName@$userIP"
	echo ""
	echo "Type 'exit' to quit"

	if [ "$OPTION" != "-G" ]; then
		echo "Type '-G' to pull, transfer, and run on swarmie(s) "
	fi
	
	if [ "$OPTION" != "-L" ]; then
		echo "Type '-L' to transfer local code and run on swarmie(s)"
	fi

	if [ "$OPTION" != "-R" ]; then
		echo "Type '-R' to run current code loaded on Swarmies"
	fi

	echo "-------------------------------------------------------------"
	echo ""
}

SuccessPing()
{
	cd ~
	echo "Found $roverIP"
	echo "-------------------------------------------------------------"
	echo "Uploading to $roverIP"
	echo ""
}

FailPing()
{
	echo "$roverIP was not found on the network.  Check IP and network settings!"
	echo "-------------------------------------------------------------"	
	sleep 4
	echo ""
}

#Code Start
#-----------------------------------------------------------------

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
	echo "'-G'[ithub] {branch} pulls and transfers workspace to swarmie(s)"
	echo "'-L'[ocal Files] duplicates current workspace and transfers to swarmie(s)"
	echo "'-R'[un] to run current code on all swarmie(s)"
	echo "-------------------------------------------------------------"
	exit 1
fi

clear

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

	#Tell User Option

	echo "Retrieving and Transferring Code from GitHub:  "

	DispOpt

	echo "dir: $dirPath branch: $branch"
	echo "-------------------------------------------------------------"
	echo ""

	Check
	PullGit_Pack

	while(true); do
		read -p "Rover Name/IP To Start:  " roverIP
			if [ "$roverIP" =  "exit" ]; then
				exit 1
			elif [ "$roverIP" = "-R" ]; then
				OPTION="-R"
				echo ""
				echo ""
				break	
			elif [ "$roverIP" = "-L" ]; then
				OPTION="-L"
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

			SuccessPing

			#Transfer/Unpack/Run
			Transfer
			Unpack_Run
			sleep 10

		#if not on the network
		else
			FailPing
		fi

		done

fi

#Transfer Local Copy
#-------------------------------------------------------------
if [ $OPTION == "-L" ]; then

	Check

	echo "Copying and transferring current folder to swarmie(s)"

	DispOpt

	echo "dir: $dirPath"
	echo "-------------------------------------------------------------"
	echo ""

	Pack

	while(true); do
		read -p "Rover Name/IP To Start:  " roverIP
			if [ "$roverIP" =  "exit" ]; then
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

				SuccessPing

				#Transfer/Unpack/Run
				Transfer
				Unpack_Run
				sleep 10

			#if not on the network
			else
				FailPing
			fi
	done	
fi

#Just Run
#-------------------------------------------------------------
if [ $OPTION == "-R" ]; then

	DispOpt

	echo "Running current swarmie(s) code"
	echo "-------------------------------------------------------------"
	echo ""

	while(true); do
		read -p "Rover Name/IP To Start:  " roverIP
			if [ "$roverIP" =  "exit" ]; then
				exit 1
			elif [ "$roverIP" = "-G" ]; then
				OPTION="-G"
				echo ""
				echo ""
				break	
			elif [ "$roverIP" = "-L" ]; then
				OPTION="-L"
				echo ""
				echo ""
				break
			fi

		#If rover is on the network
		ping -q -w2 $roverIP > /dev/null
		if [ $? -eq 0 ]; then

			SuccessPing
			Run
			sleep 10

		#if not on the network
		else	
			FailPing
		fi
	done
fi

done
