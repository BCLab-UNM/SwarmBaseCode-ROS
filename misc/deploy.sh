#!/bin/bash
#This script has been added to aid in rapid development and deployment of multiple robots!


#-------------------------READ THIS----------------------------#
#If you have changed your rovers password you MUST edit this variable with the correct password to 
#allow the reboot feature to work correctly!!
roverPass="KSC-2018"
#--------------------------------------------------------------#

OPTION=$1
branch=$2
roverIP=""
branch=""
needsReboot=false

cd ..
dirPath="$(pwd)"
dirName="$(basename `pwd`)"

#Functions
#--------------------------------------------------------------------------

Check()
{
	cd ~

        if [ -z "$(ls -A $dirPath/src/ublox/)" ] || [ -z "$(ls -A $dirPath/src/apriltags_ros/)" ]; then
		echo "The ublox and/or april tags submodule is missing."
		cd $dirPath; #Entering directory/local git repo (needed to run git commands)
		echo "Initiliazing and updating ublox and april tag submodules...";
		git submodule init;                
		git submodule update; 
	fi
}

PullGit_Pack()
{
	info="Pulling from $branch, Compiling, and Packing..."
	gnome-terminal --disable-factory -x bash -c "echo -n -e '\033]0;$info\007';
		cd $dirPath;
		echo 'Checking out branch $branch';
		git checkout $branch &&
		{
			echo 'Pulling latest build on branch $branch';
			git pull && 
			{
				echo 'Compiling $dirName branch $branch...';
				catkin build &&
				{
					cd -;
					echo 'Packing up the repository... ';
					tar czhf $dirName.tgz $dirPath;
					exit 1;
				}
				read stuff;
				exit 1; 
			}
			read stuff;
			exit 1;
		}		
		read stuff
		exit 1;
		/bin/bash;' exec $SHELL"
}

Pack()
{
	info="Compiling and Packing Local Files..."
	gnome-terminal --disable-factory -x bash -c "echo -n -e '\033]0;$info\007';
		cd $dirPath;
		{
			echo 'Compiling $dirName';
			catkin build &&
			{ 
				cd ~;
				echo 'Packing up the repository... ';
				tar czhf $dirName.tgz $dirName;
				exit 0;
			}
			read stuff
			exit 1;
		}
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

	gnome-terminal --tab -x bash -c "echo -n -e '\033]0;$roverIP\007'
		ssh -t swarmie@$roverIP 'echo 'Unpacking $dirName.tgz ...';
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
	gnome-terminal --tab -x bash -c "echo -n -e '\033]0;$roverIP\007';
		ssh -t swarmie@$roverIP 'echo 'Running $roverIP';
		cd SwarmBaseCode-ROS/misc;
		./rover_onboard_node_launch.sh $hostName;
		exit 1;
		exit 1;
		/bin/bash;' exec $SHELL"
}

DispOpt()
{
	echo ""
	echo "Type 'exit' to quit"

	if [ "$OPTION" != "-G" ]; then
		echo "Type '-G' to pull, transfer, and run on swarmie(s) "
	elif [ $OPTION == "-G" ]; then
		echo "Type '-RP' to Re Pull from the github Repository"
		echo "Type '-NB' to pull from a New Branch"
	fi

	if [ "$OPTION" != "-R" ]; then
		echo "Type '-R' to run current code loaded on swarmie(s)"
	fi
	
	if [ "$OPTION" != "-L" ]; then
		echo "Type '-L' to transfer local code and run on swarmie(s)"
	elif [ $OPTION == "-L" ]; then
                echo "Type '-RC' to recompile Source Code"
	fi

	echo "Type 'REBOOT {hostname}' to reboot a swarmie"
        echo "Type desired rover name(s) to run"
	echo "-------------------------------------------------------------"
}

SuccessPing()
{
	cd ~
	echo "Found $roverIP"
	echo "-------------------------------------------------------------"
}

FailPing()
{
	echo "$roverIP was not found on the network.  Check IP and network settings!"
	echo "-------------------------------------------------------------"	
	sleep 4
	echo ""
}

Reboot()
{
	info="Rebooting $roverIP and Reconnecting..."
	echo "$info"
	ssh -t swarmie@$roverIP "echo $roverPass | sudo -S reboot now; exit 1;"
	sleep 5
	{
		while(true); do
			ping -q -w10 $roverIP > /dev/null;
			if [ $? -eq 0 ]; then
				break;
			fi
		done
	}

	echo "Rover $roverIP is up... Ready to Reconnect!";
}

#Code Start
#-----------------------------------------------------------------

#find users current IP and store it
userIP=$(ifconfig  | grep 'inet addr:'| grep -v '127.0.0.1' | cut -d: -f2 | awk '{ print $1 }')
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
	echo "'-G'[ithub] pulls and transfers workspace to swarmie(s)"
	echo "'-L'[ocal Files] duplicates current workspace and transfers to swarmie(s)"
	echo "'-R'[un] to run current code on all swarmie(s)"
        echo "'-S'[ilent] after option to run and return to terminal without using interface"
	echo "-------------------------------------------------------------"
	exit 1
fi

#check OPTION given
#------------------------------------------------------------

while(true); do

if [ "$2" == "-S" ]; then

    i=3

    if [ $OPTION == "-G" ]; then

        #Get Branch
        read -e -p "Branch?:  " branch

        Check

        PullGit_Pack &
        wait

        while [ "${!i}" != "" ]; do
            roverIP=${!i}
            roverIP=${roverIP^^}

            if [ $roverIP == "REBOOT" ]; then
                needsReboot=true
                i=$((i+1))
                roverIP=${!i}
                roverIP=${roverIP^^}
            fi

            #If rover is on the network
            ping -q -w2 $roverIP > /dev/null
            if [ $? -eq 0 ]; then

                    if [ $needsReboot == true ]; then
                            Reboot
                            echo ""
                            Run
                            needsReboot=false
                            sleep 10
                    else
                            #Transfer/Unpack/Run
                            Transfer
                            Unpack_Run
                            sleep 10
                    fi
            #if not on the network
            else
                    FailPing
                    needsReboot=false
            fi

            i=$((i+1))

        done

    elif [ $OPTION == "-L" ]; then

        Pack &
        wait

        while [ "${!i}" != "" ]; do
            roverIP=${!i}
            roverIP=${roverIP^^}

            if [ $roverIP == "REBOOT" ]; then
                needsReboot=true
                i=$((i+1))
                roverIP=${!i}
                roverIP=${roverIP^^}
            fi

            #if we didn't just select the reboot option
            if [ $roverIP != "REBOOT" ]; then

                #If rover is on the network
                ping -q -w2 $roverIP > /dev/null

                if [ $? -eq 0 ]; then

                    if [ $needsReboot == true ]; then
                        Reboot
                        echo ""
                        Run
                        needsReboot=false
                        sleep 10
                    else
                        #Transfer/Unpack/Run
                        cd ~
                        Transfer
                        Unpack_Run
                        sleep 10
                        cd -
                    fi

                #if not on the network
                else
                    FailPing
                    needsReboot=false
                fi
            fi

            i=$((i+1))

        done

    elif [ $OPTION == "-R" ]; then

        while [ "${!i}" != "" ]; do

            roverIP=${!i}
            roverIP=${roverIP^^}

            if [ $roverIP == "REBOOT" ]; then
                needsReboot=true
                i=$((i+1))
                roverIP=${!i}
                roverIP=${roverIP^^}
            fi

            ping -q -w2 $roverIP > /dev/null

            #If rover is on the network
            if [ $? -eq 0 ]; then

                #reboot
                if [ $needsReboot == true ]; then
                    Reboot
                    echo ""
                    Run
                    sleep 10
                    needsReboot=false
                else
                    Run
                    sleep 10
                fi

            #if not on the network
            else
                    FailPing
                    needsReboot=false
            fi

            i=$((i+1))
        done
    else
        echo "Not valid option"
    fi

exit 1

#GitHub Selection
#------------------------------------------------------------
elif [ $OPTION == "-G" ]; then

    clear
	#if not everything is filled out
	if [ "$branch" == "" ]; then
		echo ""
                read -e -p "Branch?:  " branch
		echo ""
	fi

	#Tell User Option

        echo "Your network info:  $hostName@$userIP"
	echo "Retrieving and Transferring Code from GitHub:  "
	echo "dir: $dirPath branch: $branch"
	echo "-------------------------------------------------------------"

	Check

	PullGit_Pack &
	wait

	while(true); do

	i=0
	changeOption=false

                DispOpt

                read -p "Option/Rover Name(s):  " -a arr

		size=${#arr[@]}

		while [ $i -lt $size ]; do

			#retrieve string from arr
			roverIP=${arr[i]}
			roverIP=${roverIP^^}

			i=$((i+1))

			if [ "$roverIP" == "EXIT" ]; then
				exit 1
			elif [ "$roverIP" == "-R" ]; then
				OPTION="-R"
				clear
				changeOption=true
				break	
			elif [ "$roverIP" == "-L" ]; then
				OPTION="-L"
				clear
				changeOption=true
				break
			elif [ "$roverIP" == "-NB" ]; then
				branch=""
				clear
				changeOption=true
				break
			elif [ $roverIP == "-RP" ]; then
				clear
				echo ""
				echo "-------------------------------------------------------------"
				echo ""
				echo "Pulling new code from $branch"
				echo ""
				changeOption=true
				break;
			elif [ $roverIP == "REBOOT" ]; then
				needsReboot=true
				roverIP=${arr[i]}
				roverIP=${roverIP^^}
				i=$((i+1))				
			fi

			#check if rebooting
			if [ $needsReboot == true ]; then
				echo "Attempting to Reboot $roverIP"
			else
				echo "Transferring to and Running $roverIP"
			fi

			#If rover is on the network
			ping -q -w2 $roverIP > /dev/null
			if [ $? -eq 0 ]; then

				SuccessPing
	
				if [ $needsReboot == true ]; then
					Reboot
					echo ""
					Run
					needsReboot=false
					sleep 10
				else
					#Transfer/Unpack/Run
					Transfer
					Unpack_Run
					sleep 10	
				fi	
			#if not on the network
			else
				FailPing
				needsReboot=false
			fi
		done


		if [ $changeOption == true ]; then
			break
		fi

		i=0
	done

#Transfer Local Copy
#-------------------------------------------------------------
elif [ $OPTION == "-L" ]; then

    clear

	Check

        echo "Your network info:  $hostName@$userIP"
	echo "Copying and transferring current folder to swarmie(s)"
	echo "dir: $dirPath"
	echo "-------------------------------------------------------------"

	Pack &
	wait

	while(true); do

	i=0
	changeOption=false

                DispOpt

                read -p "Option/Rover Name(s):  " -a arr

		size=${#arr[@]}

		while [ $i -lt $size ]; do

			#retrieve string from arr
			roverIP=${arr[i]}
			roverIP=${roverIP^^}

			i=$((i+1))

			if [ $roverIP =  "EXIT" ]; then
				exit 1
			elif [ $roverIP = "-G" ]; then
				OPTION="-G"
				changeOption=true
				clear
				break	
			elif [ $roverIP = "-R" ]; then
				OPTION="-R"
				changeOption=true
				clear
				break
			elif [ $roverIP = "-RC" ]; then
				clear
                                changeOption=true
				echo "-------------------------------------------------------------"
				echo ""
				echo "Recompiling Source Code!"
				echo ""
				break
			elif [ $roverIP == "REBOOT" ]; then
				needsReboot=true
				roverIP=${arr[i]}
				roverIP=${roverIP^^}
				i=$((i+1))		
			fi

			#check if rebooting
			if [ $needsReboot == true ]; then
				echo "Attempting to Reboot $roverIP"
			else
				echo "Transferring to and Running $roverIP"
			fi

			#if we didn't just select the reboot option
			if [ $roverIP != "REBOOT" ]; then
	
				#If rover is on the network
				ping -q -w2 $roverIP > /dev/null

				if [ $? -eq 0 ]; then	
					SuccessPing

					if [ $needsReboot == true ]; then
						Reboot
						echo ""
						Run
						needsReboot=false
						sleep 10
					else
						#Transfer/Unpack/Run
						Transfer
						Unpack_Run
						sleep 10
					fi
	
				#if not on the network
				else
					FailPing
					needsReboot=false
				fi
			fi
		done

		if [ $changeOption == true ]; then
			break
		fi

		i=0
	done

#Just Run
#-------------------------------------------------------------
elif [ $OPTION == "-R" ]; then

    clear

        echo "Your network info:  $hostName@$userIP"
	echo "Running current swarmie(s) code"
	echo "-------------------------------------------------------------"

	while(true); do

	i=0
	changeOption=false

                DispOpt

                read -p "Option/Rover Name(s):  " -a arr

                size=${#arr[@]}

		while [ $i -lt $size ]; do

			#retrieve string from arr
			roverIP=${arr[i]}
			roverIP=${roverIP^^}

			i=$((i+1))
		
			#check input
                        if [ "$roverIP" =  "EXIT" ]; then
				exit 1
			elif [ "$roverIP" == "-G" ]; then
				OPTION="-G"
				changeOption=true
				clear
				break	
			elif [ "$roverIP" == "-L" ]; then
				OPTION="-L"
				changeOption=true
				clear
				break
			elif [ $roverIP == "REBOOT" ]; then
				needsReboot=true
				roverIP=${arr[i]}
				roverIP=${roverIP^^}
				i=$((i+1))		
			fi

			#check if rebooting
			if [ $needsReboot == true ]; then
				echo "Attempting to Reboot $roverIP"
			else
				echo "Running $roverIP"
			fi
	
			#if we didn't just select the reboot option
			if [ $roverIP != "REBOOT" ]; then
			
				#If rover is on the network
				ping -q -w2 $roverIP > /dev/null
				if [ $? -eq 0 ]; then
		
					SuccessPing

					#reboot
					if [ $needsReboot == true ]; then
						Reboot
						echo ""
						Run
						sleep 10
						needsReboot=false
					else
						Run
						sleep 10
					fi
	
				#if not on the network
				else	
					FailPing
					needsReboot=false
				fi
			fi
		done

		if [ $changeOption == true ]; then
			break
		fi

		i=0
	done
else
    echo "Unrecognized command! Please ensure you are typing in your option correctly and try again!"
    exit 1
fi

done
