#!/bin/bash
numberOfRovers=$1
#numberOfObstacles=$2
arguments=($@)
randomLocationCounter=1
originOccupied=false
repeats=0
yCoordArray=(-1.0 -0.5 0.0 0.5 1.0)
yCounter=0
uniformDist=false
clusteredDist=false
powerLawDist=false
alreadyOutThere=false

#Gets the absolute value of the value passed in 
function abs()                   # Absolute value.
{                                # Caution: Max return value = 255.
  E_ARGERR=-999999

  if [ -z "$1" ]                 # Need arg passed.
  then
    return $E_ARGERR             # Obvious error value returned.
  fi
  
  inputValue=$(bc <<< "$1>0")
  if [ "$inputValue" == "1" ]              # If greater than zero,
  then                           #
    absval=$1                    # stays as-is.
  else                           # Otherwise,
    absval=$(bc <<< "0 - $1")  # change sign.
  fi  
}

#Checks whether or not the file passed in is executable, if not, makes it executable
function permissions()
{
  file="$1"
  if [ -e "$file" ] && [ -x "$file" ]; then
    echo "File '$file' is executable."
  elif [ -e "$file" ] && [ ! -x "$file" ];then
      echo "File '$file' is not executable."
      echo "Making '$file' executable..."
      chmod +x $file
      echo "Done."
  else
    echo "$file does not exist. Please make sure $file is in working directory."
    PS3='Would you like to continue without executable: '$file'? NOTE: Continuing without this executable may result in errors. '
    options=("Continue" "Quit")
    select opt in "${options[@]}"
    do
      case $opt in
        "Continue")
           echo
           echo "Continuing without "$file""
           break
           ;;
        "Quit")
           echo "Exiting"
           exit 0
           ;;
         *) echo invalid option;;
      esac
    done
fi
}

#Gets the tag distribution selection from the user
function tagDist()
{
    echo "What tag distribution would you like"
    options=("Uniform" "Clustered" "PowerLaw")
    select opt in "${options[@]}"
    do
      case $opt in
        "Uniform")
           echo
           uniformDist=true
           echo "Continuing with Uniform distribution"
           break
           ;;
        "Clustered")
           echo
           clusteredDist=true
           echo "Continuing with clustered distribution"
           break
           ;;
        "PowerLaw")
           echo
           powerLawDist=true
           echo "Continuing with power law distribution"
           break
           ;;
         *) echo invalid option;;
      esac
    done
}

clear
#Generate executables. This is necessary because Filezilla does not handle executables that well.
echo "Checking random_locations..."
./random_locations
if [ $? -ne 0 ];then
  echo "random_locations is not executable..."
  echo "Generating executable from source..."
  g++ -O -Wall src/random_locations.cpp -o random_locations
else
  echo "random_locations is executable..."
fi
echo
echo "Checking rover_driver_node_launch..."
./rover_driver_node_launch check check
if [ $? -ne 0 ];then
  echo "rover_driver_node_launch is not executable..."
  echo "Generating executable from source..."
  g++ -O -Wall src/rover_driver_node_launch.cpp -o rover_driver_node_launch -lboost_system
else
  echo "rover_driver_node_launch is executable..."
fi
rm check.launch


#Rover number check. 
if [[ -n ${numberOfRovers//[0-9]/} ]] || [ -z "$numberOfRovers" ];then
  echo "Number of rovers necessary. If you do not want any rovers spawned, enter 0."
  echo 
  echo
#While loop for valid input
  while true;do
    read numberOfRovers
      if [[ -n ${numberOfRovers//[0-9]/} ]] || [ -z "$numberOfRovers" ];then
        echo "Number of rovers necessary. If you do not want any rovers spawned, enter 0."
        echo 
        echo
    else
        echo "$numberOfRovers rovers will be spawned."
        break
      fi
  done
#Valid name
else
  echo "$numberOfRovers will be spawned."
fi

#Get Rover Names
nameCounter=0
userInput=true
echo "NUM ROVERS : $numberOfRovers"
for (( i=1; i<=$numberOfRovers; i++ ));do
  if [ -z "${arguments[$i]}" ];then
   echo "Rover name missing, add a rover:"
    while true;do
      read roverName
        if [ -z "$roverName" ];then
          echo "Enter a valid name."
          echo
        else
          userInput=false
          echo "Added $roverName to the list"
          roverNameArray+=($roverName)
          nameCounter=$(($nameCounter+1))
	  echo "1 - ADDED TO ROVER NAME ARRAY : $roverNameArray[$nameCounter]"
          echo
          break
        fi
    done
  else
    roverNameArray[$nameCounter]="${arguments[$i]}"
    let "nameCounter = $nameCounter+1"
  fi
done

echo ${#roverNameArray[@]}
#Checks to see if the number of rover names entered doesn't match the actual number of rovers requested
counter=2
testForNumber='^[0-9]+$'
initialRoverCount=$numberOfRovers
#First make sure that the argument isn't of length zero
if ! [ userInput ];then
while [ -n ${arguments[$numberOfRovers+$counter]} ] && ! [[ ${arguments[$numberOfRovers+$counter]} = *[[:digit:]]* ]];do
    echo "Uh oh"
    echo "It looks like you wanted to spawn $initialRoverCount rovers, but you wrote more than $initialRoverCount names down"
    echo "Add ${arguments[$numberOfRovers+$counter]} to the list of rovers? [y/n]"
    while true;do
      read response
      if [ -z "$response" ];then
        echo "Add ${arguments[$numberOfRovers+$counter]} to the list of rovers? [y/n]"
      else
        if [ $response == 'y' ] || [ "$response" == "yes" ];then
          echo  
          echo "Adding ${arguments[$numberOfRovers+$counter]}"
          roverNameArray[${#roverNameArray[@]}+$counter]="${arguments[$numberOfRovers+$counter]}"
          numberOfRovers=$((numberOfRovers+1))
          echo
          break
        elif [ $response == 'n' ] || [ $response == "no" ];then
          echo "no"
          counter=$((counter+1))
          echo
          break
        else
          echo "Add ${arguments[$numberOfRovers+$counter]} to the list of rovers? [y/n]"
          echo
        fi
      fi
    done
  done
fi


##########################
#Check for name repeats
##########################
#Not working- commenting out
#for ((i = 0; i < $numberOfRovers; i++));do
#  echo
#  nameCheck="${roverNameArray[$i]}"
#  for ((j = 0; j < $numberOfRovers; j++));do
#    if [ "$nameCheck" = "${roverNameArray[$j]}" ] && [ $j -ne $i ];then
#      echo "$nameCheck is duplicated in this list"
#      while true;do
#      alreadyOutThere=false
#      echo "Enter a new name"
#      read roverName
#        if [ -z "$roverName" ];then
#          echo "Enter a valid name."
#          alreadyOutThere=true
#          echo
#        else
#          for ((k = 0; k < $numberOfRovers; k++));do
#             if [ "$roverName" = "${roverNameArray[$k]}" ] && [ $j -ne $k ];then
#               echo "Pick a new name. $roverName is a duplicate"
#               alreadyOutThere=true
#               break
#             fi
#          done
#        fi
#        if [ "$alreadyOutThere" = false ];then
#          echo "Adding $roverName to the list"
#          roverNameArray[$i]=$roverName
#          break
#        fi
#      done
    #DEBUG else
     #echo "Fine"
#    fi
#  done
#done     

#To give user the option of where to spawn rovers, commented out because all will be spawned at origin
#if [ -z "${arguments[$numberOfRovers+2]}" ];then
#  echo "Would you like to spawn them all at the origin?"
#  echo "Enter 1 for yes, 0 for no"
#  while true;do
#    read originSpawn
#      if [[ -n ${originSpawn//[0-9]/} ]] || [ -z "$originSpawn" ];then
#        echo "Enter 1 to spawn all of the rovers at the origin. Enter 0 for no"
#      elif [ "$originSpawn" -eq 1 ];then
#        echo "Will spawn all of the rovers at the origin"
#        echo
#        break
#      elif [ "$originSpawn" -eq 0 ];then
#        echo "Will not spawn the rovers at the origin."
#        echo
#        break
#      else
#        echo "Enter 1 to spawn all of the rovers at the origin. Enter 0 for no"
#        echo
#      fi
#  done
#else
#  if [ "${arguments[$numberOfRovers+2]}" -eq 1 ];then
#    echo "All spawn at origin"
#    originSpawn=1
#  elif [ "${arguments[$numberOfRovers+2]}" -eq 0 ];then
#    echo "Not spawning them at the origin"
#    originSpawn=0
#  else
#    echo "Enter 1 to spawn all of the rovers at the origin. Enter 0 for no"
#    while true;do
#      read originSpawn
#        if [[ -n ${originSpawn//[0-9]/} ]] || [ -z "$originSpawn" ];then
#          echo "Enter 1 to spawn all of the rovers at the origin. Enter 0 for no"
#        elif [ "$originSpawn" -eq 1 ];then
#          echo "Will spawn all of the rovers at the origin"
#          echo
#          break
#        elif [ "$originSpawn" -eq 0 ];then
#          echo "Will not spawn the rovers at the origin."
#          echo
#          break
#        else
#          echo "Enter 1 to spawn all of the rovers at the origin. Enter 0 for no"
#          echo
#        fi
#    done
#  fi
#fi


#if [ "$originSpawn" -eq 1 ];then 
#Spawning all rovers at the origin
echo "Spawning all of the rovers at the origin, standby"
for (( i=0; i<=$numberOfRovers; i++ ));do
  roverPos[$i]=0
done
echo
echo
#elif [ "$originSpawn" -eq 0 ];then
#  echo "Manually enter the rover positions."
#  for ((i = 0; i < $numberOfRovers; i++));do
#    echo "Enter the state you want to add ${roverNameArray[$i]} in:"
#    echo "0: Spawn at the origin"
#    echo "1: Spawn randomly"
#    echo "2: Spawn manually (x,y)"
#    echo
#    while true;do
#      read newRoverPos
#      if [[ -z "$newRoverPos" ]] || [[ -n ${newRoverPos//[0-9]/} ]];then
#        echo "Enter a valid number for ${roverNameArray[$i]} "
#      else
#        roverPos[$i]=$newRoverPos
#        break
#      fi
#    done
#  done
#fi

#Get the target count if user entered in all parameters at once
if [ userInput ];then
  targetCountIndex=$roverNameStartIndex
  let "targetCountIndex = $targetCountIndex+$numberOfRovers+1"
  numberOfTags="${arguments[$targetCountIndex]}"
else
  numberOfTags=
fi

#Get the target distribution
let "tagDistIndex = $targetCountIndex + 1"
tagDist=${arguments[$tagDistIndex]}
if [ -z "$tagDist" ] || [[ -n ${tagDist//[0-9]/} ]];then
  tagDist
elif [[ $tagDist -eq 1 ]];then
  echo "Uniform Tag Distribution"
  uniformDist=true
elif [[ $tagDist -eq 2 ]];then
  echo "Clustered Tag Distribution"
  clusteredDist=true
elif [[ $tagDist -eq 3 ]];then
  echo "PowerLaw Tag Distribution"
  powerLawDist=true
fi

#Number of tags for single distribution to be added check. This is necessary to make sure a vaild input was added and to 
#avoid errors when the tag spawning routine is called.
if [ "$uniformDist" = true ]; then
  if [[ -n ${numberOfTags//[0-9]/} ]] || [ -z "$numberOfTags" ]; then
    echo "Number of tags necessary. If you do not want any tags spawned, enter 0."
  #While loop to check if a number was entered for the input. Will not exit unless a number is input.
    while true;do
      read numberOfTags
      if [[ -n ${numberOfTags//[0-9]/} ]] || [ -z "$numberOfTags" ]; then
        echo "Number of tags necessary. If you do not want any tags spawned, enter 0."
        echo 
        echo
      else
        echo
        echo "$numberOfTags tags will be spawned."
        echo
        break
      fi
    done
  #A number for the number of tags was spawned
  else
    echo "$numberOfTags tags will be spawned."
  fi
fi

#Get the obstacle count
if [ userInput ]; then
  let "obstacleCountIndex = $tagDistIndex + 1"
  numberOfObstacles=${arguments[$obstacleCountIndex]}
else
  numberOfObstacles=
fi
if [[ -n ${numberOfObstacles//[0-9]/} ]] || [ -z "$numberOfObstacles" ];then 
  echo "How many obstacles do you want to spawn for this trial?"
  while true;do
    read numberOfObstacles
        if [[ -n ${numberOfObstacles//[0-9]/} ]] || [ -z "$numberOfObstacles" ];then
          echo "How many obstacles do you want to spawn for this trial?"
        else
          echo "Will enter $numberOfObstacles obstacles"
          echo
          break
        fi
  done
fi



#Call the permissions function for the following files
permissions 'random_locations'
permissions 'rover_driver_node_launch'

#Remove tmp files
#Get the current working directory to check if the files exist, if so then remove them
currentWorkingDirectory=${PWD} 
tmpTxtName='/tmp.txt'
if [ -f $currentWorkingDirectory$tmpTxtName ];then
   rm tmp.txt
fi
randomTmpName='randomNumbers.txt'
if [ -f $currentWorkingDirectory$randomTmpName ]; then
   rm randomNumbers.txt
#Get the target distribution
let "tagDistIndex = $targetCountIndex + 1"
tagDist=${arguments[$tagDistIndex]}
if [ -z "$tagDist" ] || [[ -n ${tagDist//[0-9]/} ]];then
  tagDist
elif [[ $tagDist -eq 1 ]];then
  echo "Uniform Tag Distribution"
  uniformDist=true
elif [[ $tagDist -eq 2 ]];then
  echo "Clustered Tag Distribution"
  clusteredDist=true
elif [[ $tagDist -eq 3 ]];then
  echo "PowerLaw Tag Distribution"
  powerLawDist=true
fi

fi

#Checks to see if the rover they choose exists in the ~/rover_workspace/src/rover_misc/gazebo/models dir.
for ((i = 0; i < $numberOfRovers; i++));do
  echo "Searching for ${roverNameArray[$i]} in ~/rover_workspace/src/rover_misc/gazebo/models/"
  echo
  if [ ! -d ~/rover_workspace/src/rover_misc/gazebo/models/"${roverNameArray[$i]}" ];then
    #Directory doesn't exist. Asks if you would like to change the name of the rover or add the model files manually
    echo "Would you like to change the rovers name you wish to spawn first [0] or add ${roverNameArray[$i]} model files manually to the directory ~/rover_workspace/src/rover_misc/gazebo/models/ [1]?"
    while true;do
      read choice
    #Choose to rename the rover
      if [ "$choice" == "0" ] && [ ! -d ~/rover_workspace/src/rover_misc/gazebo/models/"${roverNameArray[$i]}" ];then
        echo "What is the new name for the rover?"
      #Waits for vaild input for name. Will check if that rover exists. If you do not have any rovers in the directory ~/rover_workspace/src/rover_misc/gazebo/models/ you will need to add them manually
        while true;do
        read roverName
          if [ ! -d ~/rover_workspace/src/rover_misc/gazebo/models/"$roverName" ];then
            echo "Model doesn't exist. Choose a model that exists in the directory ~/rover_workspace/src/rover_misc/gazebo/models/"
            echo 
            echo
          else
            echo
            echo
          #Found the rover
            echo "$roverName found!"
            echo "$roverName will be spawned first"
            roverNameArray[$i]=$roverName
            echo "Press enter to continue"
            break
            break
          fi
        done
    #Choose to manually add the selected rover files into echo ~/rover_workspace/src/rover_misc/gazebo/models/
      elif [ "$choice" == "1" ] && [ ! -d ~/rover_workspace/src/rover_misc/gazebo/models/"$roverName" ];then
        echo "Is $roverName model files in the directory ~/rover_workspace/src/rover_misc/gazebo/models/? [y]"
        while true;do
          read response
        #response is yes but the files cannot be located
          if [ "$response" == "y" ] && [ ! -d ~/rover_workspace/src/rover_misc/gazebo/models/"$roverName" ];then
            echo 
            echo "Make sure that you inserted the model into the correct directory."
            echo
        #response is yes and the files can be located
          elif [ "$response" == "y" ] && [ -d ~/rover_workspace/src/rover_misc/gazebo/models/"$roverName" ];then
            echo
            echo "$roverName found!"
            roverNameArray[$i]=$roverName
            echo
            break
        #garbage response
          else
            echo "Command Not Recognized. 1"
          fi
        done
    #Garbage response
      elif [ ! -d ~/rover_workspace/src/rover_misc/gazebo/models/"$roverName" ];then
        echo "Command Not Recognized. 2"
      else
        break
      fi
    done
  else
    echo
    echo "${roverNameArray[$i]} found!"
  fi
done
echo

################################
# Generate rtandom numbers for use later  #
################################
  './random_locations'

################################
# ENVIRONMENT                  #
################################
source ~/rover_workspace/devel/setup.bash

################################
# ROS                          #
################################
echo "Starting roscore ..."
roscore &
sleep 3

#Starts emergency shutdown script
#gnome-terminal -e ./emergencyQuit.sh &

################################
# ROBOT NODES                  #
################################
for ((i = 0; i < $numberOfRovers; i++));do
  echo "Generating robot launch file"
  './rover_driver_node_launch' "${roverNameArray[$i]}"
  echo "Generated launch file: ""${roverNameArray[$i]}"".launch"
  echo
  mv "${roverNameArray[$i]}".launch launch_files/
  echo "Executing launch file ..."
  #roslaunch rover_driver_gazebo_launch "${roverNameArray[$i]}".launch name:="${roverNameArray[$i]}" &
  roslaunch launch_files/"${roverNameArray[$i]}".launch name:="${roverNameArray[$i]}" &
  sleep 2
  #mv *.csv logs/
  #echo "Moved log file to logs folder"
  echo "Welcome to the world of tomorrow ""${roverNameArray[$i]}"
done

################################
# WORLD                        #
################################
echo "Starting world_state"
roslaunch launch_files/world_state.launch &
sleep 2

################################
# GAZEBO                       #
################################
echo "Starting gazebo"
rosrun gazebo_ros gazebo &
sleep 5

################################
# USER INTERFACE               #
################################
echo "Starting operator interface (rqt)"
rqt &
sleep 2

###########################################
#If you want them all spawned at the origin
if [[ "$originSpawn" -eq 1 ]];then 
echo "IN FIRST IF"
  echo "Spawning all of the rovers at the origin, standby"
#  if [ $numberOfRovers -le 4 ];then #Assemble them using the usual placement  
#    for ((i = 0; i < $numberOfRovers; i++));do
#     #Assemble around origin
#      if [ $i == "0" ];then
#        gzfactory spawn -f ~/rover_workspace/src/rover_misc/gazebo/models/"${roverNameArray[$i]}"/model.sdf -m "${roverNameArray[$i]}" -x 0 -y 0 -z 0 -R 0 -P 0 -Y 0;
#        echo "0 0" >> tmp.txt
#        echo
#      elif [ $i == "1" ];then
#        gzfactory spawn -f ~/rover_workspace/src/rover_misc/gazebo/models/"${roverNameArray[$i]}"/model.sdf -m "${roverNameArray[$i]}" -x 0 -y 1.0 -z 0 -R 0 -P 0 -Y -1.55;
#        echo "0 1.0" >> tmp.txt
#        echo
#      elif [ $i == "2" ];then
#        gzfactory spawn -f ~/rover_workspace/src/rover_misc/gazebo/models/"${roverNameArray[$i]}"/model.sdf -m "${roverNameArray[$i]}" -x -1.0 -y 0 -z 0 -R 0 -P 0 -Y 3.15;
#        echo "-1.0 0" >> tmp.txt
#        echo
#      elif [ $i == "3" ];then
#        gzfactory spawn -f ~/rover_workspace/src/rover_misc/gazebo/models/"${roverNameArray[$i]}"/model.sdf -m "${roverNameArray[$i]}" -x 0 -y -1.0 -z 0 -R 0 -P 0 -Y #3.15;
#        echo "0 -1.0" >> tmp.txt
#        echo
#      elif [ $i == "4" ];then
#        gzfactory spawn -f ~/rover_workspace/src/rover_misc/gazebo/models/"${roverNameArray[$i]}"/model.sdf -m "${roverNameArray[$i]}" -x 0 -y -1.0 -z 0 -R 0 -P 0 -Y 40;
#        echo "0 -1.0" >> tmp.txt
#        echo
#      fi
#    done
#  else 
#more than four rovers, put them in a grid. Probably a more elegant placement solution out there...
#Came back to this...There is almost certainly a more elegant solution
  for ((i = 0; i < $numberOfRovers; i++));do
    if [ $yCounter -gt 5 ];then
      yCounter=0
    fi

    if [ $i -ge 0 ] && [ $i -le 4 ];then
      #echo "$i rover, at : Y = 0"
      x=0;
    elif [ $i -ge 5 ] && [ $i -le 10 ];then
      #echo "$i rover, at : Y = 1"
      x=1;
    elif [ $i -ge 11 ] && [ $i -le 16 ];then
      #echo "$i rover, at : Y = -1"
      x=-1;
    elif [ $i -ge 17 ] && [ $i -le 22 ];then
      x=2;

    fi
	echo "GOT HERE TOOOOOO"
    echo "$x ${yCoordArray[$yCounter]}" >> tmp.txt
    gzfactory spawn -f ~/rover_workspace/src/rover_misc/gazebo/models/"${roverNameArray[$i]}"/model.sdf -m "${roverNameArray[$i]}" -x $x -y ${yCoordArray[$yCounter]} -z 0 -R 0 -P 0 -Y 0;
    #echo "Spawn rover $i at X: ${xCoordArray[$yCounter]} Y: $y "
    
    yCounter=$((yCounter+1))
    #echo $yCounter
done
    
    
else
echo "IN ELSE ALSO"
  for ((i = 0; i < $numberOfRovers; i++));do
    echo ${roverNameArray[$i]}
    echo ${roverPos[$i]}
    if [ "${roverPos[$i]}" == "0" ];then
    echo 
      if [ "$originOccupied" = true ];then
        echo "Origin is occupied. Spawning ${roverNameArray[$i]} near the origin instead."
          #Assemble around origin
        if [ $i == "1" ];then
          gzfactory spawn -f ~/rover_workspace/src/rover_misc/gazebo/models/"${roverNameArray[$i]}"/model.sdf -m "${roverNameArray[$i]}" -x 0 -y -1.0 -z 0 -R 0 -P 0 -Y -1.55;
          echo "0 -1.0" >> tmp.txt
          echo
          #break
        elif [ $i == "2" ];then
          gzfactory spawn -f ~/rover_workspace/src/rover_misc/gazebo/models/"${roverNameArray[$i]}"/model.sdf -m "${roverNameArray[$i]}" -x 0 -y 1.0 -z 0 -R 0 -P 0 -Y 1.55;
          echo "0 1.0" >> tmp.txt
          echo
          #break
        elif [ $i == "3" ];then
          gzfactory spawn -f ~/rover_workspace/src/rover_misc/gazebo/models/"${roverNameArray[$i]}"/model.sdf -m "${roverNameArray[$i]}" -x -1.0 -y 0 -z 0 -R 0 -P 0 -Y -3.15;
          echo "-1.0 0" >> tmp.txt
          echo
          #break
        elif [ $i == "4" ];then
          gzfactory spawn -f ~/rover_workspace/src/rover_misc/gazebo/models/"${roverNameArray[$i]}"/model.sdf -m "${roverNameArray[$i]}" -x 0 -y -1.0 -z 0 -R 0 -P 0 -Y 40;
          echo "0 -1.0" >> tmp.txt
          echo
          #break
        fi
      else
          echo "Spawning ${roverNameArray[$i]} at origin"
          echo
          gzfactory spawn -f ~/rover_workspace/src/rover_misc/gazebo/models/"${roverNameArray[$i]}"/model.sdf -m "${roverNameArray[$i]}" -x 0 -y 0 -z 0 -R 0 -P 0 -Y 0;
          originOccupied=true
          #break
        fi
      elif [ "${roverPos[$i]}" == "1" ];then
        echo
        echo
        while true;do
          randomLocationCounter=$((randomLocationCounter+1))
          repeats=0
          randomCoordLine=($(sed -n "${randomLocationCounter}p" randomNumbers.txt))
          while read line;do
            existingCoords=( $line )
            deltaX=$(bc <<< "${existingCoords[0]} - ${randomCoordLine[0]}")
            deltaY=$(bc <<< "${existingCoords[1]} - ${randomCoordLine[1]}")
            abs $deltaX
            tooCloseX=$(bc <<< "$absval<0.5")
            abs $deltaY
            tooCloseY=$(bc <<< "$absval<0.5")
            if [ "$tooCloseX" == "1" ] && [ "$tooCloseY" == "1" ];then
              repeats=$((repeats+1))
              echo "Not fine."
            fi
          done < tmp.txt
          if [ "$repeats" -eq "0" ] ;then
            echo "These coordinates work."
            randomXCoord=${randomCoordLine[0]}
            randomYCoord=${randomCoordLine[1]}
            echo "X: $randomXCoord"
            echo "Y: $randomYCoord"
            gzfactory spawn -f ~/rover_workspace/src/rover_misc/gazebo/models/"${roverNameArray[$i]}"/model.sdf -m "${roverNameArray[$i]}" -x "$randomXCoord" -y "$randomYCoord" -z 0 -R 0 -P 0 -Y 0;
            echo "$randomXCoord" "$randomYCoord" >> tmp.txt
            echo
            break
          elif [ "$repeats" -gt "0" ] ;then
            echo "Need to seed new coordinates"
          else
            echo
          fi
        done
        break
      elif [ "${roverPos[$i]}" == "2" ];then
        while true; do
          while true; do
            echo "Enter X coordinate (-9 to 9)";read selectedXCoord
            if [ -z "$selectedXCoord" ]; then
              echo "X coordinate necessary."
            else
             break
            fi 
            echo
          done
          while true; do
            echo "Enter Y coordinate (-9 to 9)"; read selectedYCoord
            if [ -z "$selectedYCoord" ]; then
              echo "Y coordinate necessary."
            else
              break
            fi 
          done
          repeats=0
          while read line;do
            existingCoords=( $line )
            if [ "${existingCoords[0]}" != "$selectedXCoord" ] || [ "${existingCoords[1]}" != "$selectedYCoord" ];then
              echo "Fine"
            else
              repeats=$((repeats+1))
              echo "Not fine."
            fi
          done < tmp.txt
          if [ "$repeats" -eq "0"  ] ;then
            echo "These coordinates work."
            echo "X: $selectedXCoord"
            echo "Y: $selectedYCoord"
            gzfactory spawn -f ~/rover_workspace/src/rover_misc/gazebo/models/"${roverNameArray[$i]}"/model.sdf -m "${roverNameArray[$i]}" -x "$selectedXCoord" -y "$selectedYCoord" -z 0 -R 0 -P 0 -Y 0;
            echo "$selectedXCoord" "$selectedYCoord" >> tmp.txt
            break
          elif [ "$repeats" -gt "0" ] ;then
            echo "Need to seed new coordinates"
            echo "Enter new coordinates"
          else
            echo
          fi
        done
        break
      fi
  done
fi
################################
# START APRIL TAG SPAWN        #
################################
sleep 2
#SINGLE TAG SPAWN, UNIFORM DIST.
if [ "$uniformDist" = true ]; then
  echo "Spawning " "$numberOfTags" " tags"
  for ((i = 0; i < "$numberOfTags"; i++)); do
    while true;do
      #Check for repeats. Looks in tmp.txt to make sure no coordinates are repeated
      randomLocationCounter=$((randomLocationCounter+1))
      repeats=0
      echo "Line Number from randomNumbers.txt: $randomLocationCounter"
      randomCoordLine=($(sed -n "${randomLocationCounter}p" randomNumbers.txt))
     while read line;do
      existingCoords=( $line )
       deltaX=$(bc <<< "${existingCoords[0]} - ${randomCoordLine[0]}")
        deltaY=$(bc <<< "${existingCoords[1]} - ${randomCoordLine[1]}")
        abs $deltaX
        tooCloseX=$(bc <<< "$absval<0.5")
       abs $deltaY
        tooCloseY=$(bc <<< "$absval<0.5")
        if [ "$tooCloseX" == "1" ] && [ "$tooCloseY" == "1" ];then
          repeats=$((repeats+1))
          echo "Not fine."
        fi
      done < tmp.txt
     if [ "$repeats" -eq "0" ] ;then
        echo "These coordinates work."
        randomXCoord=${randomCoordLine[0]}
        randomYCoord=${randomCoordLine[1]}
        echo "X: $randomXCoord"
        echo "Y: $randomYCoord"
        gzfactory spawn -f ~/rover_workspace/src/rover_misc/gazebo/models/at"$i"/model.sdf -m at"$i" -x "$randomXCoord" -y "$randomYCoord" -z 0 -R 0 -P 0 -Y $RANDOM;
        #echo "$randomXCoord" "$randomYCoord" >> tmp.txt
        echo
        break
      elif [ "$repeats" -gt "0" ] ;then
        echo "Need to seed new coordinates"
      else
        echo
     fi
    done
  done
#CLUSTERED TAG SPAWN
elif [ "$clusteredDist" = true ]; then
  echo "Using a clustered method to spawn tags"
  echo "4 piles of 64 tags will be spawned"
  for ((i = 0; i < 4; i++)); do
    while true;do
      #repeat check
      randomLocationCounter=$((randomLocationCounter+1))
      repeats=0
      echo "Line Number from randomNumbers.txt: $randomLocationCounter"
      randomCoordLine=($(sed -n "${randomLocationCounter}p" randomNumbers.txt))
      while read line;do
        existingCoords=( $line )
        deltaX=$(bc <<< "${existingCoords[0]} - ${randomCoordLine[0]}")
        deltaY=$(bc <<< "${existingCoords[1]} - ${randomCoordLine[1]}")
        abs $deltaX
        tooCloseX=$(bc <<< "$absval<0.75")
        abs $deltaY
        tooCloseY=$(bc <<< "$absval<0.75")
        if [ "$tooCloseX" == "1" ] && [ "$tooCloseY" == "1" ];then
          repeats=$((repeats+1))
          echo "Not fine."
        fi
      done < tmp.txt
      if [ "$repeats" -eq "0" ] ;then
        echo "These coordinates work."
        randomXCoord=${randomCoordLine[0]}
        randomYCoord=${randomCoordLine[1]}
        echo "X: $randomXCoord"
        echo "Y: $randomYCoord" 
        gzfactory spawn -f ~/rover_workspace/src/rover_misc/gazebo/models/atags64_"$i"/model.sdf -m atags64_"$i" -x "$randomXCoord" -y "$randomYCoord" -z 0 -R 0 -P 0 -Y $RANDOM;
       # echo "$randomXCoord" "$randomYCoord" >> tmp.txt
        echo
        break
      elif [ "$repeats" -gt "0" ];then
        echo "Need to seed new coordinates"
      else
        echo
      fi
    done
  done
#Power law dist
elif [ "$powerLawDist" = true ]; then
  echo "Using a power law method to spawn tags"
  echo
#Spawns 1 pile of 64 tags 
  for ((i = 0; i < 1; i++)); do
    while true;do
      #Repeat Check
      randomLocationCounter=$((randomLocationCounter+1))
      repeats=0
      echo "Line Number from randomNumbers.txt: $randomLocationCounter"
      randomCoordLine=($(sed -n "${randomLocationCounter}p" randomNumbers.txt))
      while read line;do
        existingCoords=( $line )
        deltaX=$(bc <<< "${existingCoords[0]} - ${randomCoordLine[0]}")
        deltaY=$(bc <<< "${existingCoords[1]} - ${randomCoordLine[1]}")
        abs $deltaX
        tooCloseX=$(bc <<< "$absval<0.75")
#for DEBUG      echo "Too Close X: $tooCloseX"
        abs $deltaY
        tooCloseY=$(bc <<< "$absval<0.75")
#for DEBUG      echo "Too Close Y: $tooCloseY"
        if [ "$tooCloseX" == "1" ] && [ "$tooCloseY" == "1" ];then
          repeats=$((repeats+1))
          echo "Not fine."
        fi
      done < tmp.txt
      if [ "$repeats" -eq "0" ];then
        echo "These coordinates work."
        randomXCoord=${randomCoordLine[0]}
        randomYCoord=${randomCoordLine[1]}
        echo "X: $randomXCoord"
        echo "Y: $randomYCoord"
        #Spawns 1 pile of 64 tags
        gzfactory spawn -f ~/rover_workspace/src/rover_misc/gazebo/models/atags64_"$i"/model.sdf -m atags64_"$i" -x "$randomXCoord" -y "$randomYCoord" -z 0 -R 0 -P 0 -Y $RANDOM;
        #echo "$randomXCoord" "$randomYCoord" >> tmp.txt
        echo
        break
      elif [ "$repeats" -gt "0" ] ;then
        echo "Need to seed new coordinates"
      else
        echo
      fi
    done
  done
#spawns four piles of 16 tags
  for ((i = 0; i < 4; i++)); do
    while true;do
      randomLocationCounter=$((randomLocationCounter+1))
      repeats=0
      echo "Line Number from randomNumbers.txt: $randomLocationCounter"
      randomCoordLine=($(sed -n "${randomLocationCounter}p" randomNumbers.txt))
      while read line;do
        existingCoords=( $line )
        deltaX=$(bc <<< "${existingCoords[0]} - ${randomCoordLine[0]}")
        deltaY=$(bc <<< "${existingCoords[1]} - ${randomCoordLine[1]}")
        abs $deltaX
        tooCloseX=$(bc <<< "$absval<0.75")
#for DEBUG      echo "Too Close X: $tooCloseX"
        abs $deltaY
        tooCloseY=$(bc <<< "$absval<0.75")
#for DEBUG      echo "Too Close Y: $tooCloseY"
        if [ "$tooCloseX" == "1" ] && [ "$tooCloseY" == "1" ];then
          repeats=$((repeats+1))
          echo "Not fine."
        fi
      done < tmp.txt
      if [ "$repeats" -eq "0" ];then
        echo "These coordinates work."
        randomXCoord=${randomCoordLine[0]}
        randomYCoord=${randomCoordLine[1]}
        echo "X: $randomXCoord"
        echo "Y: $randomYCoord"
        #Spawns 1 pile of 64 tags
        gzfactory spawn -f ~/rover_workspace/src/rover_misc/gazebo/models/atags16_"$i"/model.sdf -m atags16_"$i" -x "$randomXCoord" -y "$randomYCoord" -z 0 -R 0 -P 0 -Y $RANDOM;
       # echo "$randomXCoord" "$randomYCoord" >> tmp.txt
        echo
        break
      elif [ "$repeats" -gt "0" ] ;then
        echo "Need to seed new coordinates"
      else
        echo
      fi
    done
  done
#spawns 16 piles of four tags
  for ((i = 0; i < 16; i++)); do
    while true;do
      randomLocationCounter=$((randomLocationCounter+1))
      repeats=0
      echo "Line Number from randomNumbers.txt: $randomLocationCounter"
      randomCoordLine=($(sed -n "${randomLocationCounter}p" randomNumbers.txt))
      while read line;do
        existingCoords=( $line )
        deltaX=$(bc <<< "${existingCoords[0]} - ${randomCoordLine[0]}")
        deltaY=$(bc <<< "${existingCoords[1]} - ${randomCoordLine[1]}")
        abs $deltaX
        tooCloseX=$(bc <<< "$absval<0.75")
#for DEBUG      echo "Too Close X: $tooCloseX"
        abs $deltaY
        tooCloseY=$(bc <<< "$absval<0.75")
#for DEBUG      echo "Too Close Y: $tooCloseY"
        if [ "$tooCloseX" == "1" ] && [ "$tooCloseY" == "1" ];then
          repeats=$((repeats+1))
          echo "Not fine."
        fi
      done < tmp.txt
      if [ "$repeats" -eq "0" ];then
        echo "These coordinates work."
        randomXCoord=${randomCoordLine[0]}
        randomYCoord=${randomCoordLine[1]}
        echo "X: $randomXCoord"
        echo "Y: $randomYCoord"
        #Spawns 1 pile of 64 tags
        gzfactory spawn -f ~/rover_workspace/src/rover_misc/gazebo/models/atags4_"$i"/model.sdf -m atags4_"$i" -x "$randomXCoord" -y "$randomYCoord" -z 0 -R 0 -P 0 -Y $RANDOM;
        #echo "$randomXCoord" "$randomYCoord" >> tmp.txt
        echo
        break
      elif [ "$repeats" -gt "0" ] ;then
        echo "Need to seed new coordinates"
      else
        echo
      fi
    done
  done
#Spawns the rest of the single tags
  for ((i = 192; i < 256; i++)); do
    while true;do
      randomLocationCounter=$((randomLocationCounter+1))
      repeats=0
      echo "Line Number from randomNumbers.txt: $randomLocationCounter"
      randomCoordLine=($(sed -n "${randomLocationCounter}p" randomNumbers.txt))
      while read line;do
        existingCoords=( $line )
        deltaX=$(bc <<< "${existingCoords[0]} - ${randomCoordLine[0]}")
        deltaY=$(bc <<< "${existingCoords[1]} - ${randomCoordLine[1]}")
        abs $deltaX
        tooCloseX=$(bc <<< "$absval<0.75")
#for DEBUG      echo "Too Close X: $tooCloseX"
        abs $deltaY
        tooCloseY=$(bc <<< "$absval<0.75")
#for DEBUG      echo "Too Close Y: $tooCloseY"
        if [ "$tooCloseX" == "1" ] && [ "$tooCloseY" == "1" ];then
          repeats=$((repeats+1))
          echo "Not fine."
        fi
      done < tmp.txt
      if [ "$repeats" -eq "0" ];then
        echo "These coordinates work."
        randomXCoord=${randomCoordLine[0]}
        randomYCoord=${randomCoordLine[1]}
        echo "X: $randomXCoord"
        echo "Y: $randomYCoord"
        #Spawns 1 pile of 64 tags
        gzfactory spawn -f ~/rover_workspace/src/rover_misc/gazebo/models/at"$i"/model.sdf -m at"$i" -x "$randomXCoord" -y "$randomYCoord" -z 0 -R 0 -P 0 -Y $RANDOM;
        echo
      #  echo "$randomXCoord" "$randomYCoord" >> tmp.txt
        break
      elif [ "$repeats" -gt "0" ] ;then
        echo "Need to seed new coordinates"
      else
        echo
      fi
    done
  done
fi
################################
# START OBSTACLE SPAWN         #
################################

sleep 2
echo "Spawning " "$numberOfObstacles" " obstacles"
for ((i = 0; i < "$numberOfObstacles"; i++)); do
  while true;do
    randomLocationCounter=$((randomLocationCounter+1))
    repeats=0
    randomCoordLine=($(sed -n "${randomLocationCounter}p" randomNumbers.txt))
    while read line;do
      existingCoords=( $line )
      deltaX=$(bc <<< "${existingCoords[0]} - ${randomCoordLine[0]}")
      deltaY=$(bc <<< "${existingCoords[1]} - ${randomCoordLine[1]}")
      abs $deltaX
      tooCloseX=$(bc <<< "$absval<0.5")
#for DEBUG      echo "Too Close X: $tooCloseX"
      abs $deltaY
      tooCloseY=$(bc <<< "$absval<0.5")
#for DEBUG      echo "Too Close Y: $tooCloseY"
      if [ "$tooCloseX" == "1" ] && [ "$tooCloseY" == "1" ];then
        repeats=$((repeats+1))
        echo "Not fine."
      fi
    done < tmp.txt
    if [ "$repeats" -eq "0" ] ;then
      echo "These coordinates work."
      randomXCoord=${randomCoordLine[0]}
      randomYCoord=${randomCoordLine[1]}
      echo "X: $randomXCoord"
      echo "Y: $randomYCoord"
      gzfactory spawn -f ~/rover_workspace/src/rover_misc/gazebo/models/swarmie_obstacle/model.sdf -m swarmie_obstacle_"$i" -x "$randomXCoord" -y "$randomYCoord" -z 0 -R 0 -P 0 -Y 0;
     # echo "$randomXCoord" "$randomYCoord" >> tmp.txt
      echo
      break
    elif [ "$repeats" -gt "0" ] ;then
      echo "Need to seed new coordinates"
    else
      echo
    fi
  done
done


#Kills the processes
#This is an infinite loop and will only accept the char 'q' as exit command. All other commands will be ignored and the processes will still running
#To spawn new objects or to start more rosnodes, you will need to open a separate terminal. This terminal will be locked.

while true; do
echo "Quit? [q]";
read answer;

  if [ "$answer" == "q" ];then
	   #Added sleeps and incremental shutdown to avoid gazebo and ros errors
           echo "Quit"
	   pkill gzclient
           sleep 1
           pkill gzserver
           sleep 1
           pkill rqt
           sleep 1
	   pkill roslaunch
           sleep 1
	   pkill rosmaster
           sleep 1
           pkill roscore
           sleep 1
           echo
           pkill world_state
           echo
           echo "Done. Safe to start script again."
           echo
	   #mv *.csv logs/
           #echo "Moved log file to logs/"
           
           exit
  else
    	   echo "Command Not Recognized..."
  fi
done


exit 1
