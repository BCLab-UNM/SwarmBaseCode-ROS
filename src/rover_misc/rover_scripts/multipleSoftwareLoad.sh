#!/bin/bash
logDir=$(date +%F--%T)
roverNameArray=()

#Make a log directory for the past software load
#if [ -d ~/rover_onboard_workspace ];then
#  mkdir ~/$logDir
#  mv /home/$USER/rover_onboard_workspace/ ~/$logDir
#fi
#if [ -d ~/rover_driver_workspace ];then
#  if [ ! -d ~/$logDir ];then
#    mkdir /home/$USER/$logDir
#  fi
#  mv /home/$USER/rover_driver_workspace/  ~/$logDir
#fi

#if [ -d ~/$logDir ];then
#  mv ~/$logDir ~/archive/
#fi

#Correctly build the workspaces using an existing script
#./catkin_make_rover.sh

#Load the software onto the swarmies
for var in "$@";do
  if [[ -n ${var//[0-9]/} ]];then
    roverNameArray=("${roverNameArray[@]}" "$var")
    echo ${roverNameArray[@]}
  fi
done
for swarmies in "${roverNameArray[@]}";do
    gnome-terminal -e "./softwareLoad.sh "$USER" "$HOSTNAME" "ksc" "$swarmies"" &
    sleep 10
done
