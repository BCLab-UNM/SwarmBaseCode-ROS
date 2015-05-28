#!/bin/bash

#************************************************#
#         rover_onboard_node_launch.sh           #
#           written by Gilbert Montague          #
#                July 30, 2014                   #
#                                                #
#           Starts rqt and roscore on driver     #
#************************************************#
#Syntax:
#./driver_launch [rovers you want to start up]

#Example:
#./driver_launch larry curly shemp moe

#Assumes the network is up and running and all of the rovers are on and ready to be started up.

roverNameArray=()

################################
# ENVIRONMENT                  #
################################
source ~/rover_driver_workspace/devel/setup.bash
source ~/rover_onboard_workspace/devel/setup.bash
# Don't ask why.  This just works.  You would do well to do this also in your own ~/.bashrc
source ~/rover_driver_workspace/devel/setup.bash

echo "Starting roscore..."
nohup roscore &
echo "Started roscore..."
sleep 2
nohup rosrun rover_driver_world_state world_state &
echo "Started World State"
sleep 2
echo "Starting rqt..." 
nohup rqt &
echo "Started rqt..."
sleep 2

#Start the onboard processes. Assume that the rovers are on and ready to go...
#Bad assumtion...?
#!/bin/bash

for var in "$@";do
  if [[ -n ${var//[0-9]/} ]];then
    roverNameArray=("${roverNameArray[@]}" "$var")
    echo ${roverNameArray[@]}
  fi
done
for swarmies in "${roverNameArray[@]}";do
  echo "ssh into $swarmies and starting all processes. Standby"
  gnome-terminal -e ""./remote_node_launch.sh" "ksc" "$swarmies"" &
  sleep 5
done

while true; do
echo "Who would you like to run the joystick with?";
read answer;


if [ "$answer" == "larry" ] || [ "$answer" == "curly" ] || [ "$answer" == "shemp" ] || [ "$answer" == "moe" ];then
    echo
    echo "Starting joystick driver for "$answer"..."
    nohup rosrun rover_driver_rqt_motor joystick_driver "$answer" &
    echo "Press start to drive "$answer""
    break
  elif [ "$answer" == "q" ];then
    echo "quit"
    pkill rqt
    pkill roscore
    pkill rosmaster
    pkill world_state
    mv *.csv ~/swarmieTestLogs
    exit 1    
  else
    echo "Name not recognized..."
    echo "Choose larry, curly, shemp, or moe"
  fi
done

while true; do
echo "Close all driver processes. [q]"
read choice;

  if [ "$choice" == "q" ];then
    echo
    pkill rqt
    pkill roscore
    pkill rosmaster
    pkill joystick_driver
    pkill world_state
    mv *.csv ~/swarmieTestLogs
    echo "Goodbye."
    exit 1
  else
    echo
    echo "Command not recognized. Press [q] to close all driver processes."
  fi
done

exit 1

