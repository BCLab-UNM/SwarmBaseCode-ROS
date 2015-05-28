SYNTAX

./rover_onboard_node_launch.sh


This script is to be located on the ~/ directory of all of the Swarmie rovers. This script will start the following rosnodes onboard the Swarmie:

rover_onboard_abridge abridge
rover_onboard_localization localization
rover_onboard_mobility mobility
rover_onboard_obstactle_detection obstacle
rover_onboard_path_planning path
rover_onboard_target_detection target
rover_onboard_target_detection camera
rover_onboard_zmapping mapping

No arguments are necessary as the script will use the hostname of the Swarmie to start the nodes. A roscore process needs to be running on another computer conneceted to the network (i.e. driverStation).

This script, after starting all of the nodes, will wait for the user to quit out by entering 'q'. Any other commands are not recognized.

When the user enters 'q', the script will close all of the rosnodes started by the script and then exit. You will be left with prompt in the ~/ directory of the Swarmie you ssh into upon quitting out of the script.

Even though this script is mostly used by the remote_node_launch.sh script, you can start this script onboard a Swarmie via an ssh session.

A copy of the script is located in the rover_scripts directory.
