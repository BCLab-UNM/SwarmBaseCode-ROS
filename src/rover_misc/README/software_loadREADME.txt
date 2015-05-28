NOTE: You must be connected to the "robot" wireless router and the Swarmie that you want to load the software onto must be turned on.
NOTE: /usr/bin/expect must be installed before running this script. To get expect, run this command in a terminal window:

sudo apt-get install expect


SYNTAX

./software_load [HOME USERNAME] [HOME HOSTNAME] [SWARMIE NETWORK] [USER (Swarmie name)]

EXAMPLES

./software_load ksc driverStation ksc moe
./software_load gmontague swarmie1 ksc shemp

Current  Swarmie network name is: ksc

This script remotely uploads the onboard software necessary to operate a Swarmie Rover.

Additional packages will need to be manually added to the script. 

The script will first ssh into the selected Swarmie and remove the build and devel directories in rover_onboard_workspace

After removing those directories, it will remove the following packages from the src directory of rover_onboard_workspace:

rover_onboard_abridge
rover_onboard_localization
rover_onboard_mobility
rover_onboard_obstacle_detection
rover_onboard_path_planning
rover_onboard_target_detection
rover_onboard_zmapping

After removing those directories, it will run a catkin_make in rover_onboard_workspace and then source rover_onboard_workspace/devel/setup.bash

After sourcing the devel/setup.bash, the script will close the ssh session and "return" back to the home computer to scp all of the new packages onto the Swarmie. Currently, the script scp's these packages onto the rover:

rover_onboard_abridge
rover_onboard_localization
rover_onboard_mobility
rover_onboard_obstacle_detection
rover_onboard_path_planning
rover_onboard_target_detection
rover_onboard_zmapping

It is necessary to make sure that these packages exist in the ~/rover_onboard_workspace/src directory on the home computer to ensure the scp works correctly.

After scping all of the packages onto the Swarmie, the script will start another ssh session with the Swarmie and copy the onboard copy of "libapriltags.a" into the onboard directory "~/rover_onboard_workspace/src/rover_onboard_target_detection/lib/"

It will then copy the rover_onboard_node_launch.sh script to the ~/ directory onboard. The script must be located in ~/rover_misc_workspace/src/rover_scripts offboard.

After copying the library over, it will run a catkin_make twice to ensure all of the packages were built correctly.

Finally, it will source the rover_onboard_workspace/devel/setup.bash and the script will exit but will leave the ssh session open. 

It is recommended to run the remote_node_startup.sh script and test all of the packages after updating the software. 


TODO:
Alter the script to iterate through the rover_onboard_workspace directory to scopy all of the packages so that packages do not need to be added manually.


