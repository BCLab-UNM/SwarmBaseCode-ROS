This directory contains all of the programs and scripts to:

Create a Swarmie for Gazebo Simulation
Launch Gazebo with obstacles, Swarmies, and tags
Remotely start rosnodes on Swarmies
Remotely update Swarmie onboard software

***********************************************
gazebo directory:
***********************************************
The gazebo directory contains the program, robotcreator, that creates a Swarmie for Gazebo. It is located under the SwarmieCreator directory. The entire Netbeans project is located in the SwarmieCreatorNB directory.

This directory (gazebo/) should be placed in your ~/rover_misc_workspace/src/ directory. If you find that the program robotcreator is not executable, type this command while in the directory that contains robotcreator.

chmod 777 robotcreator

***********************************************
rover_driver_gazebo_launch:
***********************************************

This directory contains everything that is necessary to launch Gazebo with Swarmies, tags, and obstacles. You first need to make sure that a ros package exists in your rover_driver_workspace/src titled rover_driver_gazebo_launch. So, in your rover_driver_workspace/src directory type the command

catkin_create_pkg rover_driver_gazebo_launch

and then copy and paste the directory rover_driver_gazebo_launch and overwrite the package you just created. cd to ~/rover_driver_workspace/ and run a catkin_make to complete the process.

Also, if you find that any of the scripts or programs are not executable, run the following command in the directory where the program or script it

chmod 777 [PROGRAM NAME]

It is also important to make sure your ~/.bashrc file is correct. The bottom of your ~/.bashrc should look like this:

source /opt/ros/hydro/setup.bash
source ~/rover_driver_workspace/devel/setup.bash
source ~/rover_onboard_workspace/devel/setup.bash
source /usr/share/gazebo-1.9/setup.sh
export GAZEBO_MODEL_PATH=/home/USERNAME/rover_misc_workspace/src/gazebo/models

Instead of USERNAME, insert the correct name for your machine. After making the changes, make sure to:

source ~/.bashrc

***********************************************
rover_scripts
***********************************************

This directory contains the scripts to launch the rosnodes on the physical Swarmies and the script to load the software onto the Swarmies.

This should be placed in the ~/rover_misc_workspace/src directory. You must make sure that the paths to the correct packages are correct and you must also read the READMEs associated with the scripts.

If any of the scripts are not executable, use the 

chmod 777 [SCRIPT NAME]

command to make them executable.

***********************************************
README
***********************************************

This directory contains all of the READMEs associated with the scripts and programs. It is highly recommended that you review these documents before using the scripts and programs.




