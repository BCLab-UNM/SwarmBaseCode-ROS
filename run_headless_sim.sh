#!/bin/bash

# Function definitions
startGazeboServer()
{
    local world_file_path = $1
    rosparam set /use_sim_time true
    rosrun gazebo_ros gzserver world_file_path
    echo "Attempted to start Gazebo server with world file: $1" 
}

stopGazeboServer()
{
    pkill gzserver
    echo "Attempted to stop Gazebo server" 
}

startGazeboClient()
{
    rosrun gazebo_ros gzclient __name:=gzclient
    echo "Attempted to start Gazebo client"
}

stopGazeboClient()
{
    pkill gzclient
    echo "Attempted to stop Gazebo client"
}

# Stops the ROS nodes associated with rovers
startRoverNode()
{
    local rover_name = $1
    roslaunch $PWD/launch/swarmie.launch name:=$rover_name
    echo "Attempted to start rover ROS nodes"
}

# Stops the ROS nodes associated with rovers
stopRoverNode()
{
    local rover_name = $1
    rosnode kill rover_name_APRILTAG
    rosnode kill rover_name_BASE2CAM
    rosnode kill rover_name_DIAGNOSTICS
    rosnode kill rover_name_MAP
    rosnode kill rover_name_BEHAVIOUR
    rosnode kill rover_name_SBRIDGE
    rosnode kill rover_name_NAVSAT
    rosnode kill rover_name_OBSTACLE
    rosnode kill rover_name_ODOM
    echo "Attempted to kill rover ROS nodes"
}

addRover()
{
    local rover_name = $1
    local x = $2
    local y = $3
    local z = $4
    local roll = $5
    local pitch = $6
    local yaw = $7
    
    rosrun gazebo_ros spawn_model -sdf -file $PWD/simulation/models/$rover_name/model.sdf \
               -model $rover_name \
               -x x \
               -y y \
               -z z \
               -R roll \
               -P pitch \
               -Y yaw
}

if [ $# -ne 5 -o $# -ne 4 ]
then
    echo "Usage: $0 world_file_path num_rovers(1-6) scoring_output_path experiment_time_in_minutes [visualize]"
    echo "Example: $0 ../simulation/worlds/powerlaw_targets_example.world 6 ~/swarmathon_data/experiment1.txt 30 visualize"
    echo "Example: $0 ../simulation/worlds/powerlaw_targets_example.world 6 ~/swarmathon_data/experiment1.txt 30"   
   exit 1
fi
   
echo "Running in $PWD" 
previous_gazebo_model_path=${GAZEBO_MODEL_PATH}
previous_gazebo_plugin_path=${GAZEBO_PLUGIN_PATH}
export SWARMATHON_APP_ROOT="$PWD"
export GAZEBO_MODEL_PATH="$PWD/simulation/models"
export GAZEBO_PLUGIN_PATH="$PWD/build/gazebo_plugins"
source "$PWD/devel/setup.bash"
echo Cleaning up ROS and Gazebo Processes
./cleanup.sh
echo Killing rosmaster
pkill rosmaster
echo Killing roscore
pkill roscore
roscore &
sleep 2

#---------------------------------------------------------#

# Read the world file path from command line
WORLD_FILE_PATH = $1

# Start the gazebo simulation
startGazeboServer WORLD_FILE_PATH

# Start the gazebo simulation
if [$# -eq 5 -a $5 -eq "visualize"]
   then
       startGazeboClient
fi

# Get the current sim time: start time

# Load the world file

#---------------------------------------------------------#

# Read the number of rovers to create from command line
NUM_ROVERS = $2

# Specify rover names


for 


# Add the rovers to the simulation

#---------------------------------------------------------#

# Send the autonomous command to all rovers

# Read output file path from command line
SCORE_OUTPUT_PATH = $3

# Pipe /score ros topic to output file

#---------------------------------------------------------#

# Read the experiment time from command line
EXPERIMENT_TIME = $4

# Loop --

#      Poll gazebo for current sim time

#      If current time time - start time exceeds the experiment time end simulation 

#---------------------------------------------------------#



# The rover program cleans up after itself but if there is a crash this helps to make sure there are no leftovers
echo Cleaning up ROS and Gazebo Processes
rosnode kill -a 
echo Killing rosmaster
pkill rosmaster
echo Killing roscore
pkill roscore
./cleanup.sh
# Restore previous environment
export GAZEBO_MODEL_PATH=$previous_gazebo_model_path
export GAZEBO_PLUGIN_PATH=$previous_gazebo_plugin_path
