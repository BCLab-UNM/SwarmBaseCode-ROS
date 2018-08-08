#!/bin/bash

# Function definitions

function userExit()
{
    echo "Received SIGINT. Exiting."
    rosnode kill -all
    ./cleanup.sh
    exit
}

startGazeboServer()
{
    local world_file_path=$1
    rosparam set /use_sim_time true
    setsid rosrun gazebo_ros gzserver $world_file_path &
    echo "Attempted to start Gazebo server with world file: $1"
}

stopGazeboServer()
{
    pkill gzserver
    echo "Attempted to stop Gazebo server" 
}

startGazeboClient()
{
    setsid rosrun gazebo_ros gzclient __name:=gzclient &
    echo "Attempted to start Gazebo client"
}

stopGazeboClient()
{
    pkill gzclient
    echo "Attempted to stop Gazebo client"
}

addCollectionZone()
{
setsid rosrun gazebo_ros spawn_model -sdf -file $PWD/simulation/models/collection_disk/model.sdf \
               -model collection_disk \
               -x 0 \
               -y 0 \
               -z 0 \
               -R 0 \
               -P 0 \
               -Y 0
    echo "Attempted to add collection_zone: name=collection_disk, x=0, y=0, z=0, roll=0, pitch=0, yaw=0"
}

addGroundPlane()
{
setsid rosrun gazebo_ros spawn_model -sdf -file $PWD/simulation/models/concrete_ground_plane/model.sdf \
               -model concrete_ground_plane \
               -x 0 \
               -y 0 \
               -z 0 \
               -R 0 \
               -P 0 \
               -Y 0
    echo "Attempted to add concrete ground plane: name=concrete_ground_plane, x=0, y=0, z=0, roll=0, pitch=0, yaw=0"
}

# Stops the ROS nodes associated with rovers
startRoverNodes()
{
    local rover_name=$1
    setsid roslaunch $PWD/launch/swarmie.launch name:=$rover_name > logs/$rover_name.log &
    echo "Attempted to start rover ROS nodes"
}

# Stops the ROS nodes associated with rovers
stopRoverNodes()
{
    local rover_name=$1
    rosnode kill rover_name_APRILTAG
    rosnode kill rover_name_BASE2CAM
    rosnode kill rover_name_DIAGNOSTICS
    rosnode kill rover_name_MAP
    rosnode kill rover_name_BEHAVIOUR
    rosnode kill rover_name_SBRIDGE
    rosnode kill rover_name_NAVSAT
    rosnode kill rover_name_ODOM

    rosnode cleanup
    echo "Attempted to kill rover ROS nodes: name=$rover_name"
}

addRover()
{
    local rover_name=$1
    local x=$2
    local y=$3
    local z=$4
    local roll=$5
    local pitch=$6
    local yaw=$7
    
    setsid rosrun gazebo_ros spawn_model -sdf -file $PWD/simulation/models/$rover_name/model.sdf \
           -model $rover_name \
           -x $x \
           -y $y \
           -z $z \
           -R $roll \
           -P $pitch \
           -Y $yaw
    echo "Attempted to add rover: name=$rover_name, x=$x, y=$y, z=$z, roll=$roll, pitch=$pitch, yaw=$yaw"
}

#---------------------------------------------------------#
#
#  The top level script
#
#
#---------------------------------------------------------#

# Exit script if user enters ctl-c or sends interrupt
trap userExit SIGINT

# If not given 4 or 5 arguments then show the usage text
if [ $# -ne 5 -a $# -ne 4 ]
then
    echo "Usage: $0 world_file_path num_rovers(1-8) scoring_output_path experiment_duration_in_minutes [visualize]"
    echo "Example: $0 simulation/worlds/powerlaw_targets_example.world 6 ~/swarmathon_data/experiment1.txt 30 visualize"
    echo "Example: $0 simulation/worlds/powerlaw_targets_example.world 6 ~/swarmathon_data/experiment1.txt 30"   
   exit 1
fi

EXPERIMENT_REAL_SETUP_START_TIME_IN_SECONDS=$(date +%s)

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

echo "Experiment started at $(date +%d-%m-%Y" "%H:%M:%S)."

#---------------------------------------------------------#
# Set the interval at which to check whether the experiment duration has elapsed
# The following line set the interval to 1 minute
END_EXPERIMENT_CHECK_INTERVAL=1s

# Delay between adding rovers
# The following line set the interval to 2 seconds
MODEL_ADD_INTERVAL=2s

#---------------------------------------------------------#
# The maximum number of rovers a user can request is currently
MAX_ROVERS=8

#---------------------------------------------------------#
# Read the world file path from command line
WORLD_FILE_PATH=$1
echo "World file path: $WORLD_FILE_PATH"

# Start the gazebo simulation
startGazeboServer $WORLD_FILE_PATH

# Start the gazebo simulation
if [ $# -eq 5 -a "$5" = "visualize" ]
then
    echo "User requested that the Gazebo client be started"
    startGazeboClient
fi

# Read the number of rovers to create from command line
NUM_ROVERS=$2
echo "The user requested $NUM_ROVERS rovers."
if [[ $NUM_ROVERS -gt $MAX_ROVERS ]]; then
    echo "User requested too many rovers. Maximum rovers is $MAX_ROVERS. Exiting."
    exit 2
fi

#---------------------------------------------------------#

addCollectionZone

addGroundPlane


#---------------------------------------------------------#

# Add the rovers to the simulation

#  The distances that determine the X, Y coords of the rovers is determined as follows:
#  The distance to the rover from a corner position is calculated differently
#  than the distance to a cardinal position.
# 
#  The cardinal direction rovers are a straightforward calculation where:
#      a = the distance to the edge of the collection zone
#          i.e., 1/2 of the collection zone square side length
#      b = the 50cm distance required by the rules for placing the rover
#      c = offset for the simulation for the center of the rover (30cm)
#          i.e., the rover position is at the center of its body
# 
#  The corner rovers use trigonometry to calculate the distance where each
#  value of d, e, and f, are the legs to an isosceles right triangle. In
#  other words, we are calculating and summing X and Y offsets to position
#  the rover.
#      d = a
#      e = xy offset to move the rover 50cm from the corner of the collection zone
#      f = xy offset to move the rover 30cm to account for its position being
#          calculated at the center of its body
# 
#                        *  *          d = 0.508m
#                      *      *        e = 0.354m
#                    *          *    + f = 0.212m
#                  *     /*     *    ------------
#                  *    / | f *            1.072m
#                    * /--| *
#                     /* *
#                    / | e
#                   /--|
#      *************
#      *          /|
#      *         / |
#      *        /  | d                 a = 0.508m
#      *       /   |     *********     b = 0.500m
#      *      /    |     *       *   + c = 0.300m
#      *     *-----|-----*---*   *   ------------
#      *        a  *  b  * c     *         1.308m
#      *           *     *********
#      *           *
#      *           *
#      *           *
#      *************

# Specify rover names
ROVER_NAMES=( "achilles" "aeneas" "ajax" "diomedes" "hector" "paris" "thor" "zeus" )

# Specify rover start coordinates
ROVER_POSITIONS_X=( -1.308 0.000 1.308 0.000 1.072 -1.072 -1.072 1.072 )
ROVER_POSITIONS_Y=( 0.000 -1.308 0.000 1.308 1.072 -1.072 1.072 -1.072 )
 
# In this case, the yaw is the value that turns rover "left" and "right" */
ROVER_YAWS=( 0.000 1.571 -3.142 -1.571 -2.356 0.785 -0.785 2.356 )

echo "Adding rovers to Gazebo and starting their ROS nodes..."

# Add rovers to the simulation and start the associated ROS nodes
for (( i=0;i<$NUM_ROVERS;i++ ));
do
    sleep $MODEL_ADD_INTERVAL
    addRover ${ROVER_NAMES[i]} ${ROVER_POSITIONS_X[i]} ${ROVER_POSITIONS_Y[i]} 0 0 0 ${ROVER_YAWS[i]}
    sleep $MODEL_ADD_INTERVAL
    startRoverNodes ${ROVER_NAMES[i]}
done

echo "Finished adding rovers."

#---------------------------------------------------------#

sleep $MODEL_ADD_INTERVAL

echo "Setting rovers to autonomous mode..."

# Send the autonomous command to all rovers
for (( i=0;i<$NUM_ROVERS;i++ ));
do
    # Publish the autonomous mode command ("2") to each rover. Latch the message ("-l").
    rostopic pub -l /${ROVER_NAMES[i]}/mode std_msgs/UInt8 2 &
    echo "Publishing 2 on /${ROVER_NAMES[i]}/mode"
done
 
echo "Finished setting rovers to autonomous mode."

# Read output file path from command line
SCORE_OUTPUT_PATH=$3

mkdir -p $(dirname $SCORE_OUTPUT_PATH)

echo "User specified $SCORE_OUTPUT_PATH as the file to which score information should be appended."

#---------------------------------------------------------#

# Read the experiment time from command line
EXPERIMENT_DURATION_IN_MINUTES=$4
EXPERIMENT_DURATION_IN_SECONDS=$(( $EXPERIMENT_DURATION_IN_MINUTES*60 ))

echo "Experiment duration will be $EXPERIMENT_DURATION_IN_SECONDS seconds."

# Read the current sim time (in seconds) from the ros topic /clock
# Uses awk to match on first occurence of "secs: number".
# {print $2} prints the number to the shell variable 
# exit the awk after first match because we dont care about "nsecs: number"
CURRENT_TIME=$(rostopic echo -n 1 /clock | awk '/secs: [0-9]+$/{print $2; exit}')
START_TIME=$(rostopic echo -n 1 /clock | awk '/secs: [0-9]+$/{print $2; exit}')

echo "Initialised current gazebo time to $CURRENT_TIME"
echo "Initialised start gazebo time to $START_TIME"

# Let the simulation run until the experiment duration is reached
YELLOW='\033[0;33m'
PURPLE='\033[0;35m'
NC='\033[0m' # No Color

EXPERIMENT_REAL_SETUP_END_TIME_IN_SECONDS=$(date +%s)
EXPERIMENT_REAL_START_TIME_IN_SECONDS=$(date +%s)

#Collect the score over time and write to screen and file
echo "Time (s), Score\n" >> $SCORE_OUTPUT_PATH

until (( $CURRENT_TIME-$START_TIME>=$EXPERIMENT_DURATION_IN_SECONDS )); do
    # Update the current sim time
    CURRENT_TIME=$(rostopic echo -n 1 /clock | awk '/secs: [0-9]+$/{print $2; exit}')
    echo -e "Experiment time remaining ${YELLOW}$(( $EXPERIMENT_DURATION_IN_SECONDS-($CURRENT_TIME-$START_TIME) ))${NC} seconds."
    # Pipe /score ros topic to output file
    echo -e "${PURPLE}Time: $(($CURRENT_TIME-$START_TIME)), Score: $(rostopic echo -n 1 /collectionZone/score | sed 's/[^0-9]*//g')${NC}"
    echo "$(($CURRENT_TIME-$START_TIME)), $(rostopic echo -n 1 /collectionZone/score | sed 's/[^0-9]*//g')\n" >> $SCORE_OUTPUT_PATH
    
    sleep $END_EXPERIMENT_CHECK_INTERVAL
done

echo "The specified experiment duration ($EXPERIMENT_DURATION_IN_MINUTES) has elapsed. End autonomous mode for all rovers."

# Send the autonomous command to all rovers
for (( i=0;i<$NUM_ROVERS;i++ ));
do
    # Publish the autonomous mode command ("2") to each rover. Latch the message ("-l").
    rostopic pub -l /${ROVER_NAMES[i]}/mode std_msgs/String "2" &
    echo "Publishing 1 on /${ROVER_NAMES[i]}/mode"
done

EXPERIMENT_REAL_END_TIME_IN_SECONDS=$(date +%s)

ELAPSED_REAL_TIME_IN_SECONDS=$(( $EXPERIMENT_REAL_END_TIME_IN_SECONDS-$EXPERIMENT_REAL_START_TIME_IN_SECONDS ))
ELAPSED_REAL_TIME_IN_MINUTES=$(( $ELAPSED_REAL_TIME_IN_SECONDS/60 ))
ELAPSED_SETUP_REAL_TIME_IN_SECONDS=$(( $EXPERIMENT_REAL_SETUP_END_TIME_IN_SECONDS-$EXPERIMENT_REAL_SETUP_START_TIME_IN_SECONDS ))
ELAPSED_SETUP_REAL_TIME_IN_MINUTES=$(( $ELAPSED_SETUP_REAL_TIME_IN_SECONDS/60 ))
ELAPSED_TOTAL_TIME_IN_SECONDS=$(( $ELAPSED_SETUP_REAL_TIME_IN_SECONDS+$ELAPSED_REAL_TIME_IN_SECONDS ))
ELAPSED_TOTAL_TIME_IN_MINUTES=$(( $ELAPSED_TOTAL_TIME_IN_SECONDS/60 ))

echo "Experiment setup took $ELAPSED_SETUP_REAL_TIME_IN_SECONDS seconds."
echo "Experiment setup took $ELAPSED_SETUP_REAL_TIME_IN_MINUTES minutes."

echo "Experiment run took $ELAPSED_REAL_TIME_IN_SECONDS seconds."
echo "Experiment run took $ELAPSED_REAL_TIME_IN_MINUTES minutes."

echo "Experiment run took $ELAPSED_TOTAL_TIME_IN_SECONDS seconds."
echo "Experiment run took $ELAPSED_TOTAL_TIME_IN_MINUTES minutes."

echo "Finished placing all rovers into manual mode. Ending simulation..."

# Report some simulation efficiency information
echo "The $EXPERIMENT_DURATION_IN_MINUTES minute long experiment took $ELAPSED_REAL_TIME_IN_MINUTES minutes of real time to simulate with $ELAPSED_SETUP_REAL_TIME_IN_MINUTES minutes of setup time."

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

echo "Experiment finished at $(date +%d-%m-%Y" "%H:%M:%S)."
