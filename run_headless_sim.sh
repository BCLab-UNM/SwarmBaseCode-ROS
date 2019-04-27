#!/bin/bash
#Thanks to G. Matthew Fricke (mfricke@cs.unm.edu) for the initial script
#Return Codes
#return 0 all good
#return 1 usage
#return 2 kill signal received
#return 3 bad argument
#return 4-8 UNUSED
#return 9 Score topic not publishing
#return 10-18 rover's status or targets/image/compressed not publishing

# Function definitions
function userExit() {
	code=$1
	echo -e "$red Exit code: $1 $reset"
	if [ $# -eq 0 ]
		then
		code=2
		echo -e "$red Received SIGINT. Exiting. $reset"
	fi
	echo Cleaning up ROS and Gazebo Processes
	rosnode kill -a
	pkill rosmaster
	pkill roscore
	chmod +x $(catkin locate)/cleanup.sh
	$(catkin locate)/cleanup.sh 
	
	# Restore previous environment
	export GAZEBO_MODEL_PATH=$previous_gazebo_model_path
	export GAZEBO_PLUGIN_PATH=$previous_gazebo_plugin_path
	echo -e "$cyan Waiting ... $reset"
	sleep 10
	echo -e "$cyan ... Done waiting $reset"
	exit $code
} #end userExit

usage() {
	echo -e "$0 -w|--world file \n -t|--type final|prelim \n -r|-rovers 1-8 \n -o|--output file \n -d|--duration time_mins \n -v|--visualize \n -s|--seed #"
	#./flags.sh -w simulation/worlds/powerlaw_targets_example.world -t prelim -r 1 -o output.txt -d 2 -s 100 -v
	exit 1
}

startGazeboServer() {
	local world_file_path=$1
	local random_seed=$2
	rosparam set /use_sim_time true
	rosrun gazebo_ros gzserver $world_file_path --seed $random_seed --verbose &
	echo -e "$cyan Attempted to start Gazebo server with world file: $world_file_path and random seed $random_seed $reset"
} #end startGazeboServer

startGazeboClient() {
	rosrun gazebo_ros gzclient __name:=gzclient &
	echo -e "$cyan Attempted to start Gazebo client $reset"
} #end startGazeboClient

addCollectionZone() {
	$spawn\collection_disk/model.sdf -model collection_disk -x 0 -y 0 -z 0 -R 0 -P 0 -Y 0 
	echo -e "$cyan Attempted to add collection_zone: name=collection_disk, x=0, y=0, z=0, roll=0, pitch=0, yaw=0 $reset"
} #end addCollectionZone

addGroundPlane() {
	$spawn\concrete_ground_plane/model.sdf -model concrete_ground_plane -x 0 -y 0 -z 0 -R 0 -P 0 -Y 0 || echo -e "$red Concrete_ground_plane Spawn Failed $reset" 
	echo -e "$cyan Attempted to add concrete ground plane: name=concrete_ground_plane, x=0, y=0, z=0, roll=0, pitch=0, yaw=0 $reset"
} #end addGroundPlane

addCam() {
	$spawn\aerial_cam/model.sdf -model aerial_cam -x 0 -y 0 -z 0 -R 0 -P 0 -Y 0 || echo -e "$red aerial_cam Spawn Failed $reset" 
	echo -e "$cyan Attempted to add aerial cam: name=aerial_cam, x=0, y=0, z=0, roll=0, pitch=0, yaw=0 $reset"
} #end addGroundPlane

addWalls(){
	local roundType=$1 #defalt is final
	echo -e "$cyan Adding Wallz for $roundType round $reset"
	size=11.55 #final wall's distance from collection disk's cener
	[ $roundType ==  "prelim" ] && size=7.5 #if prelim round, final wall's distance from collection disk's cener
	$spawn\barrier_$roundType\_round/model.sdf -model "Barrier_North" -x $size -y 0 -z 0 -R 0 -P 0 -Y 0
	$spawn\barrier_$roundType\_round/model.sdf -model "Barrier_West" -x 0 -y $size -z 0 -R 0 -P 0 -Y 1.570796
	$spawn\barrier_$roundType\_round/model.sdf -model "Barrier_East" -x 0 -y -$size -z 0 -R 0 -P 0 -Y 1.570796
	$spawn\barrier_$roundType\_round/model.sdf -model "Barrier_South" -x -$size -y 0 -z 0 -R 0 -P 0 -Y 0
	echo -e "$green Done Adding Wallz for $roundType round $reset"
} #end addWalls

addRover() {
	local rover_name=$1
	local x=$2
	local y=$3
	local z=$4
	local roll=$5
	local pitch=$6
	local yaw=$7
	$spawn$rover_name/model.sdf -model $rover_name -x $x -y $y -z $z -R $roll -P $pitch -Y $yaw
	echo -e "$cyan Attempted to add rover: name=$rover_name, x=$x, y=$y, z=$z, roll=$roll, pitch=$pitch, yaw=$yaw $reset"
} #end addRover

addRovers(){
	# Add the rovers to the simulation
	#  The distances that determine the X, Y coords of the rovers is determined as follows:
	#  The distance to the rover from a corner position is calculated differently
	#  than the distance to a cardinal position.
	# 
	#  The cardinal direction rovers are a straightforward calculation where:
	#    a = the distance to the edge of the collection zone
	#      i.e., 1/2 of the collection zone square side length
	#    b = the 50cm distance required by the rules for placing the rover
	#    c = offset for the simulation for the center of the rover (30cm)
	#      i.e., the rover position is at the center of its body
	#
	#  The corner rovers use trigonometry to calculate the distance where each
	#  value of d, e, and f, are the legs to an isosceles right triangle. In
	#  other words, we are calculating and summing X and Y offsets to position
	#  the rover.
	#	  d = a
	#	  e = xy offset to move the rover 50cm from the corner of the collection zone
	#	  f = xy offset to move the rover 30cm to account for its position being
	#		  calculated at the center of its body
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
	local NUM_ROVERS=$1
	# Delays between adding rovers and starting their nodes in seconds
	MODEL_ADD_INTERVAL=3s
	# Specify rover start coordinates
	ROVER_POSITIONS_X=( -1.308 0.000 1.308 0.000 1.072 -1.072 -1.072 1.072 )
	ROVER_POSITIONS_Y=( 0.000 -1.308 0.000 1.308 1.072 -1.072 1.072 -1.072 )
	# In this case, the yaw is the value that turns rover "left" and "right" */
	ROVER_YAWS=( 0.000 1.571 -3.142 -1.571 -2.356 0.785 -0.785 2.356 )
	echo -e " $cyan Adding rovers to Gazebo and starting their ROS nodes... $reset"
	# Add rovers to the simulation and start the associated ROS nodes
	for (( i=0;i<$NUM_ROVERS;i++ ));
	do
		sleep $MODEL_ADD_INTERVAL
		addRover ${ROVER_NAMES[i]} ${ROVER_POSITIONS_X[i]} ${ROVER_POSITIONS_Y[i]} 0 0 0 ${ROVER_YAWS[i]}
		sleep $MODEL_ADD_INTERVAL
		startRoverNodes ${ROVER_NAMES[i]}
	done

	echo -e "$green Finished adding rovers. $reset"
} #end addRovers

# Stops the ROS nodes associated with rovers
startRoverNodes() {
	local rover_name=$1
	setsid roslaunch $(catkin locate)/launch/swarmie.launch name:=$rover_name >./logs/$rover_name.log &
	echo "Attempted to start rover ROS nodes"
} #end startRoverNodes

publishRoverModes(){
	local NUM_ROVERS=$1
	local mode=$2
	modenum=1 #manual mode
	[ $mode ==  "autonomous" ] && modenum=2
	echo "Setting rovers to $mode mode..."
	# Send the autonomous command to all rovers
	for (( i=0;i<$NUM_ROVERS;i++ ));
	do
		# Publish the autonomous mode command ("2") to each rover. 
		rostopic pub /${ROVER_NAMES[i]}/mode std_msgs/UInt8 2 &
		echo "Publishing 2 on /${ROVER_NAMES[i]}/mode"
		sleep 2
	done
	echo "Finished setting rovers to $mode mode."
} #end publishRoverModes

#Colors
reset="\e[0m"
red="\e[1;31m"
cyan="\e[1;36m"
green="\e[1;32m"

#---------------------------------------------------------#
#  Main
#
#---------------------------------------------------------#

export dir=$(catkin locate)
# Exit script if user enters ctl-c or sends interrupt
trap userExit SIGINT SIGTERM 

#defalts
WORLD_FILE_PATH=`find ./simulation/worlds -type f -name "*.world" | head -n1` #find the first world
type="prelim"
NUM_ROVERS=1
SCORE_OUTPUT_PATH=/dev/null
duration=3
visualize="/bin/false"
RANDOM_SEED=100
#parse arguments
while test $# -gt 0; do
	case "$1" in
		-h|--help|/h|/help)
			shift
			usage
			;;
		-w|--world)
			shift
			echo "world args $#"
			#if there is an argument and its a file
			if [ $# -gt 0  -a -f "$1" ]; then
				WORLD_FILE_PATH=$1
				echo "Setting world to: $WORLD_FILE_PATH"
				shift
			else
				echo "Missing/Invalid world try:"
				echo "`find ./simulation/worlds -type f -name "*.world" | head -n5`" #give the first 5 worlds as a sugestion
				exit 3
			fi
			;;
		-t|--type) #final|prelim
			shift
			#if there is an argument and its either prelim or final
			if [ $# -gt 0 -a "$1" = "prelim" -o "$1" = "final" ]; then
				type=$1
				echo "Setting round type to: $type"
				shift
			else
				echo "Missing/Invaild round type enter either prelim or final"
				exit 3
			fi
			;;
		-r|-rovers)
			shift
			#if has agument and is #1-8
			if [ $# -gt 0 -a "$1" -gt 0 -a "$1" -lt 9 ]; then
				NUM_ROVERS=$1
				echo "Setting numbers of rovers to: $NUM_ROVERS"
				shift
			else
				echo "Missing/Invaild number of rovers enter numbers between 1-8"
				exit 3
			fi
			
			;;
		-o|--output)
			shift
			#have permission to touch the file
			if [ $# -gt 0 ] && touch "$1"; then
				SCORE_OUTPUT_PATH=$1
				echo "Setting output to: $SCORE_OUTPUT_PATH"
				shift
			else
				echo "Missing/Invaild output file make sure you have permissions"
				exit 3
			fi
			;;
		-d|--duration)
			shift #make sure its an int 1>=
			if [ $# -gt 0 -a "$1" -gt 0 ]; then
			duration=$1
			echo "Setting duration to: $duration"
			else
				echo "Missing/Invaild duration please enter whole numbers"
				exit 3
			fi 
			#&& echo "duration $1" || echo "missing duration"
			shift
			;;
		-v|--visualize) 
			shift 
			echo "Visualizion enabled"
			visualize="/bin/true" #set boolean
			;;
		-s|--seed)
			shift #make sure its an int
			if [ $# -gt 0 -a "$1" -gt -1 ]; then 
				RANDOM_SEED=$1
				echo "Setting seed to: $seed"
				shift
			else
				echo "Missing/Invaild seed"
				exit 3
			fi
			;;
		*)
			shift
			echo "Not a valid argument"
			usage 
			;;
	esac
done

echo "Running in $dir" 
previous_gazebo_model_path=${GAZEBO_MODEL_PATH}
previous_gazebo_plugin_path=${GAZEBO_PLUGIN_PATH}
export GAZEBO_MODEL_PATH="$dir/simulation/models"
export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:./build/lib/:$dir/build/gazebo_plugins
spawn="rosrun gazebo_ros spawn_model -sdf -file $dir/simulation/models/"
ROVER_NAMES=( "achilles" "aeneas" "ajax" "diomedes" "hector" "paris" "thor" "zeus" )
SLEEP_INTERVAL=.4 # Set the interval at which to check whether the experiment duration has elapsed
source "$dir/devel/setup.bash"
echo Cleaning up ROS and Gazebo Processes
chmod +x $dir/cleanup.sh
$dir/cleanup.sh
echo Killing rosmaster
pkill rosmaster
echo Killing roscore
pkill roscore
sleep 2
roscore &
sleep 2

echo "Experiment started at $(date +%d-%m-%Y" "%H:%M:%S)."
#---------------------------------------------------------#
# Start the gazebo simulation
startGazeboServer $WORLD_FILE_PATH $RANDOM_SEED
sleep 5
$visualize && startGazeboClient # Start the gazebo simulation if got agrument
echo -e "$cyan Adding models to the world $reset"
addGroundPlane
#addCam
addRovers $NUM_ROVERS
addCollectionZone
addWalls $type # will be either prelim or final
sleep 10
echo -e "$green Done adding models to the world $reset"
#---------------------------------------------------------#
publishRoverModes $NUM_ROVERS "autonomous"

#---------------------------   Checking for errors    ------------------------------#
# Testing for  [Err] [Model.cc:921] Sensors failed to initialize when loading model[collection_disk] via the factory mechanism.Plugins for the model will not be loaded."
echo "Looking for /collectionZone/score"
[[ $(rostopic list /collectionZone/score) ]] || userExit 9
n=10 #10-18
for (( i=0;i<$NUM_ROVERS;i++ )); do
	echo "Looking for /${ROVER_NAMES[i]}/swarmie_status"
	[[ $(rostopic list /${ROVER_NAMES[i]}/swarmie_status) ]] || userExit $(( $n + $i ))
	echo "Looking for /${ROVER_NAMES[i]}/targets/image/compressed"
	[[ $(rostopic list /${ROVER_NAMES[i]}/targets/image/compressed) ]] || userExit $(( $n + $i ))
done #end foreach checking rovers status exists
#---------------------------   Done checking for errors    ------------------------------#

# Read the current sim time (in seconds) from the ros topic /clock
rosstart=`rostopic echo -n 1 /clock/clock/secs | head -n 1`; start=$(date +%s)
score="Error"
echo "Time, Score" | tee -a $SCORE_OUTPUT_PATH
# Let the simulation run until the experiment duration is reached arg 4 is the time in mins
until (( $(( $rosnow-$rosstart )) >= $(( $duration*60 )) )); do
	score=`rostopic echo -n 1 /collectionZone/score/data | head -n1 | tr -d '"'`  # update the score
	rosnow=`rostopic echo -n 1 /clock/clock/secs | head -n 1` # get the current ros time # lookinto the --filter 
	echo "$(( $rosnow-$rosstart )) : $score" | tee -a $SCORE_OUTPUT_PATH  # print out something that looks like 492 : 3 time_in_secs : score
	sleep $SLEEP_INTERVAL
done
now=$(date +%s) #system time in seconds since epoch
echo "Sim Complete, Ending autonomous mode for all rovers."
publishRoverModes $NUM_ROVERS "manual" # Send the manual command to all rovers

echo "Sys Time:" $((now - start)) ", Sim Time:"  $((rosnow - rosstart)) | tee -a $SCORE_OUTPUT_PATH
echo "Score:$score" | tee -a $SCORE_OUTPUT_PATH
echo $score >/tmp/finalScore
userExit 0

