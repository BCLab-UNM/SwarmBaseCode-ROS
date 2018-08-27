#!/bin/bash

# Handle user early termination through SIGINT (or ctrl-c)
function userExit()
{
    echo "Received ctrl-c or interrupt. Exiting."
    ./cleanup.sh
    # Kill processes in our process group (our children)
    kill -- -$$

}


checkIfNodesAreRunning()
{
    # Check that the expected nodes are running
    echo "check nodes are running"
    #/achilles_APRILTAG
    #/achilles_BASE2CAM
    checkIfNodeIsRunning /achilles_BEHAVIOUR
    #/achilles_MAP
    #/achilles_NAVSAT
    #/achilles_ODOM
    #/gazebo
}

checkIfNodeIsRunning()
{
    local node_name=$1
    if [[ $(rosnode ping $node_name) = *"unknown node"* ]]; then
	echo "Check failed: $node_name is not running."
	exit 1
    fi
}

# Trap SIGINT and call userExit 
trap userExit SIGINT


# Run the headless simulation see sun_headless_sim.sh usage output for parameter definitions
setsid ./run_headless_sim.sh simulation/worlds/powerlaw_targets_example.world 3 /dev/null 2 42 &

sleep 30

checkIfNodesAreRunning


