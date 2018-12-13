#!/bin/bash
OPTSTRING=`getopt -l frame-rate: -- $0 $@`
frame_rate=10
eval set -- "$OPTSTRING"
while true
do
    case "$1" in
        --frame-rate ) frame_rate="$2"; shift 2 ;;
        -- ) shift ; break ;;
    esac
done
roscore &
sleep 10 # give roscore enough time to start
./rover_onboard_node_launch.sh --frame-rate $frame_rate localhost $1
