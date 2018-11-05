#!/bin/bash

robots=""

for robot in $@
do
    robots=$robots" -new-tab -url file://$PWD/rosbridge/robot_interface.html?r=$robot"
done

firefox $robots
