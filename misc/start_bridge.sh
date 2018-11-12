#!/bin/bash

robots=""

for robot in $@
do
    robots=$robots" -new-tab -url file://$PWD/rosbridge/robot_interface.html?robotname=$robot"
done

firefox $robots
