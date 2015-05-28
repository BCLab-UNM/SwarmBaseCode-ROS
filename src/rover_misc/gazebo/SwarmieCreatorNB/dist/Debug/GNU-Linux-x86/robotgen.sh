#!/bin/bash
echo "How many robots? "
read numRobots
for ((i = 0; i < "$numRobots"; i++)); do
echo "Enter Robot Name:"; read robotName
#TODO ADD FORCE ROBOT NAME
echo "Enter Robot Color:"; read robotColor
#TODO ADD FORCE ROBOT COLOR
'./robotcreator' "$robotName" "$robotColor"
echo "Done"
echo
echo "Generated a" "$robotColor" "$robotName" "in newRobots/"
echo
done


