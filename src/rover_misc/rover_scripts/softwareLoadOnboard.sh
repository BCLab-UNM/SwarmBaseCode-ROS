#!/bin/bash
echo "Removing rover_onboard_workspace..."
rm -r rover_onboard_workspace
echo "Creating rover_onboard_workspace..."
mkdir rover_onboard_workspace
cd rover_onboard_workspace
echo "Changed directory to rover_onboard_workspace/"
mkdir src
echo "Changed directory to rover_onboard_workspace/src/"
cd src
echo "Initializing workspace..."
catkin_init_workspace
echo "Changed directory to rover_onboard_workspace/"
cd ..
echo "Starting the bare catkin_make..."
catkin_make
echo "Removing rover_onboard_workspace/src..."
rm -r src/
cd devel
echo "Changed directory to rover_onboard_workspace/devel/"
mkdir include
echo "Changed directory to ~/"
cd ~/
echo "Moving rover_driver_world_state"
mv ~/rover_driver_world_state ~/rover_onboard_workspace/devel/include/
echo "Moving src/"
mv ~/src ~/rover_onboard_workspace
"Copying libapriltag.a"
rm ~/rover_onboard_workspace/src/rover_onboard_target_detection/lib/libapriltag.a
cp ~/libapriltag.a ~/rover_onboard_workspace/src/rover_onboard_target_detection/lib/
cd ~/rover_onboard_workspace
catkin_make
