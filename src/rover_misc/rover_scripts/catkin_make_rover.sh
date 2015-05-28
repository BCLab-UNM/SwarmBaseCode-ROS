#!/bin/bash
logDir=$(date +%F--%T)

if [ -d ~/rover_onboard_workspace ];then
  mkdir ~/$logDir
  mv /home/$USER/rover_onboard_workspace/ ~/$logDir
fi

if [ -d ~/rover_driver_workspace ];then
  if [ ! -d ~/$logDir ];then
    mkdir /home/$USER/rover_driver_workspace/ ~/$logDir
  fi
  mv /home/$USER/rover_driver_workspace/ ~/$logDir
fi

if [ -d ~/$logDir ];then
  mv ~/$logDir ~/archive/
fi

echo "Starting catkin_make process"
echo "Unpacking tars"
if [ ! -f ~/rover_driver_workspace.tar.gz ] || [ ! -f ~/rover_onboard_workspace.tar.gz ];then
  echo "You do not have the necessary workspaces downloaded."
  echo "Exiting"
  exit 0
fi
echo "Unpacking rover_driver_workspace"
tar -xvf rover_driver_workspace.tar.gz
echo "Unpacking rover_onboard_workspace"
tar -xvf rover_onboard_workspace.tar.gz
if [ -f ~/rover_misc_workspace.tar.gz ];then
  tar -xvf ~/rover_misc_workspace.tar.gz
fi

if [ ! -d ~/rover_onboard_workspace/ ] || [ ! -d ~/rover_driver_workspace ];then
  echo "You do not have the necessary workspaces downloaded."
  echo "Download rover_onboard_workspace and rover_driver_workspace to your ~/ directory"
  echo "Exiting"
  exit 0
fi
cd ~/rover_onboard_workspace/
catkin_make
source ~/rover_onboard_workspace/devel/setup.bash
cd ~/rover_driver_workspace/
catkin_make
cd ~/rover_onboard_workspace/
catkin_make
source ~/rover_onboard_workspace/devel/setup.bash
chmod +x ~/rover_driver_workspace/src/rover_driver_gazebo_launch/src/rover_driver_gazebo_launch.sh
echo "END"
