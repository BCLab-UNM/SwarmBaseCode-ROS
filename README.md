# Swarmie-ROS

A ROS (Robot Operating System) controller framework for the Swarmie robots used in the [NASA Swarmathon](http://swarmathon.cs.unm.edu), a national swarm robotics competition. This particular framework is a ROS implementation of the CPFA (central-place foraging algorithm) developed for [iAnt](http://iant.cs.unm.edu) robot swarms at the University of New Mexico.

This repository contains:

1. Source code for ROS libraries (i.e. packages) that control different aspects of the Swarmie robot, including localization, mapping, mobility, and obstacle and target detection
2. 3D .STL models for the physical Swarmie build 
3. Bash shell scripts for initializing simulated Swarmies in the Gazebo simulator, as well as physical Swarmies

### Quick Start Installation Guide

Swarmie-ROS is designed and tested exclusively on Ubuntu 14.04 LTS (Trusty Tahr) and ROS Indigo Igloo. This framework may compile and run correctly under other versions of Ubuntu and ROS, but **NOTE** that these other systems are untested and are therefore not supported.

##### 1. Install ROS Indigo

Follow the detailed instructions for installing ROS Indigo under Ubuntu 14.04 [here](http://wiki.ros.org/indigo/Installation/Ubuntu). We recommend the Desktop-Full installation, which includes the Gazebo 2 simulator.

##### 2. Modify your shell's startup file

Open your Linux shell's startup file in the text editor of your choice. For the vast majority of users, this means editing the bash startup script ~/.bashrc. Add these lines to the end of your .bashrc file:

```
#Swarmie setup                                                                                             
if [ -f ~/rover_workspace/devel/setup.bash ];
then
  source ~/rover_workspace/devel/setup.bash
fi
export GAZEBO_MODEL_PATH=~/rover_workspace/src/rover_misc/gazebo/models
```

After saving the startup file, make sure to source the file so your current shell is up-to-date:

```
source ~/.bashrc
```

##### 3. Install additional required ROS libraries



```
sudo apt-get install aptitude
```

```
sudo apt-get install ros-indigo-hector-gazebo
```
