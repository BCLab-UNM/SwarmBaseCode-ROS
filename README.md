# Swarmie-ROS

This repository is a ROS (Robot Operating System) controller framework for the Swarmie robots used in the [NASA Swarmathon](http://swarmathon.cs.unm.edu), a national swarm robotics competition. This particular framework is a ROS implementation of the CPFA (central-place foraging algorithm) developed for [iAnt](http://iant.cs.unm.edu) robot swarms at the University of New Mexico.

This repository contains:

1. Source code for ROS libraries (i.e. packages) that control different aspects of the Swarmie robot, including localization, mapping, mobility, and obstacle and target detection
2. 3D .STL models for the physical Swarmie build 
3. Bash shell scripts for initializing simulated Swarmies in the Gazebo simulator, as well as physical Swarmies

### Quick Start Installation Guide

Swarmie-ROS is designed and tested exclusively on Ubuntu 14.04 LTS (Trusty Tahr) and ROS Indigo Igloo. This framework may compile and run correctly under other versions of Ubuntu and ROS, but **NOTE** that these other systems are untested and are therefore not supported.

##### 1. Install ROS Indigo

Follow the detailed instructions for installing ROS Indigo under Ubuntu 14.04 [here](http://wiki.ros.org/indigo/Installation/Ubuntu). We recommend the Desktop-Full installation, which includes the Gazebo 2 simulator.

##### 2. Install additional Gazebo plugins

Our simulated Swarmies use existing Gazebo plugins, external to this repo, to replicate sonar, IMU, and GPS sensors. These plugins are contained in the hector_gazebo_plugins package, which should be installed using the apt-get package management tool:

```
sudo apt-get install ros-indigo-hector-gazebo-plugins
```

##### 3. Install Swarmie-ROS

1. Clone, or [download](https://github.com/BCLab-UNM/Swarmie-ROS/archive/master.zip) extract, this GitHub repository to your home directory (~/)
2. Change your current working directory to the root directory of the downloaded repo
  * If you cloned this repo using git:
    ```
    cd ~/Swarmie-ROS
    ```
    
  * If you downloaded and extracted this repo directly from GitHub
    ```
    cd ~/Swarmie-ROS-master
    ```

3. Compile Swarmie-ROS as a ROS catkin workspace:
  ```
  catkin_make
  ```
