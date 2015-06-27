# Swarmie-ROS

This repository is a ROS (Robot Operating System) controller framework for the Swarmie robots used in the [NASA Swarmathon](http://swarmathon.cs.unm.edu), a national swarm robotics competition. This particular framework is a ROS implementation of the CPFA (central-place foraging algorithm) developed for [iAnt robot swarms](http://iant.cs.unm.edu) at the [University of New Mexico](http://www.unm.edu/).

This repository contains:

1. Source code for ROS libraries (i.e. packages) that control different aspects of the Swarmie robot, including localization, mapping, mobility, and obstacle and target detection
2. 3D .STL models for the physical Swarmie build 
3. Bash shell scripts for initializing simulated Swarmies in the Gazebo simulator, as well as physical Swarmies

### Quick Start Installation Guide

Swarmie-ROS is designed and tested exclusively on Ubuntu 14.04 LTS (Trusty Tahr) and ROS Indigo Igloo. This framework may compile and run correctly under other versions of Ubuntu and ROS, but **NOTE** that these other systems are untested and are therefore not supported at this time.

##### 1. Install ROS Indigo

Follow the detailed instructions for installing ROS Indigo under Ubuntu 14.04 [here](http://wiki.ros.org/indigo/Installation/Ubuntu). We recommend the Desktop-Full installation, which includes the Gazebo 2 simulator.

##### 2. Install additional Gazebo plugins

Our simulated Swarmies use existing Gazebo plugins, external to this repo, to replicate sonar, IMU, and GPS sensors. These plugins are contained in the [hector_gazebo_plugins](http://wiki.ros.org/hector_gazebo_plugins) package, which should be installed using the apt-get package management tool:

```
sudo apt-get install ros-indigo-hector-gazebo-plugins
```

##### 3. Install Swarmie-ROS

1. Clone, or [download](https://github.com/BCLab-UNM/Swarmie-ROS/archive/master.zip) and extract, this GitHub repository to your home directory (~/)

2. Rename the downloaded repo so it can be properly identified by ROS and catkin
  * If you cloned this repo using git:
    ```
    mv ~/Swarmie-ROS ~/rover_workspace
    ```
    
  * If you downloaded and extracted this repo directly from GitHub
    ```
    mv ~/Swarmie-ROS-master ~/rover_workspace
    ```

3. Change your current working directory to the root directory of the downloaded repo:

  ```
  cd ~/rover_workspace
  ```

4. Compile Swarmie-ROS as a ROS catkin workspace:

  ```
  catkin_make
  ```

5. Update your bash session by automatically exporting the enviromental variable that stores the location of Gazebo's model files:

  ```
  echo "export GAZEBO_MODEL_PATH=~/rover_workspace/src/rover_misc/gazebo/models" >> ~/.bashrc
  source ~/.bashrc
  ```
