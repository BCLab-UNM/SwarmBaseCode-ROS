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

##### 3. Install git (if git is already installed, skip to step 4):

```
sudo apt-get install git
```

##### 4. Install Swarmie-ROS

2. Clone this GitHub repository to your home directory (~/):

  ```
  cd ~/
  git clone git@github.com:BCLab-UNM/Swarmie-ROS.git
  ```

3. Rename the downloaded repo so it can be properly identified by ROS and catkin:

  ```
  mv ~/Swarmie-ROS ~/rover_workspace
  ```

4. Change your current working directory to the root directory of the downloaded repo:

  ```
  cd ~/rover_workspace
  ```

5. Set up GPS submodule:

  ```
  git submodule init
  git submodule update
  ```

6. Compile Swarmie-ROS as a ROS catkin workspace:

  ```
  catkin_make
  ```
  
7. Update your bash session to automatically source the setup file for Swarmie-ROS:

  ```
  echo "~/rover_workspace/devel/setup.bash" >> ~/.bashrc
  source ~/.bashrc
  ```

8. Update your bash session to automatically export the enviromental variable that stores the location of Gazebo's model files:

  ```
  echo "export GAZEBO_MODEL_PATH=~/rover_workspace/src/rover_misc/gazebo/models" >> ~/.bashrc
  source ~/.bashrc
  ```
