# SwarmBaseCode-ROS

This repository is a ROS (Robot Operating System) controller framework for the Swarmie robots used in the [NASA Swarmathon](http://www.nasaswarmathon.com), a national swarm robotics competition. 

This repository contains:

1. Source code for ROS libraries (i.e. packages) that control different aspects of the Swarmie robot, including robot behaviours such as search strategies and obstacle avoidance, diagnostics, and user interface elements. 
2. 3D .STL models for the physical Swarmie build 
3. Bash shell scripts for initializing simulated Swarmies in the Gazebo simulator, as well as physical Swarmies

- For a step-by-step guide to using Swarmathon-ROS to control a physical Swarmie robot, please see the instructions in the physical robot [Quick Start Guide](https://github.com/BCLab-UNM/Swarmathon-Docs/blob/master/Quick%20Start%20Physical%20Guide.md).

- Please submit bug reports for Swarmathon-ROS through GitHub's Issues system. For all other questions regarding the Swarmathon-ROS code base, please visit the forums on the [NASA Swarmathon website](http://www.nasaswarmathon.com).

- We recommend Jason M. O'Kane's [A Gentle Introduction to ROS](https://cse.sc.edu/~jokane/agitr/) for an in-depth walkthrough.

- Please consult the [git scm](https://git-scm.com/) and [git best practices](https://sethrobertson.github.io/GitBestPractices/) for guidelines on the most effective approaches to maintaining code. Teams will be expected to commit new code at least every two weeks, and ideally commit one or more times per week. Consult the [NASA Swarmathon Timeline](http://www.nasaswarmathon.com) for specifics on how often code should be committed, as well as the cutoff date for final code revision before the competition.

Be: sure you are using the latest drivers for your video card using the "additional drivers tool" in Ubuntu. Gazebo client often does not do well with the open source drivers.

![Alt text](https://github.com/BCLab-UNM/SwarmBaseCode-ROS/blob/master/readmeImages/InstallGraphics.png "Additional Drivers")

### Quick Start Installation Guide

Swarmathon-ROS is designed and tested exclusively on the 64 bit version of Ubuntu 16.04 LTS (Xenial Xerus) and ROS Kinetic Kame. Other systems are untested and are therefore not supported at this time.

##### 1. Install ROS Kinetic Kame

Detailed instructions for installing ROS Kinetic Kame under Ubuntu 16.04 [here](http://wiki.ros.org/kinetic/Installation/Ubuntu) or follow the summarized instructions below:

```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116

sudo apt update
sudo apt install ros-kinetic-desktop-full
sudo rosdep init
rosdep update      # Note this is not run with sudo
```

Note: if you accidentally ran ```sudo rosdep update``` you can repair the permissions ```sudo rosdep fix-permissions```.

You may request the installation of addition packages on the competition rovers. To do so create a pull request modifying the [preinstalled packages list](https://github.com/BCLab-UNM/Swarmathon-Docs/blob/master/PreinstalledCompetitionPackages.md) document.

##### 2. Install additional ROS packages

We use the [catkin_tools](https://catkin-tools.readthedocs.io/) package to build the Swarmathon-ROS code base:

```
sudo apt install python-rosinstall python-catkin-tools
```

Our simulated and physical Swarmies use existing ROS plugins, external to this repo, to facilitate non-linear state estimation through sensor fusion and frame transforms. These plugins are contained in the [robot_localization](http://wiki.ros.org/robot_localization) package, which should be installed using the apt-get package management tool:

```
sudo apt install ros-kinetic-robot-localization
```

##### 3. Install additional Gazebo plugins

Our simulated Swarmies use existing Gazebo plugins, external to this repo, to replicate sonar, IMU, and GPS sensors. These plugins are contained in the [hector_gazebo_plugins](http://wiki.ros.org/hector_gazebo_plugins) package, which should be installed using the apt-get package management tool:

```
sudo apt install ros-kinetic-hector-gazebo-plugins
```

Thw Swarmies can receive commands from the thumb sticks on a Microsoft Xbox 360 controller. The ROS [joystick_drivers](http://wiki.ros.org/joystick_drivers) package, which contains a generic Linux joystick driver compatible with this controller, should also be installed using the apt tool:

```
 sudo apt install ros-kinetic-joystick-drivers
```

Joystick commands can also be simulated using the direction keys (Up=I, Down=K, Left=J, Right=L) on the keyboard. The Rover GUI window must have focus for keyboard control to work.

##### 4. Install git (if git is already installed, skip to step 5):

```
sudo apt install git
```

##### 5. Install Swarmathon-ROS

1. Clone this GitHub repository to your home directory (~), renaming the repo so ROS and catkin can properly identify it (you can name the target directory whatever you like):

  ```
  cd ~
  git clone https://github.com/BCLab-UNM/SwarmBaseCode-ROS.git SwarmBaseCode-ROS
  ```

2. Change your current working directory to the root directory of the downloaded repo.


3. Set up [ublox](http://wiki.ros.org/ublox) GPS submodule and April Tag library:

  ```
  git submodule init
  git submodule update
  ```

4. Compile Swarmathon-ROS as a ROS catkin workspace:
 
  Make sure bash is aware of the location of the ROS environment:
  ```
  if ! grep -q "source /opt/ros/kinetic/setup.bash" ~/.bashrc
  then 
    echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
  fi
  source ~/.bashrc
  ```
  
  Tell catkin to build the Swarmathon code:
  
  ```
  catkin build
  ```
    
##### 6. Run the Swarmathon-ROS simulation:

1. Change the permissions on the simulation run script to make it exectuatable (assuming you use the target directory name SwarmBaseCode-ROS):
  
  ```
  cd ~/SwarmBaseCode-ROS
  chmod +x ./run.sh
  ```
  
2. Start the simulation

  ```
  ./run.sh
  ```

The GUI will now launch. The run script kills a number of gazebo and ROS processes. Killing these processes is suggested by gazebosim.com as the best way to clean up the gazebo environment at the moment.

This is the first screen of the GUI:

![Alt text](https://github.com/BCLab-UNM/Swarmathon-ROS/blob/master/readmeImages/guiFirstScreen.png "Opening Screen")

Click the "Simulation Control" tab:

![Alt text](https://github.com/BCLab-UNM/Swarmathon-ROS/blob/master/readmeImages/simControlTab.png "Simulation Parameters")

Choose the ground texture, whether this is a preliminary or final round (3 or 6 robots), and the distribution of targets.

Click the "Build Simulation" button when ready.

The gazebo physics simulator will open.

![Alt text](https://github.com/BCLab-UNM/Swarmathon-ROS/blob/master/readmeImages/buildSim.png "Gazebo Simulator")

Click back to the Swarmathon GUI and select the "Sensor Display" tab.

![Alt text](https://github.com/BCLab-UNM/Swarmathon-ROS/blob/master/readmeImages/sensorDisplayTab.png "Sesnor display")

Any active rovers, simulated or real will be displayed in the rover list on the left side.

![Alt text](https://github.com/BCLab-UNM/Swarmathon-ROS/blob/master/readmeImages/activeRovers.png "Active rovers")

Select a rover to view its sensor outputs. 

![Alt text](https://github.com/BCLab-UNM/Swarmathon-ROS/blob/master/readmeImages/roverSensorOutputs.png "Rover sensor outputs")

There are four sensor display frames:

The camera output. This is a rover eye's view of the world.

The ultrasound output is shown as three white rays, one for each ultrasound. The length of the rays indicates the distance to any objects in front of the ultrasound. The distance in meters is displayed in text below these rays. The maximum distance reported by the ultrasounds is 3 meters.  

The IMU sensor display consists of a cube where the red face is the bottom of the rover, the blue face is the top of the rover, and the red and blue bars are the front and back of the rover. The cube is viewed from the top down. The cube is positioned according to the IMU orientation data. For example, if the rover flips over, the red side will be closest to the observer. Accelerometer data is shown as a 3D vector projected into 2D space pointing towards the sum of the accelerations in 3D space.
 
The map view shows the path taken by the currently selected rover. Green is the encoder position data. In simulation, the encoder position data comes from the odometry topic being published by Gazebo's skid steer controller plugin. In the real robots, it is the encoder output. GPS points are shown as red dots. The EKF is the output of an extended Kalman filter which fuses data from the IMU, GPS, and encoder sensors.

Click on the "Task Status" tab.

![Alt text](https://github.com/BCLab-UNM/Swarmathon-ROS/blob/master/readmeImages/taskStatusTab.png "Task Status tab")

This tab displays the number of targets detected, the number of targets collected, and the number of obstacle avoidance calls.

To close the simulation and the GUI, click the red exit button in the top left-hand corner.

### Software Documentation

Source code for Swarmathon-ROS can be found in the repository /src directory. This diretory contains severals subdirectories, each of which contain a single ROS package. Here we present a high-level description of each package.

- ```abridge```: A serial interface between Swarmathon-ROS and the A-Star 32U4 microcontroller onboard the physical robot. In the Swarmathon-ROS simulation, ```abridge``` functionality is supplanted by [gazebo_ros_skid_steer_drive](http://docs.ros.org/kinetic/api/gazebo_plugins/html/classgazebo_1_1GazeboRosSkidSteerDrive.html) (motor and encoders) and [hector_gazebo_plugins](http://wiki.ros.org/hector_gazebo_plugins) (sonar and IMU; see [step 3](https://github.com/BCLab-UNM/Swarmathon-ROS/blob/master/README.md#3-install-additional-gazebo-plugins) of the Quick Start guide).
- ```behavours```: The top-level controller class for the physical and simulated robots. This package receives messages from sensors and implements behaviours the robot should follow in response. This packages also receives pose updates from [robot_localization](http://wiki.ros.org/robot_localization) (see [step 2](https://github.com/BCLab-UNM/Swarmathon-ROS/blob/master/README.md#2-install-additional-ros-plugins) of the Quick Start guide) and commands from [joystick_drivers](http://wiki.ros.org/joystick_drivers) (see [step 3](https://github.com/BCLab-UNM/Swarmathon-ROS/blob/master/README.md#3-install-additional-gazebo-plugins) of the Quick Start guide).
- ```rqt_rover_gui```: A Qt-based graphical interface for the physical and simulated robots. See [How to use Qt Creator](https://github.com/BCLab-UNM/Swarmathon-ROS/blob/master/README.md#how-to-use-qt-creator-to-edit-the-simulation-gui) for details on this package.
- ```target_detection```: An image processor that detects [AprilTag](https://april.eecs.umich.edu/wiki/index.php/AprilTags) fiducial markers in the onboard camera's video stream. This package receives images from the ```usbCamera``` class (for physical robots) or [gazebo_ros_camera](http://docs.ros.org/kinetic/api/gazebo_plugins/html/classgazebo_1_1GazeboRosCamera.html) (for simulated robots), and, if an AprilTag is detected in the image, returns the integer value encoded in the tag.
- ```ublox```: A serial interface to the ublox GPS receiver onboard the physical robot. This package is installed as a git submodule in the Swarmathon-ROS repo. See the [ublox ROS wiki page](http://wiki.ros.org/ublox) for more information.

### How to use Qt Creator to edit the simulation GUI

1. Install Qt Creator:

  ```
  sudo apt-get install qtcreator
  ```
  
2. Build the workspace:

  ```
  catkin build
  ```

3. Run Qt Creator:
  ```
  qtcreator &
  ```

4. Choose "Open File or Project" from the File menu

5. Navigate to ```~/rover_workspace/src/rqt_rover_gui/```

6. Select CMakeLists.txt

7. Click "Open" to continue.

8. Enter ```path to your home directory /rover_workspace/build``` in the text box, this is the default build path. You cannot use the ~ as a shorthand to your home directory here.

9. Click Configure Project

10. Click on the Projects icon on the left toolbar

11. Enter ```-DCMAKE_INSTALL_PREFIX=../install -DCATKIN_DEVEL_PREFIX=../devel``` in the CMake arguments text box

12. Click the "Edit" toolbox icon on the left

13. Double-click CMakeLists.txt

14. Click the "Build Now" button to build the project

Qt Creator can now be used to build the rover_workspace

Note: start qtcreator in your terminal with rover_workspace as the current directory. Source the ~/.bashrc if the catkin environment variables are not set so that QT Creator can properly build the project.

### Debugging with GDB and Qt Creator

Debuggers are particularly useful for tracking down segfaults and for tracing through the logic of your programs. In order to use the GNU debugger (GDB) with the swarmathon competition add the following line to the CMakelists.txt file for the project you want to debug.

```set(CMAKE_BUILD_TYPE Debug)```

This will compile your code with debugging symbols enabled.

Since ROS is multithreaded you may need to attach the debugger to threads that have been spawned by your program. To enable this enter the following in a terminal:

```sudo apt-get install libcap2-bin```

```sudo setcap cap_sys_ptrace=eip /usr/bin/gdb```

To use QT Creator to debug your already running program click the "Debug" menu. Choose "Start Debugging" and then "Attach to Running Application...". You will be able to use the graphical interface to GDB from here. 
