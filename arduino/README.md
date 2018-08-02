Swarmathon-Arduino
==============

This repository is an Arduino microcontroller library for the Swarmie robots used in the [NASA Swarmathon](http://www.nasaswarmathon.com), a national swarm robotics competition created by the [University of New Mexico](http://www.unm.edu/). This particular library is an interface to the [Swarmathon-ROS](https://github.com/BCLab-UNM/Swarmathon-ROS) controller framework that facilitates the opertion of lower-level functionality onboard the physical robot, including brushed DC motors, integrated quadrature encoders, a 10-axis IMU (inertial measurement unit), and ultrasonic distance sensors.

For information on setting up the Arduino IDE and programming Arduino microcontrollers, please consult [Getting Started with Arduino](https://www.arduino.cc/en/Guide/HomePage). Aside from the IDE, no other plugins or tools are required to begin using this libra*ry.

**Please use version 1.8.5 of the IDE.**

If you are using Linux, you'll need to run several additional commands to ensure that the Arduino IDE has the correct permissions to run. First, open a **Terminal** window and run the command `sudo usermod -a -G dialout username` to add your user account to the `dialout` user group, where `username` should be replaced by your own user name. Then, run the command `echo 'ATTRS{idVendor}=="1ffb", ENV{ID_MM_DEVICE_IGNORE}="1"' | sudo tee /etc/udev/rules.d/77-arduino.rules` to force Ubuntu's modem-manager to ignore the Arduino when it is plugged in. Finally, reload the `udev` device manager with the command `sudo udevadm trigger`.

After installing the Arduino IDE, run the application and open the Arduino IDE Preferences window (under "File > Preferences" in Linux and Windows, or "Arduino > Preferences" in Mac OS X). Under the Settings tab, in the text box titled "Sketchbook location", enter the full path to your Swarmathon-Arduino directory, then click "OK":

![Arduino IDE Sketchbook location](https://github.com/BCLab-UNM/Swarmathon-Arduino/blob/master/readmeImages/ArduinoIDESketchbookLocation.png)

**Note** that you must exit and reopen the Arduino IDE before the change to the Sketchbook location is applied.

## Setup

1. Clone this GitHub repository to your home directory (~):

  ```
  cd ~
  git clone https://github.com/BCLab-UNM/Swarmathon-Arduino.git 
  ```

2. To set up the Arduino IDE to communicate with the Swarmie's [Pololu A-Star microcontroller](https://www.pololu.com/product/3104), which runs an Arduino-compatible bootloader, first set the board type under "Select > Board" to "Arduino Leonardo".

  ![Arduino IDE Board Type](https://github.com/BCLab-UNM/Swarmathon-Arduino/blob/master/readmeImages/ArduinoIDEBoardType.png)

3. Ensure that the A-Star is plugged into your PC (not the Swarmie's NUC), then select the proper serial port under "Select > Port". Your port number will most likely differ from the one shown in the screenshot below, but you should still see "Arduino Leonardo" next to the correct port.

  ![Arduino IDE Serial Port](https://github.com/BCLab-UNM/Swarmathon-Arduino/blob/master/readmeImages/ArduinoIDESerialPort.png)

4. If you haven't loaded it already, open the Swarmathon_Arduino.ino sketch under "File > Open" by navigating to your Swarmathon-Arduino directory.

  ![Arduino IDE Open Sketch](https://github.com/BCLab-UNM/Swarmathon-Arduino/blob/master/readmeImages/ArduinoIDEOpenSketch.png)
  ![Arduino IDE Open Sketch2](https://github.com/BCLab-UNM/Swarmathon-Arduino/blob/master/readmeImages/ArduinoIDEOpenSketch2.png)

5. Upload the sketch to the A-Star by clicking on the "Upload" button, a right arrow in the upper-left corner of the Arduino IDE.

  ![Arduino IDE Upload Sketch](https://github.com/BCLab-UNM/Swarmathon-Arduino/blob/master/readmeImages/ArduinoIDEUploadSketch.png)

6. If your upload is successful, you should see output in the black terminal box at the bottom of the Arduino IDE regarding the size of the sketch, as well as a "Done uploading" message above this black box. If the Arduino IDE outputs an error, please double-check that you have followed steps 1 through 4 above correctly.

  ![Arduino IDE Upload Success](https://github.com/BCLab-UNM/Swarmathon-Arduino/blob/master/readmeImages/ArduinoIDEUploadSuccess.png)
  

## Debugging

If you are encountering issues with your Swarmie, such as missing IMU, encoder, and/or ultrasound data, or problems with driving the robot, please consult the suggestions below after connecting the A-Star microcontroller to your PC and following steps 1 through 6 above.

1. Open the Arduino IDE and click the Serial Monitor button, a magnifying glass in the upper-right corner of the Arduino IDE.

  ![Arduino IDE Serial Monitor](https://github.com/BCLab-UNM/Swarmathon-Arduino/blob/master/readmeImages/ArduinoIDESerialMonitor.png)

  In the drop-down menus at the bottom of the Serial Monitor window that appears, ensure that the line ending option is set to "Newline", and that the baud rate is set to "115200 baud".

  ![Arduino IDE Line Ending and Baudrate](https://github.com/BCLab-UNM/Swarmathon-Arduino/blob/master/readmeImages/ArduinoIDELineEndingBaudRate.png)

2. Type `d` into the entry bar at the top of the Serial Monitor window and click the **Send** button.

  ![Arduino Debug Data Input](https://github.com/BCLab-UNM/Swarmathon-Arduino/blob/master/readmeImages/ArduinoDebugDataInput.png)

  You should receive a comma-delimited string of 18 floating-point values, similar to, but not identical to, the string shown here.
  
  ![Arduino Debug Data Output](https://github.com/BCLab-UNM/Swarmathon-Arduino/blob/master/readmeImages/ArduinoDebugDataOutput.png)
  
3. Type `f,1` into the entry bar and click **Send**. This command should open the gripper fingers to the angle shown.

  ![Arduino Debug Fingers Open](https://github.com/BCLab-UNM/Swarmathon-Arduino/blob/master/readmeImages/ArduinoDebugFingersOpen.png)
  
4. Type `w,1` into the entry bar and click **Send**. This command should lower the gripper wrist to the angle shown.

  ![Arduino Debug Wrist Down](https://github.com/BCLab-UNM/Swarmathon-Arduino/blob/master/readmeImages/ArduinoDebugWristDown.png)
  
5. High-center the robot on a box so that none of the wheels are touching the ground. Ensure that the motors are turned on by flipping the red switch on the back of the robot up.

  ![Arduino Debug High Center](https://github.com/BCLab-UNM/Swarmathon-Arduino/blob/master/readmeImages/ArduinoDebugHighCenter.png)
  
   Type `v,80,80` into the entry bar and click **Send**. The wheels should spin forward (rotating toward the front of the robot) for one second, then stop automatically.
  
  Type `v,-80,-80` into the entry bar and click **Send**. The wheels should spin backward (rotating toward the back of the robot) for one second, then stop automatically.
  
  Type `v,-80,80` into the entry bar and click **Send**. The wheels on the left side of the robot should spin backward, while the wheels on the right side of the robot should spin forward, for one second, then stop automatically.
  
  Finally, type `v,80,-80` into the entry bar and click **Send**. The wheels on the left side of the robot should spin forward, while the wheels on the right side of the robot should spin backward, for one second, then stop automatically.
