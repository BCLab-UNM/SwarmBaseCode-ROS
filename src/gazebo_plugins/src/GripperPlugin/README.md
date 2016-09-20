# GripperPlugin README

This plugin implements a gripper controller for the NASA Swarmathon Rovers. There are several required XML tags and numberous optional XML tags that can be used to customize the behavior of the gripper and to debug the plugin.

| Required XML Tag | Value  | Definition                                                          |
|-----------------:|:------:|:--------------------------------------------------------------------|
|       wristJoint | string | name of the gripper's wrist joint defined in the SDF file           |
|  leftFingerJoint | string | name of the gripper's left finger joint defined in the SDF file     |
| rightFingerJoint | string | name of the gripper's right finger joint defined in the SDF file    |
|       wristTopic | string | name of the subscription topic for a specific rover's wrist joint   |
|      fingerTopic | string | name of the subscription topic for a specific rover's finger joints |

| Optional XML Tags   | Value               | Definition                                                                         |
|--------------------:|:-------------------:|:-----------------------------------------------------------------------------------|
|               debug |                     | container tag for the "printToConsole" and "printDelayInSeconds" tags              |
|      printToConsole | bool                | "debug" sub-tag; true = debugging is on; false = debugging is off                  |
| printDelayInSeconds | float               | "debug" sub-tag; the number of seconds to delay between debug print statements     |
|          updateRate | float               | the subscriber refresh rate for the gripper; the number of updates per second      |
|            wristPID | float, float, float | PID values for the wrist joint: Kp, Ki, Kd                                         |
|           fingerPID | float, float, float | PID values for both of the finger joints: Kp, Ki, Kd                               |
|    wristForceLimits | float, float        | min and max amounts of force (in Newtons) that can be applied to the wrist joint   |
|   fingerForceLimits | float, float        | min and max amounts of force (in Newtons) that can be applied to the finger joints |

The following code example demonstrates how to use the plugin in a Rover's SDF configuration file:

```xml
		<!-- Gripper plugin-->
		<plugin name="gripper_sim" filename="libgazebo_plugins_gripper.so">
			<!-- required: joint definitions -->
			<wristJoint>gripper_wrist_joint</wristJoint>
			<leftFingerJoint>gripper_left_finger_joint</leftFingerJoint>
			<rightFingerJoint>gripper_right_finger_joint</rightFingerJoint>

			<!-- required: data input topics for this plugin -->
			<wristTopic>/achilles/wristAngle</wristTopic>
			<fingerTopic>/achilles/fingerAngle</fingerTopic>

			<!-- optional: print debug info to the console -->
			<debug>
				<!-- true (debug mode is on) or false (debug mode is off) -->
				<printToConsole>false</printToConsole>
				<!-- time (in simulated seconds) between debug print statements -->
				<printDelayInSeconds>5.0</printDelayInSeconds>
			</debug>

			<!-- optional: the refresh rate of the gripper and all debugging output -->
			<!-- this value is the number of updates performed per second -->
			<!-- warning: low values (i.e. < 100) can cause the gripper to have poor performance -->
			<updateRate>1000.0</updateRate>

			<!-- optional: set the default PID values for the wrist joint -->
			<!-- values in order: Proportional gain, Integral gain, Derivative gain -->
			<wristPID>2.5 0.0 0.0</wristPID>

			<!-- optional: set the default PID values for the finger joints -->
			<!-- values in order: Proportional gain, Integral gain, Derivative gain -->
			<!-- it is always assumed that both fingers use the same PID values -->
			<fingerPID>2.5 0.0 0.0</fingerPID>

			<!-- optional: set the min/max force applicable to the wrist joint in Newtons -->
			<!-- values in order: minimum force, maximum force -->
			<wristForceLimits>-10 10</wristForceLimits>

			<!-- optional: set the min/max force applicable to the finger joints in Newtons -->
			<!-- values in order: minimum force, maximum force -->
			<!-- it is always assumed that both fingers use the same min/max force values -->
			<fingerForceLimits>-10 10</fingerForceLimits>
		</plugin>
```
