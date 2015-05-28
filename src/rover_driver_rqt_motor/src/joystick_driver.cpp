/* 
 * File:   	joystick_driver.cpp
 * Author: 	kurt leucht
 * Description:	handles raw joystick inputs at the driver station & publishes
 * 		ROS Joy type messages for onboard computer to act upon
 * 
 * Created:	June 10, 2014
 * Major Change Log:
 * 	6/11/2014 - added infinite retry loop in case joystick is not connected
 *	6/23/2014 - (mwn) implemented parameter server	
 *
 */

// TODO: read topic /driver/joystick_whom and publish on that topic if found, else publish on driver/joystick topic

// ROS headers
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
// C and Linux headers
#include <cstdlib>
#include <fcntl.h>
#include <linux/input.h>
#include <signal.h>
#include <stdio.h>
#include <unistd.h>

using namespace std;

#define MODULE_VERSION	"1.0"
#define STRING_LENGTH	255

const char compatible_joystick_labels[6][STRING_LENGTH] = {
    "usb-Logitech_Logitech_Cordless_RumblePad_2-event-joystick",
    "usb-Logitech_Gamepad_F310_B468A938-event-joystick",
    // TODO: handle joysticks generically, without the model number & serial number embedded
    "usb-Logitech_Gamepad_F310_BB21D241-event-joystick",
    "usb-Logitech_Wireless_Gamepad_F710_B4179B5D-event-joystick",
    "usb-Logitech_Wireless_Gamepad_F710_C1CE9F9D-event-joystick",
    "usb-Logitech_Wireless_Gamepad_F710_26C49679-event-joystick",
};

// TODO: move these globals to a shared header file or go OO with getters?
#define SUCCESS			0
#define ERROR			-1

#define JOY_TOTAL_AXES		6
#define JOY_TOTAL_BUTTONS	16

#define JOY_AXIS_LEFT_THUMB_X	0
#define JOY_AXIS_LEFT_THUMB_Y	1
#define JOY_AXIS_RIGHT_THUMB_X	2
#define JOY_AXIS_RIGHT_THUMB_Y	3
#define JOY_AXIS_LEFT_FINGER	4
#define JOY_AXIS_RIGHT_FINGER	5

#define JOY_BTN_LEFT_PAD_RIGHT	0
#define JOY_BTN_LEFT_PAD_LEFT	1
#define JOY_BTN_LEFT_PAD_UP	2
#define JOY_BTN_LEFT_PAD_DOWN	3
#define JOY_BTN_RIGHT_PAD_X	4
#define JOY_BTN_RIGHT_PAD_Y	5
#define JOY_BTN_RIGHT_PAD_A	6
#define JOY_BTN_RIGHT_PAD_B	7
#define JOY_BTN_LEFT_FINGER	8
#define JOY_BTN_RIGHT_FINGER	9
#define JOY_BTN_LEFT_THROTTLE	10
#define JOY_BTN_RIGHT_THROTTLE	11
#define JOY_BTN_BACK		12
#define JOY_BTN_START		13
#define JOY_BTN_LEFT_THUMB	14
#define JOY_BTN_RIGHT_THUMB	15

#define HOSTNAME "rassor2" 
int JOY_THUMB_AXIS_RANGE_RAW;
int JOY_FINGER_AXIS_RANGE_RAW;
float JOY_AXIS_DEADBAND;
int JOY_PUBLISH_BUFFER_SIZE;
int JOY_PUBLISH_RATE_HZ;
int JOY_CONNECT_RETRY_SECONDS;
int JOY_NUM_COMPATIBLE;

/*
 * This subroutine loads all parameters from the ROS Parameter Server
 */
int initParams() {
    //ros::param::get("joystick_driver/hostname", HOSTNAME);	
    ros::param::get("joystick_driver/axis/thumb_range_raw", JOY_THUMB_AXIS_RANGE_RAW);
    ros::param::get("joystick_driver/axis/finger_range_raw", JOY_FINGER_AXIS_RANGE_RAW);
    ros::param::get("joystick_driver/axis/deadband", JOY_AXIS_DEADBAND);
    ros::param::get("joystick_driver/publish/buffer_size", JOY_PUBLISH_BUFFER_SIZE);
    ros::param::get("joystick_driver/publish/rate_hz", JOY_PUBLISH_RATE_HZ);
    ros::param::get("joystick_driver/connect_retry_seconds", JOY_CONNECT_RETRY_SECONDS);
    ros::param::get("joystick_driver/num_compatible", JOY_NUM_COMPATIBLE);

    // TODO: remove after parameter server is initialized with correct values
    JOY_THUMB_AXIS_RANGE_RAW = 65536;
    JOY_FINGER_AXIS_RANGE_RAW = 256;
    JOY_AXIS_DEADBAND = 0.15;
    JOY_PUBLISH_BUFFER_SIZE = 10;
    JOY_PUBLISH_RATE_HZ = 100;
    JOY_CONNECT_RETRY_SECONDS = 2;
    JOY_NUM_COMPATIBLE = 6;

    return SUCCESS;
}

/*
 * This subroutine scales the raw joystick axis values into Joy messages and also handles deadband
 */
sensor_msgs::Joy joy_scale_deadband(sensor_msgs::Joy joy_msg,
        float raw_value,
        int which_axis,
        int negate) {
    // perform the scaling
    if ((JOY_AXIS_LEFT_THUMB_X == which_axis) || (JOY_AXIS_LEFT_THUMB_Y == which_axis) ||
            (JOY_AXIS_RIGHT_THUMB_X == which_axis) || (JOY_AXIS_RIGHT_THUMB_Y == which_axis)) {
        // scaling for sticks (goes negative 1 to positive 1)
        joy_msg.axes[which_axis] = (float) (raw_value * (negate) / (JOY_THUMB_AXIS_RANGE_RAW / 2));
    } else if ((JOY_AXIS_LEFT_FINGER == which_axis) || (JOY_AXIS_RIGHT_FINGER == which_axis)) {
        // scaling for analog buttons (goes zero to positive 1)
        joy_msg.axes[which_axis] = (float) (raw_value * (negate) / JOY_FINGER_AXIS_RANGE_RAW);
    } else {
        ROS_ERROR("Unknown joystick axis ID: %d", which_axis);
    }

    // handle deadband in middle of joysticks
    if ((joy_msg.axes[which_axis] < JOY_AXIS_DEADBAND) &&
            (joy_msg.axes[which_axis] > (JOY_AXIS_DEADBAND * -1))) {
        // close enough to zero to be at full stop
        joy_msg.axes[which_axis] = 0.0;
    }

    return joy_msg;
}

void killHandler(int s) {
    exit(0);
}

/*
 * This module reads raw joystick inputs and publishes sensor_msgs::Joy messages 
 * for the onboard motion control module to respond to.
 */
int main(int argc, char* argv[]) {
    char host[128];
    gethostname(host, sizeof (host));
    string hostname(host);

    ROS_INFO("joystick driver: module is initializing (v%s)", MODULE_VERSION);

    //Gets the name of the robot to publish for joystick. i.e. "/moe/joystick"

    string robotInputName;
    char *robotJoystickName;
    string robotJoystickPath;
    //if there is a name input, assign the name to a char array 
    if (argc >= 2) {
        robotInputName = argv[1];
        robotJoystickPath = "/" + robotInputName + "/joystick";
        //changes the input from user to a char array for sprintf())
        robotJoystickName = new char[robotJoystickPath.size() + 1];
        robotJoystickName[robotJoystickPath.size()] = 0;
        memcpy(robotJoystickName, robotJoystickPath.c_str(), robotJoystickPath.size());
    } else {
        robotInputName = hostname;
        robotJoystickPath = "/" + robotInputName + "/joystick";
        //changes the input from user to a char array for sprintf())
        robotJoystickName = new char[robotJoystickPath.size() + 1];
        robotJoystickName[robotJoystickPath.size()] = 0;
        memcpy(robotJoystickName, robotJoystickPath.c_str(), robotJoystickPath.size());
        cout << "No name input. Default is:" << hostname << endl;
    }

    // set up ROS node
    char node_name[STRING_LENGTH];
    sprintf(node_name, "%s_joystick_driver", HOSTNAME);
    ros::init(argc, argv, node_name);
    ros::NodeHandle n;
    ROS_DEBUG("joystick driver: node initialized successfully");

    //initialize Parameters
    initParams();
    ROS_DEBUG("joystick driver: parameters loaded successfully");

    // set up ROS message publication
    char publish_path[STRING_LENGTH];
    //sprintf(publish_path, "/%s/joystick", HOSTNAME);
    //checks for input name

    sprintf(publish_path, robotJoystickName);

    ros::Publisher my_publisher = n.advertise<sensor_msgs::Joy>(publish_path, JOY_PUBLISH_BUFFER_SIZE);
    ros::Rate loop_rate(JOY_PUBLISH_RATE_HZ);
    ROS_DEBUG("joystick driver: message publication initialized successfully");

    int fd; // file handle for joystick input file
    struct input_event ie; // event data type that the joystick file data is in
    sensor_msgs::Joy joy_msg; // this is the ROS message we will publish
    joy_msg.header.frame_id = "none"; // transform frame ID, not really used here

    // set size of Joy message based on number of buttons and axes
    joy_msg.axes.resize(JOY_TOTAL_AXES);
    joy_msg.buttons.resize(JOY_TOTAL_BUTTONS);

    // wait forever for joystick to be plugged in
    bool joystick_found = false;
    while (!joystick_found) {
        // iterate over all known compatible joystick file types
        int index;
        char file_path[STRING_LENGTH];
        for (index = 0; index < JOY_NUM_COMPATIBLE; index++) {
            // compose complete file path
            sprintf(file_path, "/dev/input/by-id/%s", compatible_joystick_labels[index]);
            // try to open joystick file
            if ((fd = open(file_path, O_RDONLY)) == ERROR) {
                ROS_DEBUG("joystick driver: none found at %s", file_path);
            } else {
                ROS_INFO("joystick driver: found joystick/gamepad");
                joystick_found = true;
            }

            // break out of for loop after the first is found
            if (true == joystick_found) {
                break;
            }
        }

        // delay for the next retry
        if (false == joystick_found) {
            sleep(JOY_CONNECT_RETRY_SECONDS);
            ROS_DEBUG("joystick driver: retrying for joystick connect");
        }
        signal(SIGINT, killHandler);
    }

    // TODO:  add code to notice when joystick is unplugged and halt

    // outer loop includes sleep to keep from publishing cyclic messages without joystick inputs
    while (ros::ok()) {
        // TODO: a bunch of joystick commands can come in quickly, so compile
        //       all into a single ROS message publication at 100 Hz
        //       otherwise we go way over our target rate of 100 Hz
        // TODO: note and minimize bandwidth (use RQT to monitor)
        while (ros::ok()) {
            // TODO: this blocks, control-c can't even break out!
            read(fd, &ie, sizeof (struct input_event));

            ROS_DEBUG("time %ld.%06ld\ttype %d\tcode %d\tvalue %d",
                    ie.time.tv_sec, ie.time.tv_usec, ie.type, ie.code, ie.value);

            float raw_value = ie.value;
            bool publish_msg = true;

            // scale the axis or set the button that has changed

            // left joystick X-axis (don't negate)
            if ((ie.type == 3) && (ie.code == 0)) {
                joy_msg = joy_scale_deadband(joy_msg, raw_value, JOY_AXIS_LEFT_THUMB_X, -1);
            }// left joystick Y-axis (negate)
            else if ((ie.type == 3) && (ie.code == 1)) {
                joy_msg = joy_scale_deadband(joy_msg, raw_value, JOY_AXIS_LEFT_THUMB_Y, -1);
            }// left throttle treated as both an axis and a button (don't negate)
            else if ((ie.type == 3) && (ie.code == 2)) {
                // axis scaling
                joy_msg = joy_scale_deadband(joy_msg, raw_value, JOY_AXIS_LEFT_FINGER, 1);
                // button logic
                if (joy_msg.axes[JOY_AXIS_LEFT_FINGER] >= 0.8) {
                    joy_msg.buttons[JOY_BTN_LEFT_THROTTLE] = 1;
                } else {
                    joy_msg.buttons[JOY_BTN_LEFT_THROTTLE] = 0;
                }
            }// right joystick X-axis (don't negate)
            else if ((ie.type == 3) && (ie.code == 3)) {
                joy_msg = joy_scale_deadband(joy_msg, raw_value, JOY_AXIS_RIGHT_THUMB_X, 1);
            }// right joystick Y-axis (negate)
            else if ((ie.type == 3) && (ie.code == 4)) {
                joy_msg = joy_scale_deadband(joy_msg, raw_value, JOY_AXIS_RIGHT_THUMB_Y, -1);
            }// right throttle treated as both an axis and a button (don't negate)
            else if ((ie.type == 3) && (ie.code == 5)) {
                // axis scaling
                joy_msg = joy_scale_deadband(joy_msg, raw_value, JOY_AXIS_RIGHT_FINGER, 1);
                // button logic
                if (joy_msg.axes[JOY_AXIS_RIGHT_FINGER] >= 0.8) {
                    joy_msg.buttons[JOY_BTN_RIGHT_THROTTLE] = 1;
                } else {
                    joy_msg.buttons[JOY_BTN_RIGHT_THROTTLE] = 0;
                }
            }// left pad, right and left buttons
            else if ((ie.type == 3) && (ie.code == 16)) {
                if (1 == raw_value) {
                    joy_msg.buttons[JOY_BTN_LEFT_PAD_RIGHT] = 1;
                    joy_msg.buttons[JOY_BTN_LEFT_PAD_LEFT] = 0;
                } else if (-1 == raw_value) {
                    joy_msg.buttons[JOY_BTN_LEFT_PAD_RIGHT] = 0;
                    joy_msg.buttons[JOY_BTN_LEFT_PAD_LEFT] = 1;
                } else {
                    joy_msg.buttons[JOY_BTN_LEFT_PAD_RIGHT] = 0;
                    joy_msg.buttons[JOY_BTN_LEFT_PAD_LEFT] = 0;
                }
            }// left pad, up and down buttons
            else if ((ie.type == 3) && (ie.code == 17)) {
                if (1 == raw_value) {
                    joy_msg.buttons[JOY_BTN_LEFT_PAD_UP] = 0;
                    joy_msg.buttons[JOY_BTN_LEFT_PAD_DOWN] = 1;
                } else if (-1 == raw_value) {
                    joy_msg.buttons[JOY_BTN_LEFT_PAD_UP] = 1;
                    joy_msg.buttons[JOY_BTN_LEFT_PAD_DOWN] = 0;
                } else {
                    joy_msg.buttons[JOY_BTN_LEFT_PAD_UP] = 0;
                    joy_msg.buttons[JOY_BTN_LEFT_PAD_DOWN] = 0;
                }
            }// right pad, X button
            else if ((ie.type == 1) && (ie.code == 307)) {
                joy_msg.buttons[JOY_BTN_RIGHT_PAD_X] = (int) raw_value;
            }// right pad, Y button
            else if ((ie.type == 1) && (ie.code == 308)) {
                joy_msg.buttons[JOY_BTN_RIGHT_PAD_Y] = (int) raw_value;
            }// right pad, A button
            else if ((ie.type == 1) && (ie.code == 304)) {
                joy_msg.buttons[JOY_BTN_RIGHT_PAD_A] = (int) raw_value;
            }// right pad, B button
            else if ((ie.type == 1) && (ie.code == 305)) {
                joy_msg.buttons[JOY_BTN_RIGHT_PAD_B] = (int) raw_value;
            }// left finger button
            else if ((ie.type == 1) && (ie.code == 310)) {
                joy_msg.buttons[JOY_BTN_LEFT_FINGER] = (int) raw_value;
            }// right finger button
            else if ((ie.type == 1) && (ie.code == 311)) {
                joy_msg.buttons[JOY_BTN_RIGHT_FINGER] = (int) raw_value;
            }// back button
            else if ((ie.type == 1) && (ie.code == 314)) {
                joy_msg.buttons[JOY_BTN_BACK] = (int) raw_value;
            }// start button
            else if ((ie.type == 1) && (ie.code == 315)) {
                joy_msg.buttons[JOY_BTN_START] = (int) raw_value;
            }// left thumb joystick press button
            else if ((ie.type == 1) && (ie.code == 317)) {
                joy_msg.buttons[JOY_BTN_LEFT_THUMB] = (int) raw_value;
            }// right thumb joystick press button
            else if ((ie.type == 1) && (ie.code == 318)) {
                joy_msg.buttons[JOY_BTN_RIGHT_THUMB] = (int) raw_value;
            }// all other types and codes are ignored
            else {
                publish_msg = false;
            }

            if (publish_msg) {
                // write time into ROS message header
                joy_msg.header.stamp = ros::Time().now();

                // publish Joystick message
                my_publisher.publish(joy_msg);
                ros::spinOnce();
            }

        } // end while file read
        signal(SIGINT, killHandler);
        // sleep for next publish cycle
        // this only occurs if the data rate from the joystick is way low
        loop_rate.sleep();

    } // end while ROS is OK

    return (SUCCESS);
}






