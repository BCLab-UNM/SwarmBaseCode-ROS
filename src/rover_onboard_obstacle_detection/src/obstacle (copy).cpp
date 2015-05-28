/*
 * Author: Karl A. Stolleis
 * Maintainer: Karl A. Stolleis
 * Email: karl.a.stolleis@nasa.gov; kurt.leucht@nasa.gov
 * NASA Center: Kennedy Space Center
 * Mail Stop: NE-C1
 * 
 * Project Name: Swarmie Robotics NASA Center Innovation Fund
 * Principal Investigator: Cheryle Mako
 * Email: cheryle.l.mako@nasa.gov
 * 
 * Date Created: June 6, 2014
 * Safety Critical: NO
 * NASA Software Classification: D
 * 
 * This software is copyright the National Aeronautics and Space Administration (NASA)
 * and is distributed under the GNU LGPL license.  All rights reserved.
 * Permission to use, copy, modify and distribute this software is granted under
 * the LGPL and there is no implied warranty for this software.  This software is provided
 * "as is" and NASA or the authors are not responsible for indirect or direct damage
 * to any user of the software.  The authors and NASA are under no obligation to provide
 * maintenence, updates, support or modifications to the software.
 * 
 * Revision Log:
 *      
 */

#include <cstdlib>
#include <string.h>
#include <string>
#include <iostream>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt64.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Float32MultiArray.h>
#include <sensor_msgs/Range.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Vector3.h>

using namespace std;

void leftHandler(const sensor_msgs::Range::ConstPtr& message);
void centerHandler(const sensor_msgs::Range::ConstPtr& message);
void rightHandler(const sensor_msgs::Range::ConstPtr& message);
//void modeHandler(const std_msgs::UInt8::ConstPtr& message);
void parameterHandler(const std_msgs::Float32MultiArray message);
void loadParameters(const ros::TimerEvent&);

void setMode();

string publishedName;
char host[128];

ros::Subscriber leftSubscriber;
ros::Subscriber centerSubscriber;
ros::Subscriber rightSubscriber;
//ros::Subscriber modeSubscriber;
ros::Timer parameterTimer;

ros::Publisher obstaclePublish;
std_msgs::UInt8 obstacleMode;

float leftVal = 500.0;
float centerVal = 500.0;
float rightVal = 500.0;
//bool autoMode = 500.0;

//Default hardcoded parameters, consider moving these to be UI driven or from Parameter Server
float senseRange = .25; //meters the ultrasonic detectors look for obstacles
float cautionRange = .5; //meters the ultrasonic detectors command the robot to slow down to caution speed
float parameterTime = 1.0; //how often to check for new parameter server values

int main(int argc, char** argv) {

    gethostname(host, sizeof (host));
    string hostname(host);

    if (argc >= 2) {
        publishedName = argv[1];
        cout << "Welcome to the world of tomorrow " << publishedName << "! Obstacle module started." << endl;
    } else {
        publishedName = hostname;
        cout << "No name selected. Default is: " << publishedName << endl;
    }

    ros::init(argc, argv, (publishedName + "_OBSTACLE"));
    ros::NodeHandle oNH;

    ros::TimerEvent actNow;
    loadParameters(actNow);
    
    leftSubscriber = oNH.subscribe((publishedName + "/USLeft"), 10, leftHandler);
    centerSubscriber = oNH.subscribe((publishedName + "/USCenter"), 10, centerHandler);
    rightSubscriber = oNH.subscribe((publishedName + "/USRight"), 10, rightHandler);
    //modeSubscriber = oNH.subscribe((publishedName + "/mode"), 10, modeHandler);

    obstaclePublish = oNH.advertise<std_msgs::UInt8>((publishedName + "/obstacle"), 10);

    parameterTimer = oNH.createTimer(ros::Duration(parameterTime), loadParameters);

    while (ros::ok()) {
        ros::spin();
    }

    return 0;
}

void leftHandler(const sensor_msgs::Range::ConstPtr& message) {
    leftVal = message->range;
    setMode();
}

void centerHandler(const sensor_msgs::Range::ConstPtr& message) {
    centerVal = message->range;
    setMode();
}

void rightHandler(const sensor_msgs::Range::ConstPtr& message) {
    rightVal = message->range;
    setMode();
}

//void modeHandler(const std_msgs::UInt8::ConstPtr& message) {
    //if (message->data > 1) {
        //autoMode = true;
    //} else {
        //autoMode = false;
    //}

//}

void setMode() {
    if (leftVal > senseRange && centerVal < senseRange && rightVal > senseRange) {
        obstacleMode.data = 2;
    } else if (leftVal < senseRange && centerVal > senseRange && rightVal > senseRange) {
        obstacleMode.data = 3;
    } else if (leftVal > senseRange && centerVal > senseRange && rightVal < senseRange) {
        obstacleMode.data = 4;
    } else if (leftVal < senseRange && centerVal < senseRange && rightVal > senseRange) {
        obstacleMode.data = 5;
    } else if (leftVal > senseRange && centerVal < senseRange && rightVal < senseRange) {
        obstacleMode.data = 6;
    } else if (leftVal < senseRange && centerVal > senseRange && rightVal < senseRange) {
        obstacleMode.data = 7;
    } else if (leftVal < senseRange && centerVal < senseRange && rightVal < senseRange) {
        obstacleMode.data = 8;
    } else if (leftVal < cautionRange || centerVal < cautionRange || rightVal < cautionRange) {
        obstacleMode.data = 9;
    } else {
        obstacleMode.data = 1;
    }    

    obstaclePublish.publish(obstacleMode);
}

void loadParameters(const ros::TimerEvent&) {
/*
 * Load parameters from parameter server. ROS doesn't seem to follow latest C++11
 * standard so while it says it supports floats, it doesn't because floats 
 * can't be passed to templates in C++11. Doubles can, so passing everything as
 * a double and casting it back to a float.
 * 
 * Or I just can't figure it out....
 * 
 */    
    double param;

    ros::param::param((publishedName + "/mobility/senseRange"), param, (double) senseRange); senseRange = (float) param;
    ros::param::param((publishedName + "/mobility/cautionRange"), param, (double) cautionRange); cautionRange = (float) param;
}


