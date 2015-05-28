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
 * 	10/14/2014 (kwl): added covariance values to sensor data publication
 *      
 */

#include <string.h>
#include <unistd.h>  
#include <sstream>
#include <vector>
#include <usbSerial.h>
#include <math.h>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt64.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Quaternion.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
// #include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <tf/tf.h>

using namespace std;

// optional parameter server stored variables
float ABRIDGE_TIMER_VALUE_IN_SECONDS =		0.20;	
float WHEEL_ODOM_COVARIANCE_INIT_VALUE =	0.1;	
float IMU_SENSOR_COVARIANCE_INIT_VALUE =	1.0;	
float GPS_SENSOR_COVARIANCE_INIT_VALUE =	10.0;	
float WHEEL_ODOM_COVARIANCE_GROWTH_FACTOR =	0.0001;	
float IMU_SENSOR_COVARIANCE_GROWTH_FACTOR =	0.000001;	
float GPS_SENSOR_COVARIANCE_GROWTH_FACTOR =	0.0;	

#ifndef M_PI
    double M_PI = 3.14159265358979323846264338327950288419716939937510582;
#endif

void cmdHandler(const geometry_msgs::Twist::ConstPtr& message);
//void teleopHBHandler(const std_msgs::UInt64::ConstPtr& message);
void updateHB(const ros::TimerEvent& e);
void updateData(const ros::TimerEvent& e);
void publishData();
void parseData(string data);
void initParams();

char host[128];
char delimiter = ',';
vector<string> dataSet;
long serverHB = 0;
long internalHB = 0;

USBSerial usb;
char port[] = "/dev/ttyUSB0";
int baud = 115200;
char dataCmd[] = "d\n";
char moveCmd[16];
int COVARIANCE_TYPE_UNKNOWN =		0;
int COVARIANCE_TYPE_APPROXIMATED =	1;
int COVARIANCE_TYPE_DIAGONAL_KNOWN =	2;
int COVARIANCE_TYPE_KNOWN =		3;
float wheel_odom_covariance_value;	
float imu_sensor_covariance_value;	
float gps_sensor_covariance_value;
unsigned int timer_counter = 0;	
unsigned int timer_frequency = 0;	

std_msgs::String rtime;
std_msgs::String status;
sensor_msgs::Range usLeft;
sensor_msgs::Range usCenter;
sensor_msgs::Range usRight;
sensor_msgs::Imu imu;
sensor_msgs::NavSatFix gps;
//nav_msgs::Odometry odom;

ros::Subscriber moveSubscriber;

ros::Subscriber hbSubscriber;
ros::Timer hbTimer;

ros::Publisher imuPublish;
ros::Publisher gpsPublish;
//ros::Publisher odomPublish;
ros::Publisher usLeftPublish;
ros::Publisher usCenterPublish;
ros::Publisher usRightPublish;
ros::Publisher timePublish;
ros::Publisher statusPublish;
ros::Timer publishTimer;

int main(int argc, char **argv) {

    gethostname(host, sizeof (host));
    string hostname(host);
    string publishedName;

    ros::init(argc, argv, (hostname + "_ABRIDGE"));
    initParams();

    usb.openUSBPort(port, baud);
    sleep(5);

    ros::NodeHandle aNH;
    
    if (argc >= 2) {
        publishedName = argv[1];
        cout << "Welcome to the world of tomorrow " << publishedName << "!  ABridge module started." << endl;
    } else {
        publishedName = hostname;
        cout << "No Name Selected. Default is: " << publishedName << endl;
    }

    moveSubscriber = aNH.subscribe((publishedName + "/mobility"), 10, cmdHandler);
    //hbSubscriber = aNH.subscribe((publishedName + "/teleop/heartbeat"), 10, teleopHBHandler);
    //hbTimer = aNH.createTimer(ros::Duration(1.0), updateHB);

    imuPublish = aNH.advertise<sensor_msgs::Imu>((publishedName + "/imu"), 10);
    gpsPublish = aNH.advertise<sensor_msgs::NavSatFix>((publishedName + "/gps"), 10);
    //odomPublish = aNH.advertise<nav_msgs::Odometry>((publishedName + "/odom"), 10);
    timePublish = aNH.advertise<std_msgs::String>((publishedName + "/time"), 10);
    statusPublish = aNH.advertise<std_msgs::String>((publishedName + "/status"), 10);

    timer_frequency = 1 / ABRIDGE_TIMER_VALUE_IN_SECONDS;
    publishTimer = aNH.createTimer(ros::Duration(ABRIDGE_TIMER_VALUE_IN_SECONDS), updateData);
    
    usLeftPublish = aNH.advertise<sensor_msgs::Range>((publishedName + "/USLeft"), 10);
    usCenterPublish = aNH.advertise<sensor_msgs::Range>((publishedName + "/USCenter"), 10);
    usRightPublish = aNH.advertise<sensor_msgs::Range>((publishedName + "/USRight"), 10);
    
    usLeft.radiation_type = 0;
    usLeft.field_of_view = 55;
    usCenter.radiation_type = 0;
    usCenter.field_of_view = 55;
    usRight.radiation_type = 0;
    usRight.field_of_view = 55;

    wheel_odom_covariance_value = WHEEL_ODOM_COVARIANCE_INIT_VALUE;	
    imu_sensor_covariance_value = IMU_SENSOR_COVARIANCE_INIT_VALUE;	
    gps_sensor_covariance_value = GPS_SENSOR_COVARIANCE_INIT_VALUE;	
    
    status.data = "INITIALIZED";

    while (ros::ok()) {
       ros::spin();
    }

    return EXIT_SUCCESS;
}

//void teleopHBHandler(const std_msgs::UInt64::ConstPtr& message) {
//    serverHB = message->data;
//    internalHB = serverHB;
//}

//void updateHB(const ros::TimerEvent& e) {
//    //if (serverHB < (internalHB - 5)) {
//        //cout << "stop" << endl;
//        sprintf(moveCmd, "s\n");
//        usb.sendData(moveCmd);
//        //cout << "                       STOP TIMEOUT" << endl;
//    //}
//    internalHB++;
//}

void cmdHandler(const geometry_msgs::Twist::ConstPtr& message) {
    //cout << "MOVE COMMAND" << endl;

    boost::array<double, 36> odom_covariance_array = {{
	wheel_odom_covariance_value, 0, 0,
	0, 0, 0,
	0, wheel_odom_covariance_value, 0,
	0, 0, 0,
	0, 0, wheel_odom_covariance_value,
	0, 0, 0,
	0, 0, 0, 99999, 0, 0,
	0, 0, 0, 0, 99999, 0,
	0, 0, 0, 0, 0, 99999
	}};
    //odom.pose.covariance = odom_covariance_array;
    //odom.twist.covariance = odom_covariance_array;

    if (message->linear.x < 0) {
        sprintf(moveCmd, "m,%d,%d\n", (int) (message->linear.x * 255), (int) (message->linear.x * 255));
        usb.sendData(moveCmd);
        //cout << "forward" << endl;
    } else if (message->linear.x > 0) {
        sprintf(moveCmd, "m,%d,%d\n", (int) (message->linear.x * 255), (int) (message->linear.x * 255));
        usb.sendData(moveCmd);
        //cout << "back" << endl;
    } else if (message->angular.z > 0) {
        sprintf(moveCmd, "t,%d\n", (int) (message->angular.z * 255));
        usb.sendData(moveCmd);
        //cout << "left" << endl;
    } else if (message->angular.z < 0) {
        sprintf(moveCmd, "t,%d\n", (int) (message->angular.z * 255));
        usb.sendData(moveCmd);
        //cout << "right" << endl;
    } else if (message->angular.z == 0 || message->linear.x == 0) {
        sprintf(moveCmd, "s\n");
        usb.sendData(moveCmd);
        //cout << "STOP" << endl;
    }
    memset(&moveCmd, '\0', sizeof (moveCmd));
}

void updateData(const ros::TimerEvent& e) {
    timer_counter++;
    usb.sendData(dataCmd);
    parseData(usb.readData());
    status.data = "ONLINE";
    publishData();
}

void publishData() {
    timePublish.publish(rtime);
    usLeftPublish.publish(usLeft);
    usCenterPublish.publish(usCenter);
    usRightPublish.publish(usRight);
    imuPublish.publish(imu);
    gpsPublish.publish(gps);
    //odomPublish.publish(odom);
    statusPublish.publish(status); 
}

void parseData(string str) {
    istringstream oss(str);
    string word;
    while (getline(oss, word, delimiter)) {
        dataSet.push_back(word);
    }
    if (dataSet.size() >= 14) {
        if (!(timer_counter % timer_frequency)) {
                // grow covariance values once per second
        	gps_sensor_covariance_value = gps_sensor_covariance_value * 
							(1 + GPS_SENSOR_COVARIANCE_GROWTH_FACTOR);
        	imu_sensor_covariance_value = imu_sensor_covariance_value * 
							(1 + IMU_SENSOR_COVARIANCE_GROWTH_FACTOR);
        	wheel_odom_covariance_value = wheel_odom_covariance_value * 
							(1 + WHEEL_ODOM_COVARIANCE_GROWTH_FACTOR);
        }
        rtime.data = dataSet.at(0);
        gps.latitude = atof(dataSet.at(1).c_str());
        gps.longitude = atof(dataSet.at(2).c_str());
        boost::array<double, 9> gps_covariance_array = {{
	    gps_sensor_covariance_value, 0, 0,
	    0, gps_sensor_covariance_value, 0,
	    0, 0, gps_sensor_covariance_value
	    }};
        gps.position_covariance = gps_covariance_array;
        gps.position_covariance_type = COVARIANCE_TYPE_APPROXIMATED;
        imu.orientation = tf::createQuaternionMsgFromYaw(atof(dataSet.at(3).c_str()) * M_PI / 180.0);
        imu.orientation.z = fmod(-1 * imu.orientation.z, 360.0);
        imu.angular_velocity.x = atof(dataSet.at(4).c_str());
        imu.angular_velocity.y = atof(dataSet.at(5).c_str());
        imu.angular_velocity.z = atof(dataSet.at(6).c_str());
        imu.linear_acceleration.x = atof(dataSet.at(7).c_str());
        imu.linear_acceleration.y = atof(dataSet.at(8).c_str());
        imu.linear_acceleration.z = atof(dataSet.at(9).c_str());
        boost::array<double, 9> imu_covariance_array = {{
	    imu_sensor_covariance_value, 0, 0,
	    0, imu_sensor_covariance_value, 0,
	    0, 0, imu_sensor_covariance_value
	    }};
        imu.orientation_covariance = imu_covariance_array;
        imu.angular_velocity_covariance = imu_covariance_array;
        imu.linear_acceleration_covariance = imu_covariance_array;
        usCenter.range = atof(dataSet.at(10).c_str()) / 100.0;
        usLeft.range = atof(dataSet.at(11).c_str()) / 100.0;
        usRight.range = atof(dataSet.at(12).c_str()) / 100.0;
        int temp = (int) atof(dataSet.at(13).c_str());
        if(temp <= 0){
            gps.status.STATUS_NO_FIX;
        } else {
            gps.status.STATUS_FIX;
        }
    }
    
    dataSet.clear();
}

void initParams() {
	// if the parameters exist in the parameter server they will get overwritten, 
	// otherwise hard coded values will remain in place
	ros::param::get("/ABRIDGE_TIMER_VALUE_IN_SECONDS", ABRIDGE_TIMER_VALUE_IN_SECONDS);	
	ros::param::get("/WHEEL_ODOM_COVARIANCE_INIT_VALUE", WHEEL_ODOM_COVARIANCE_INIT_VALUE);
	ros::param::get("/IMU_SENSOR_COVARIANCE_INIT_VALUE", IMU_SENSOR_COVARIANCE_INIT_VALUE);
	ros::param::get("/GPS_SENSOR_COVARIANCE_INIT_VALUE", GPS_SENSOR_COVARIANCE_INIT_VALUE);
	ros::param::get("/WHEEL_ODOM_COVARIANCE_GROWTH_FACTOR", WHEEL_ODOM_COVARIANCE_GROWTH_FACTOR);
	ros::param::get("/IMU_SENSOR_COVARIANCE_GROWTH_FACTOR", IMU_SENSOR_COVARIANCE_GROWTH_FACTOR);
	ros::param::get("/GPS_SENSOR_COVARIANCE_GROWTH_FACTOR", GPS_SENSOR_COVARIANCE_GROWTH_FACTOR);
}


