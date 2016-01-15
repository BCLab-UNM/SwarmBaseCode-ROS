#include <ros/ros.h>

//ROS libraries
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

//ROS messages
#include <std_msgs/String.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Range.h>

//Package include
#include <usbSerial.h>

using namespace std;

//aBridge functions
void cmdHandler(const geometry_msgs::Twist::ConstPtr& message);
void serialActivityTimer(const ros::TimerEvent& e);
void publishRosTopics();
void parseData(string data);

//Globals
sensor_msgs::Imu imu;
nav_msgs::Odometry odom;
sensor_msgs::Range sonarLeft;
sensor_msgs::Range sonarCenter;
sensor_msgs::Range sonarRight;
USBSerial usb;
const int baud = 115200;
char dataCmd[] = "d\n";
char moveCmd[16];
char host[128];
char delimiter = ',';
vector<string> dataSet;
float linearSpeed = 0.;
float turnSpeed = 0.;
const float deltaTime = 0.1;

//Publishers
ros::Publisher imuPublish;
ros::Publisher odomPublish;
ros::Publisher sonarLeftPublish;
ros::Publisher sonarCenterPublish;
ros::Publisher sonarRightPublish;

//Subscribers
ros::Subscriber velocitySubscriber;

//Timers
ros::Timer publishTimer;

int main(int argc, char **argv) {
    
    gethostname(host, sizeof (host));
    string hostname(host);
    string publishedName;
    ros::init(argc, argv, (hostname + "_ABRIDGE"));
    
    ros::NodeHandle param("~");
    string devicePath;
    param.param("device", devicePath, string("/dev/ttyUSB0"));
    usb.openUSBPort(devicePath, baud);
    
    sleep(5);
    
    ros::NodeHandle aNH;
    
    if (argc >= 2) {
        publishedName = argv[1];
        cout << "Welcome to the world of tomorrow " << publishedName << "!  ABridge module started." << endl;
    } else {
        publishedName = hostname;
        cout << "No Name Selected. Default is: " << publishedName << endl;
    }
    
    imuPublish = aNH.advertise<sensor_msgs::Imu>((publishedName + "/imu"), 10);
    odomPublish = aNH.advertise<nav_msgs::Odometry>((publishedName + "/odom"), 10);
    sonarLeftPublish = aNH.advertise<sensor_msgs::Range>((publishedName + "/sonarLeft"), 10);
    sonarCenterPublish = aNH.advertise<sensor_msgs::Range>((publishedName + "/sonarCenter"), 10);
    sonarRightPublish = aNH.advertise<sensor_msgs::Range>((publishedName + "/sonarRight"), 10);
    
    velocitySubscriber = aNH.subscribe((publishedName + "/velocity"), 10, cmdHandler);
    
    publishTimer = aNH.createTimer(ros::Duration(deltaTime), serialActivityTimer);
    
    imu.header.frame_id = "base_link";
    
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";
    
    ros::spin();
    
    return EXIT_SUCCESS;
}

void cmdHandler(const geometry_msgs::Twist::ConstPtr& message) {
    // remove artificial factor that was multiplied for simulation. this scales it back down to -1.0 to +1.0
    linearSpeed = (message->linear.x) / 1.5;
    turnSpeed = (message->angular.z) / 8;
    
    if (linearSpeed != 0.) {
        sprintf(moveCmd, "m,%d\n", (int) (linearSpeed * 255));
        usb.sendData(moveCmd);
    } else if (turnSpeed != 0.) {
        sprintf(moveCmd, "t,%d\n", (int) (turnSpeed * 255));
        usb.sendData(moveCmd);
    } else {
        sprintf(moveCmd, "s\n");
        usb.sendData(moveCmd);
    }
    
    memset(&moveCmd, '\0', sizeof (moveCmd));
}

void serialActivityTimer(const ros::TimerEvent& e) {
    usb.sendData(dataCmd);
    parseData(usb.readData());
    publishRosTopics();
}

void publishRosTopics() {
    imuPublish.publish(imu);
    odomPublish.publish(odom);
    sonarLeftPublish.publish(sonarLeft);
    sonarCenterPublish.publish(sonarCenter);
    sonarRightPublish.publish(sonarRight);
    
    tf::TransformBroadcaster odomTransformPublish;
    geometry_msgs::TransformStamped odomTransform;
    odomTransform.header.stamp = ros::Time::now();
    odomTransform.header.frame_id = "odom";
    odomTransform.child_frame_id = "base_link";
    odomTransform.transform.translation.x = odom.pose.pose.position.x;
    odomTransform.transform.translation.y = odom.pose.pose.position.y;
    odomTransform.transform.translation.z = 0.0;
    odomTransform.transform.rotation = odom.pose.pose.orientation;
    odomTransformPublish.sendTransform(odomTransform);
}

void parseData(string str) {
    istringstream oss(str);
    string word;
    while (getline(oss, word, delimiter)) {
        dataSet.push_back(word);
    }
    if (dataSet.size() == 18) {
        imu.header.stamp = ros::Time::now();
        imu.linear_acceleration.x = atof(dataSet.at(0).c_str());
        imu.linear_acceleration.y = atof(dataSet.at(1).c_str());
        imu.linear_acceleration.z = atof(dataSet.at(2).c_str());
        imu.angular_velocity.x = atof(dataSet.at(3).c_str());
        imu.angular_velocity.y = atof(dataSet.at(4).c_str());
        imu.angular_velocity.z = atof(dataSet.at(5).c_str());
        imu.orientation = tf::createQuaternionMsgFromRollPitchYaw(atof(dataSet.at(6).c_str()), atof(dataSet.at(7).c_str()), atof(dataSet.at(8).c_str()));
        
        odom.header.stamp = ros::Time::now();
        odom.pose.pose.position.x += atof(dataSet.at(9).c_str()) / 100.0;
        odom.pose.pose.position.y += atof(dataSet.at(10).c_str()) / 100.0;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(atof(dataSet.at(11).c_str()));
        odom.twist.twist.linear.x = atof(dataSet.at(12).c_str()) / 100.0;
        odom.twist.twist.linear.y = atof(dataSet.at(13).c_str()) / 100.0;
        odom.twist.twist.angular.z = atof(dataSet.at(14).c_str());
        
        sonarLeft.range = atof(dataSet.at(15).c_str()) / 100.0;
        sonarCenter.range = atof(dataSet.at(16).c_str()) / 100.0;
        sonarRight.range = atof(dataSet.at(17).c_str()) / 100.0;
    }
    
    dataSet.clear();
}
