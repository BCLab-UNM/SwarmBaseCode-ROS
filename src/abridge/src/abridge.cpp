#include <ros/ros.h>

//ROS libraries
#include <tf/transform_datatypes.h>

//ROS messages
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Range.h>

//Package include
#include <usbSerial.h>

using namespace std;

//aBridge functions
void cmdHandler(const geometry_msgs::Twist::ConstPtr& message);
void fingerAngleHandler(const std_msgs::Float32::ConstPtr& angle);
void wristAngleHandler(const std_msgs::Float32::ConstPtr& angle);
void serialActivityTimer(const ros::TimerEvent& e);
void publishRosTopics();
void parseData(string data);

//Globals
geometry_msgs::QuaternionStamped fingerAngle;
geometry_msgs::QuaternionStamped wristAngle;
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
float linearSpeed = 0.;
float yawError = 0.;
const float deltaTime = 0.1;


//PID constants and arrays
float Kpv = 0; //Proportinal Velocity
float Kiv = 0; //Integral Velocity
float Kdv = 0; //Derivative Velocity
int stepV = 0; //keeps track of the point in the array for adding new error each update cycle.
float evArray[1000]; //array of previous error for (arraySize/hz) seconds (error Velocity Array)
float pvError = 0; //previouse velocity error

float Kpy = 0; //Proportinal Yaw   
float Kiy = 0; //Inegral Yaw
float Kdy = 0; //Derivative Yaw
int stepY = 0; //keeps track of the point in the array for adding new error each update cycle.
float eyArray[1000]; //array of previous error for (arraySize/hz) seconds (error Yaw Array)
float pyError = 0; //previouse yaw error


//Publishers
ros::Publisher fingerAnglePublish;
ros::Publisher wristAnglePublish;
ros::Publisher imuPublish;
ros::Publisher odomPublish;
ros::Publisher sonarLeftPublish;
ros::Publisher sonarCenterPublish;
ros::Publisher sonarRightPublish;

//Subscribers
ros::Subscriber velocitySubscriber;
ros::Subscriber fingerAngleSubscriber;
ros::Subscriber wristAngleSubscriber;

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
    
    fingerAnglePublish = aNH.advertise<geometry_msgs::QuaternionStamped>((publishedName + "/fingerAngle/prev_cmd"), 10);
    wristAnglePublish = aNH.advertise<geometry_msgs::QuaternionStamped>((publishedName + "/wristAngle/prev_cmd"), 10);
    imuPublish = aNH.advertise<sensor_msgs::Imu>((publishedName + "/imu"), 10);
    odomPublish = aNH.advertise<nav_msgs::Odometry>((publishedName + "/odom"), 10);
    sonarLeftPublish = aNH.advertise<sensor_msgs::Range>((publishedName + "/sonarLeft"), 10);
    sonarCenterPublish = aNH.advertise<sensor_msgs::Range>((publishedName + "/sonarCenter"), 10);
    sonarRightPublish = aNH.advertise<sensor_msgs::Range>((publishedName + "/sonarRight"), 10);
    
    velocitySubscriber = aNH.subscribe((publishedName + "/velocity"), 10, cmdHandler);
    fingerAngleSubscriber = aNH.subscribe((publishedName + "/fingerAngle/cmd"), 1, fingerAngleHandler);
    wristAngleSubscriber = aNH.subscribe((publishedName + "/wristAngle/cmd"), 1, wristAngleHandler);
    
    publishTimer = aNH.createTimer(ros::Duration(deltaTime), serialActivityTimer);
    
    imu.header.frame_id = publishedName+"/base_link";
    
    odom.header.frame_id = publishedName+"/odom";
    odom.child_frame_id = publishedName+"/base_link";

    ros::spin();
    
    for (int i = 0; i < 1000; i++)
    {
    evArray[i] = 0;
    eyArray[i] = 0;
    }
    
    return EXIT_SUCCESS;
}


void cmdHandler(const geometry_msgs::Twist::ConstPtr& message) {
    // remove artificial factor that was multiplied for simulation. this scales it back down to -1.0 to +1.0
  linearSpeed = (message->linear.x); // / 1.5;
  yawError = (message->angular.z); // / 8;
  
  
  //PID Code {
  float sat = 255; //Saturation point
  
  //Velocity--------------------------
  float xVel = odom.twist.twist.linear.x;
  float yVel = odom.twist.twist.linear.y;
  float vel = sqrt(xVel*xVel + yVel*yVel);
  float velError = linearSpeed - vel; //calculate the error
  float IV = 0;
  
  //Propotinal
  float PV = Kpv * velError; 
  if (PV > sat) //limit the max and minimum output of proportinal
  PV = sat;
  if (PV < -sat)
  PV= -sat;
  
  //Integral
  if (velError > 1 || velError < -1) //only use integral when error is larger than presumed noise.
  {
    evArray[stepV] = velError; //add error into the error Array.
    stepV++;
    
    if (stepV >= 1000)
    stepV = 0;
    
    float sumV= 0;
    for (int i= 0; i < 1000; i++) //sum the array to get the error over time from t = 0 to present.
    {
        sumV += evArray[i];
    }
    
    IV = Kiv * sumV;
   }  
   
    //anti windup                             //if PV is already commanding greater than half power dont use the integrel
    if (fabs(IV) > sat/2 || fabs(PV) > sat/2) //reset the integral to 0 if it hits its cap of half max power
    {
       for (int i= 0; i < 1000; i++)
        {
           evArray[i] = 0;
        }            
        IV = 0;
    }

    float DV = 0;
    if (!(fabs(PV) > sat/2)) 
    {
    //Derivative
       DV = Kdv * (velError - pvError) * 10; //10 being the frequency of the system giving us a one second prediction base.
    }
    pvError = velError; //set previouse error to current error 
    
    float velOut = PV + IV + DV;
    if (velOut > sat) //cap vel command
    {
        velOut = sat;
    }
    else if (velOut < -sat)
    {
        velOut = -sat;
    }
  
    
  //Yaw-----------------------------
  float IY = 0;
    
  //Propotinal
  float PY = Kpv * yawError; 
  if (PY > sat) //limit the max and minimum output of proportinal
  PY = sat;
  if (PY < -sat)
  PY= -sat;
  
  //Integral
  if (yawError > 0.1 || yawError < -0.1) //only use integral when error is larger than presumed noise.
  {
    eyArray[stepY] = yawError; //add error into the error Array.
    stepY++;
    
    if (stepY >= 1000)
    stepY = 0;
    
    float sumY= 0;
    for (int i= 0; i < 1000; i++) //sum the array to get the error over time from t = 0 to present.
    {
        sumY += eyArray[i];
    }
    
    IY = Kiy * sumY;
   } 
   
    //anti windup                                     //if PV is already commanding greater than half power dont use the integrel;
    if (fabs(IY) > sat/2 || fabs(PY) > sat/2) //reset the integral to 0 if it hits its cap
    {
       for (int i= 0; i < 1000; i++)
        {
           eyArray[i] = 0;
        }            
        IY = 0;
    }

    float DY = 0;
    if (!(fabs(PY) > sat/2)) 
    {
    //Derivative
       DY = Kdy * (yawError - pyError) * 10; //10 being the frequency of the system giving us a one second prediction base.
    }
    pyError = yawError;
    
    float yawOut = PY + IY + DY;
    if (yawOut > sat) //cap yaw command
    {
        yawOut = sat;
    }
    else if (yawOut < -sat)
    {
        yawOut = -sat;
    }
    // } end PID
   
   int left = velOut + yawOut;
   int right = velOut - yawOut;
   
   if (left > sat)  {left = sat;}
   if (left < -sat) {left = -sat;}
   if (right > sat) {right = sat;}
   if (right < -sat){right = -sat;}
    
    sprintf(moveCmd, "v,%d%d\n", left, right);
    usb.sendData(moveCmd);    
    memset(&moveCmd, '\0', sizeof (moveCmd));
}


// The finger and wrist handlers receive gripper angle commands in floating point
// radians, write them to a string and send that to the arduino
// for processing.
void fingerAngleHandler(const std_msgs::Float32::ConstPtr& angle) {
  char cmd[16]={'\0'};

  // Avoid dealing with negative exponents which confuse the conversion to string by checking if the angle is small
  if (angle->data < 0.01) {
    // 'f' indicates this is a finger command to the arduino
    sprintf(cmd, "f,0\n");
  } else {
    sprintf(cmd, "f,%.4g\n", angle->data);
  }
  usb.sendData(cmd);
  memset(&cmd, '\0', sizeof (cmd));
}

void wristAngleHandler(const std_msgs::Float32::ConstPtr& angle) {
    char cmd[16]={'\0'};

    // Avoid dealing with negative exponents which confuse the conversion to string by checking if the angle is small
  if (angle->data < 0.01) {
    // 'w' indicates this is a wrist command to the arduino
    sprintf(cmd, "w,0\n");
  } else {
    sprintf(cmd, "w,%.4g\n", angle->data);
  }
  usb.sendData(cmd);
  memset(&cmd, '\0', sizeof (cmd));
}

void serialActivityTimer(const ros::TimerEvent& e) {
    usb.sendData(dataCmd);
    parseData(usb.readData());
    publishRosTopics();
}

void publishRosTopics() {
    fingerAnglePublish.publish(fingerAngle);
    wristAnglePublish.publish(wristAngle);
    imuPublish.publish(imu);
    odomPublish.publish(odom);
    sonarLeftPublish.publish(sonarLeft);
    sonarCenterPublish.publish(sonarCenter);
    sonarRightPublish.publish(sonarRight);
}

void parseData(string str) {
    istringstream oss(str);
    string sentence;
    
    while (getline(oss, sentence, '\n')) {
		istringstream wss(sentence);
		string word;

		vector<string> dataSet;
		while (getline(wss, word, ',')) {
			dataSet.push_back(word);
		}

		if (dataSet.size() >= 3 && dataSet.at(1) == "1") {

			if (dataSet.at(0) == "GRF") {
				fingerAngle.header.stamp = ros::Time::now();
				fingerAngle.quaternion = tf::createQuaternionMsgFromRollPitchYaw(atof(dataSet.at(2).c_str()), 0.0, 0.0);
			}
			else if (dataSet.at(0) == "GRW") {
				wristAngle.header.stamp = ros::Time::now();
				wristAngle.quaternion = tf::createQuaternionMsgFromRollPitchYaw(atof(dataSet.at(2).c_str()), 0.0, 0.0);
			}
			else if (dataSet.at(0) == "IMU") {
				imu.header.stamp = ros::Time::now();
				imu.linear_acceleration.x = atof(dataSet.at(2).c_str());
				imu.linear_acceleration.y = atof(dataSet.at(3).c_str());
				imu.linear_acceleration.z = atof(dataSet.at(4).c_str());
				imu.angular_velocity.x = atof(dataSet.at(5).c_str());
				imu.angular_velocity.y = atof(dataSet.at(6).c_str());
				imu.angular_velocity.z = atof(dataSet.at(7).c_str());
				imu.orientation = tf::createQuaternionMsgFromRollPitchYaw(atof(dataSet.at(8).c_str()), atof(dataSet.at(9).c_str()), atof(dataSet.at(10).c_str()));
			}
			else if (dataSet.at(0) == "ODOM") {
				odom.header.stamp = ros::Time::now();
				odom.pose.pose.position.x += atof(dataSet.at(2).c_str()) / 100.0;
				odom.pose.pose.position.y += atof(dataSet.at(3).c_str()) / 100.0;
				odom.pose.pose.position.z = 0.0;
				odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(atof(dataSet.at(4).c_str()));
				odom.twist.twist.linear.x = atof(dataSet.at(5).c_str()) / 100.0;
				odom.twist.twist.linear.y = atof(dataSet.at(6).c_str()) / 100.0;
				odom.twist.twist.angular.z = atof(dataSet.at(7).c_str());
			}
			else if (dataSet.at(0) == "USL") {
				sonarLeft.header.stamp = ros::Time::now();
				sonarLeft.range = atof(dataSet.at(2).c_str()) / 100.0;
			}
			else if (dataSet.at(0) == "USC") {
				sonarCenter.header.stamp = ros::Time::now();
				sonarCenter.range = atof(dataSet.at(2).c_str()) / 100.0;
			}
			else if (dataSet.at(0) == "USR") {
				sonarRight.header.stamp = ros::Time::now();
				sonarRight.range = atof(dataSet.at(2).c_str()) / 100.0;
			}

		}
	}
}
