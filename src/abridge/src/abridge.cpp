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
#include <std_msgs/UInt8.h>

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
const float deltaTime = 0.1;
int currentMode = 0;


//PID constants and arrays
float linearSpeed = 0.;
float Kpv = 120; //Proportinal Velocity
float Kiv = 20; //Integral Velocity
float Kdv = 15; //Derivative Velocity
float velFF = 0; //velocity feed forward
int stepV = 0; //keeps track of the point in the array for adding new error each update cycle.
float evArray[1000]; //array of previous error for (arraySize/hz) seconds (error Velocity Array)
float velError[4] = {0,0,0,0}; //contains current velocity error and error 4 steps in the past.

float Kpy = 200; //Proportinal Yaw   
float Kiy = 15; //Inegral Yaw
float Kdy = 15; //Derivative Yaw
int stepY = 0; //keeps track of the point in the array for adding new error each update cycle.
float eyArray[1000]; //array of previous error for (arraySize/hz) seconds (error Yaw Array)
float yawError[4] = {0,0,0,0}; //contains current yaw error and error 4 steps in the past.



//Publishers
ros::Publisher fingerAnglePublish;
ros::Publisher wristAnglePublish;
ros::Publisher imuPublish;
ros::Publisher odomPublish;
ros::Publisher sonarLeftPublish;
ros::Publisher sonarCenterPublish;
ros::Publisher sonarRightPublish;
ros::Publisher infoLogPublisher;

//Subscribers
ros::Subscriber physVelocitySubscriber;
ros::Subscriber fingerAngleSubscriber;
ros::Subscriber wristAngleSubscriber;
ros::Subscriber modeSubscriber;

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
    void modeHandler(const std_msgs::UInt8::ConstPtr& message);
    
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
    infoLogPublisher = aNH.advertise<std_msgs::String>("/infoLog", 1, true);
    
    physVelocitySubscriber = aNH.subscribe((publishedName + "/physVelocity"), 10, cmdHandler);
    fingerAngleSubscriber = aNH.subscribe((publishedName + "/fingerAngle/cmd"), 1, fingerAngleHandler);
    wristAngleSubscriber = aNH.subscribe((publishedName + "/wristAngle/cmd"), 1, wristAngleHandler);
    modeSubscriber = aNH.subscribe((publishedName + "/mode"), 1, modeHandler);

    
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
  yawError[0] = (message->angular.z); // / 8;

  float xVel = odom.twist.twist.linear.x;
  float yVel = odom.twist.twist.linear.y;
  float vel = sqrt(xVel*xVel + yVel*yVel);
  float IV = 0;
  float sat = 255; //Saturation point



  if (currentMode == 1)
  {
	yawError[0] *= 255/Kpy;
	velError[0] = linearSpeed * 255/Kpv; //scale values between -255 and 255;
  }
  else
  {
    if (linearSpeed > 0.5) velFF = 255;
    else if (linearSpeed > 0.4) velFF = 180;
    else if (linearSpeed > 0.3) velFF = 120;
    else if (linearSpeed > 0.2) velFF = 50;
    else if (linearSpeed > 0.1) velFF = 25;

    velError[0] = linearSpeed - vel; //calculate the error
  }
  
  
  //PID Code {

  
  //Velocity--------------------------

  
  //Propotinal
  float PV = Kpv * ((velError[0]+velError[1])/2); 
  if (PV > sat) //limit the max and minimum output of proportinal
  PV = sat;
  if (PV < -sat)
  PV= -sat;
  
  //Integral
  if (velError[0] > 0.01 || velError[0] < -0.01) //only use integral when error is larger than presumed noise.
  {
    evArray[stepV] = velError[0]; //add error into the error Array.
    stepV++;
    
    if (stepV >= 1000)
    stepV = 0;
  }
    
    float sumV= 0;
    for (int i= 0; i < 1000; i++) //sum the array to get the error over time from t = 0 to present.
    {
        sumV += evArray[i];
    }
    
    IV = Kiv * sumV;
     
   
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
       DV = Kdv * ((velError[0]+velError[1])/2 - (velError[2]+velError[3])/2) * 10; //10 being the frequency of the system giving us a one second prediction base.
    }
    velError[3] = velError[2];
    velError[2] = velError[1];
    velError[1] = velError[0]; //set previouse error to current error 
    
    float velOut = PV + IV + DV + velFF;
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
  float PY = Kpy * ((yawError[0]+yawError[1])/2); 
  if (PY > sat) //limit the max and minimum output of proportinal
  PY = sat;
  if (PY < -sat)
  PY= -sat;
  
  //Integral
  if (yawError[0] > 0.1 || yawError[0] < -0.1) //only use integral when error is larger than presumed noise.
  {
    eyArray[stepY] = yawError[0]; //add error into the error Array.
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
       DY = Kdy * ((yawError[0]+yawError[1])/2 - (yawError[2]+yawError[3])/2) * 10; //10 being the frequency of the system giving us a one second prediction base.
    }
    yawError[3] = yawError[2];
    yawError[2] = yawError[1];
    yawError[1] = yawError[0]; //set previouse error to current error 
    
    float yawOut = PY + IY + DY;
    if (yawOut > sat) //cap yaw command
    {
        yawOut = sat;
    }
    else if (yawOut < -sat)
    {
        yawOut = -sat;
    }

    if (PV > 0)
	if(velOut < 0) velOut = 0;
    else if(PV < 0)
	if(velOut > 0) velOut = 0;

    if (PY > 0)
	if(yawOut < 0) yawOut = 0;
    else if(PY < 0)
	if(yawOut > 0) yawOut = 0;


    // } end PID

      if (currentMode == 1)
  	{
	yawOut = PY;
	velOut = PV;	
  	}
   
   int left = velOut - yawOut;
   int right = velOut + yawOut;
   
   if (left > sat)  {left = sat;}
   if (left < -sat) {left = -sat;}
   if (right > sat) {right = sat;}
   if (right < -sat){right = -sat;}

   if(linearSpeed == 0 && yawError[0] == 0)
	{
		left = 0;
		right = 0;
	}


	/*stringstream ss;
	ss << "linearSpeed : " << xVel << " error : " << velError[0] << " comSpeed : " << linearSpeed << " IV : " << IV;
	std_msgs::String msg;
	msg.data = ss.str();
    	infoLogPublisher.publish(msg);//good for tunning and bug fixing*/
    
    sprintf(moveCmd, "v,%d,%d\n", left, right);
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



void modeHandler(const std_msgs::UInt8::ConstPtr& message) {
	currentMode = message->data;
}
