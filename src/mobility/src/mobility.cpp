#include <ros/ros.h>

//ROS libraries
#include <angles/angles.h>
#include <random_numbers/random_numbers.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

//ROS messages
#include <std_msgs/Float32.h>
#include <std_msgs/Int16.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/Range.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <apriltags_ros/AprilTagDetectionArray.h>

// To handle shutdown signals so the node quits properly in response to "rosnode kill"
#include <ros/ros.h>
#include <signal.h>

using namespace std;

//Random number generator
random_numbers::RandomNumberGenerator* rng;	

//Mobility Logic Functions
void setVelocity(double linearVel, double angularVel);
void simP(double linearVel, double angularVel);
void openFingers(); // Open fingers to 90 degrees
void closeFingers();// Close fingers to 0 degrees
void raiseWrist();  // Return wrist back to 0 degrees
void lowerWrist();  // Lower wrist to 50 degrees

//Numeric Variables
geometry_msgs::Pose2D currentLocation;
geometry_msgs::Pose2D goalLocation;
int currentMode = 0;
float mobilityLoopTimeStep = 0.1; //time between the mobility loop calls
float status_publish_interval = 1;
float killSwitchTimeout = 10;
bool targetDetected = false;
bool targetCollected = false;

// state machine states
#define STATE_MACHINE_TRANSFORM	0
#define STATE_MACHINE_ROTATE	1
#define STATE_MACHINE_TRANSLATE	2
int stateMachineState = STATE_MACHINE_TRANSFORM;

geometry_msgs::Twist velocity;
char host[128];
string publishedName;
char prev_state_machine[128];

//Publishers
ros::Publisher velocityPublish;
ros::Publisher stateMachinePublish;
ros::Publisher status_publisher;
ros::Publisher fingerAnglePublish;
ros::Publisher wristAnglePublish;
ros::Publisher infoLogPublisher;
ros::Publisher physVelocityPublish;

//Subscribers
ros::Subscriber joySubscriber;
ros::Subscriber modeSubscriber;
ros::Subscriber targetSubscriber;
ros::Subscriber obstacleSubscriber;
ros::Subscriber odometrySubscriber;


//Timers
ros::Timer stateMachineTimer;
ros::Timer publish_status_timer;
ros::Timer killSwitchTimer;
ros::Timer targetDetectedTimer;
time_t start; //records time for delays in sequanced actions, 1 second resolution.
float tDiff = 0;

//Transforms
tf::TransformListener *tfListener;

// OS Signal Handler
void sigintEventHandler(int signal);

//Callback handlers
void joyCmdHandler(const sensor_msgs::Joy::ConstPtr& message);
void modeHandler(const std_msgs::UInt8::ConstPtr& message);
void targetHandler(const apriltags_ros::AprilTagDetectionArray::ConstPtr& tagInfo);
void obstacleHandler(const std_msgs::UInt8::ConstPtr& message);
void odometryHandler(const nav_msgs::Odometry::ConstPtr& message);
void mobilityStateMachine(const ros::TimerEvent&);
void publishStatusTimerEventHandler(const ros::TimerEvent& event);
void killSwitchTimerEventHandler(const ros::TimerEvent& event);
void targetDetectedReset(const ros::TimerEvent& event);


int main(int argc, char **argv) {

    gethostname(host, sizeof (host));
    string hostname(host);

    rng = new random_numbers::RandomNumberGenerator(); //instantiate random number generator
    goalLocation.theta = rng->uniformReal(0, 2 * M_PI); //set initial random heading

    //select initial search position 50 cm from center (0,0)
	goalLocation.x = 0.5 * cos(goalLocation.theta);
	goalLocation.y = 0.5 * sin(goalLocation.theta);

    if (argc >= 2) {
        publishedName = argv[1];
        cout << "Welcome to the world of tomorrow " << publishedName << "!  Mobility module started." << endl;
    } else {
        publishedName = hostname;
        cout << "No Name Selected. Default is: " << publishedName << endl;
    }

    // NoSignalHandler so we can catch SIGINT ourselves and shutdown the node
    ros::init(argc, argv, (publishedName + "_MOBILITY"), ros::init_options::NoSigintHandler);
    ros::NodeHandle mNH;

    signal(SIGINT, sigintEventHandler); // Register the SIGINT event handler so the node can shutdown properly

    joySubscriber = mNH.subscribe((publishedName + "/joystick"), 10, joyCmdHandler);
    modeSubscriber = mNH.subscribe((publishedName + "/mode"), 1, modeHandler);
    targetSubscriber = mNH.subscribe((publishedName + "/targets"), 10, targetHandler);
    obstacleSubscriber = mNH.subscribe((publishedName + "/obstacle"), 10, obstacleHandler);
    odometrySubscriber = mNH.subscribe((publishedName + "/odom/filtered"), 10, odometryHandler);

    status_publisher = mNH.advertise<std_msgs::String>((publishedName + "/status"), 1, true);
    velocityPublish = mNH.advertise<geometry_msgs::Twist>((publishedName + "/velocity"), 10);
    stateMachinePublish = mNH.advertise<std_msgs::String>((publishedName + "/state_machine"), 1, true);
    fingerAnglePublish = mNH.advertise<std_msgs::Float32>((publishedName + "/fingerAngle/cmd"), 1, true);
    wristAnglePublish = mNH.advertise<std_msgs::Float32>((publishedName + "/wristAngle/cmd"), 1, true);
    infoLogPublisher = mNH.advertise<std_msgs::String>("/infoLog", 1, true);
    physVelocityPublish = mNH.advertise<geometry_msgs::Twist>((publishedName + "/physVelocity"), 10);

    publish_status_timer = mNH.createTimer(ros::Duration(status_publish_interval), publishStatusTimerEventHandler);
    //killSwitchTimer = mNH.createTimer(ros::Duration(killSwitchTimeout), killSwitchTimerEventHandler);
    stateMachineTimer = mNH.createTimer(ros::Duration(mobilityLoopTimeStep), mobilityStateMachine);
    targetDetectedTimer = mNH.createTimer(ros::Duration(0), targetDetectedReset, true);
    
    tfListener = new tf::TransformListener();

    std_msgs::String msg;
    msg.data = "Log Started";
    infoLogPublisher.publish(msg);
    ros::spin();
start = time(0);
    
    return EXIT_SUCCESS;
}

void mobilityStateMachine(const ros::TimerEvent&) {
    std_msgs::String stateMachineMsg;
    std_msgs::String msg;
    
    if (currentMode == 2 || currentMode == 3) { //Robot is in automode


		switch(stateMachineState) {
			
			//Select rotation or translation based on required adjustment
			//If no adjustment needed, select new goal
			case STATE_MACHINE_TRANSFORM: {
				stateMachineMsg.data = "TRANSFORMING";
				//If angle between current and goal is significant
				if (fabs(angles::shortest_angular_distance(currentLocation.theta, goalLocation.theta)) > 0.8) //if error in heading is greater than 0.8 radians
				{
					stateMachineState = STATE_MACHINE_ROTATE; //rotate
				}
				//If goal has not yet been reached
				else if (fabs(angles::shortest_angular_distance(currentLocation.theta, atan2(goalLocation.y - currentLocation.y, goalLocation.x - currentLocation.x))) < M_PI_2) //needs explenation
				{
					stateMachineState = STATE_MACHINE_TRANSLATE; //translate
				}
				//If returning with a target
				else if (targetCollected) {
					//If goal has not yet been reached
					if (hypot(0.0 - currentLocation.x, 0.0 - currentLocation.y) > 0.5) {
				        //set angle to center as goal heading
						goalLocation.theta = M_PI + atan2(currentLocation.y, currentLocation.x);
						
						//set center as goal position
						goalLocation.x = 0.0;
						goalLocation.y = 0.0;
					}
					//Otherwise, drop off target and select new random uniform heading
					else {
						//open fingers
						std_msgs::Float32 angle;
						angle.data = M_PI_2;
						fingerAnglePublish.publish(angle);
						
						//reset flag
						targetCollected = false;
						
						goalLocation.theta = rng->uniformReal(0, 2 * M_PI);
					}
				}
				//If no targets have been detected, assign a new goal
				else if (!targetDetected) {

					//select new heading from Gaussian distribution around current heading
					goalLocation.theta = rng->gaussian(currentLocation.theta, 0.25);
					
					//select new position 50 cm from current location
					goalLocation.x = currentLocation.x + (0.5 * cos(goalLocation.theta));
					goalLocation.y = currentLocation.y + (0.5 * sin(goalLocation.theta));
				}
				
				//Purposefully fall through to next case without breaking
			}
			
			//Calculate angle between currentLocation.theta and goalLocation.theta
			//Rotate left or right depending on sign of angle
			//Stay in this state until angle is minimized
			case STATE_MACHINE_ROTATE: {
				stateMachineMsg.data = "ROTATING";
				float errorYaw = angles::shortest_angular_distance(currentLocation.theta, goalLocation.theta); //calculate the diffrence between current heading and desired heading.
				
			    if (fabs(angles::shortest_angular_distance(currentLocation.theta, goalLocation.theta)) > 0.8) //if angle is greater than 0.8 radians rotate but dont drive forward.
			    {		
					setVelocity(0.0, errorYaw); //rotate but dont drive
					break;
			    }
				else {
					stateMachineState = STATE_MACHINE_TRANSLATE; //move to translate step
					//fall through on purpose.
				}

			}
			
			//Calculate angle between currentLocation.x/y and goalLocation.x/y
			//Drive forward
			//Stay in this state until angle is at least PI/2
			case STATE_MACHINE_TRANSLATE: {
				stateMachineMsg.data = "TRANSLATING";
				float errorYaw = angles::shortest_angular_distance(currentLocation.theta, goalLocation.theta); //calculate the distance between current heading and desired heading.
				
				//goal not yet reached drive while maintaining proper heading.
				if (fabs(angles::shortest_angular_distance(currentLocation.theta, atan2(goalLocation.y - currentLocation.y, goalLocation.x - currentLocation.x))) < M_PI_2) {
					setVelocity(4.0, errorYaw); //drive and turn simultaniously
				}
				else if (fabs(angles::shortest_angular_distance(currentLocation.theta, goalLocation.theta)) > 0.1) //goal is reached but desired heading is still wrong turn only
				{
					setVelocity(0.0, errorYaw); //rotate but dont drive
			    }
			    else
				{
					setVelocity(0.0, 0.0); //stop
					
					//close fingers
					std_msgs::Float32 angle;
					angle.data = 0;
					fingerAnglePublish.publish(angle);
					
					stateMachineState = STATE_MACHINE_TRANSFORM; //move back to transform step
				}
			    break;
			}
		
			default: {
			    break;
			}
		}
	}

    else { // mode is NOT auto

        // publish current state for the operator to see
        stateMachineMsg.data = "WAITING";
    }

    // publish state machine string for user, only if it has changed, though
    if (strcmp(stateMachineMsg.data.c_str(), prev_state_machine) != 0) {
        stateMachinePublish.publish(stateMachineMsg);
        sprintf(prev_state_machine, "%s", stateMachineMsg.data.c_str());
    }
}

void setVelocity(double linearVel, double angularVel) 
{
  // Stopping and starting the timer causes it to start counting from 0 again.
  // As long as this is called before the kill swith timer reaches killSwitchTimeout seconds
  // the rover's kill switch wont be called.
  killSwitchTimer.stop();
  killSwitchTimer.start();

  velocity.linear.x = linearVel, // * 1.5;
  velocity.angular.z = angularVel; // * 8; //scaling factor for sim; removed by aBridge node
  physVelocityPublish.publish(velocity);



  simP(linearVel,angularVel);
  
}

/***********************
 * ROS CALLBACK HANDLERS
 ************************/

void targetHandler(const apriltags_ros::AprilTagDetectionArray::ConstPtr& message) {

        // If in manual mode do not try to automatically pick up the target
        if (currentMode == 1) return;
 
	if (message->detections.size() > 0) {
		
		geometry_msgs::PoseStamped tagPose = message->detections[0].pose;
		
		//if target is close enough
		if (hypot(hypot(tagPose.pose.position.x, tagPose.pose.position.y), tagPose.pose.position.z) < 0.2) {
			//assume target has been picked up by gripper
			targetCollected = true;
			
			//lower wrist to avoid ultrasound sensors
			std_msgs::Float32 angle;
			angle.data = M_PI_2/4;
			wristAnglePublish.publish(angle);
		}
		
		else {
			tagPose.header.stamp = ros::Time(0);
			geometry_msgs::PoseStamped odomPose;

			try {
				tfListener->waitForTransform(publishedName + "/odom", publishedName + "/camera_link", ros::Time(0), ros::Duration(1.0));
				tfListener->transformPose(publishedName + "/odom", tagPose, odomPose);
			}

			catch(tf::TransformException& ex) {
				ROS_INFO("Received an exception trying to transform a point from \"odom\" to \"camera_link\": %s", ex.what());
			}

			//if this is the goal target
			if (message->detections[0].id == 256) {
				//open fingers to drop off target
				std_msgs::Float32 angle;
				angle.data = M_PI_2;
				fingerAnglePublish.publish(angle);
			}

			//Otherwise, if no target has been collected, set target pose as goal
			else if (!targetCollected) {
				//set goal heading
				goalLocation.theta = atan2(odomPose.pose.position.y - currentLocation.y, odomPose.pose.position.x - currentLocation.x);
				
				//set goal position
				goalLocation.x = odomPose.pose.position.x - (0.26 * cos(goalLocation.theta));
				goalLocation.y = odomPose.pose.position.y - (0.26 * sin(goalLocation.theta));
				
				//set gripper
				std_msgs::Float32 angle;
				//open fingers
				angle.data = M_PI_2;
				fingerAnglePublish.publish(angle);
				//lower wrist
				angle.data = 0.8;
				wristAnglePublish.publish(angle);
				
				//set state and timeout
				targetDetected = true;
				targetDetectedTimer.setPeriod(ros::Duration(5.0));
				targetDetectedTimer.start();
				
				//switch to transform state to trigger return to center
				stateMachineState = STATE_MACHINE_TRANSFORM;
			}
		}
	}
}

void modeHandler(const std_msgs::UInt8::ConstPtr& message) {
	currentMode = message->data;
	setVelocity(0.0, 0.0);
}

void obstacleHandler(const std_msgs::UInt8::ConstPtr& message) {
	if (!targetDetected && (message->data > 0)) {
		//obstacle on right side
		if (message->data == 1) {
			//select new heading 0.2 radians to the left
			goalLocation.theta = currentLocation.theta + 0.2;
		}
		
		//obstacle in front or on left side
		else if (message->data == 2) {
			//select new heading 0.2 radians to the right
			goalLocation.theta = currentLocation.theta - 0.2;
		}
							
		//select new position 50 cm from current location
		goalLocation.x = currentLocation.x + (0.5 * cos(goalLocation.theta));
		goalLocation.y = currentLocation.y + (0.5 * sin(goalLocation.theta));
		
		//switch to transform state to trigger collision avoidance
		stateMachineState = STATE_MACHINE_TRANSFORM;
	}
}

void odometryHandler(const nav_msgs::Odometry::ConstPtr& message) {
	//Get (x,y) location directly from pose
	currentLocation.x = message->pose.pose.position.x;
	currentLocation.y = message->pose.pose.position.y;
	
	//Get theta rotation by converting quaternion orientation to pitch/roll/yaw
	tf::Quaternion q(message->pose.pose.orientation.x, message->pose.pose.orientation.y, message->pose.pose.orientation.z, message->pose.pose.orientation.w);
	tf::Matrix3x3 m(q);
	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);
	currentLocation.theta = yaw;
}

void joyCmdHandler(const sensor_msgs::Joy::ConstPtr& message) {
	if (currentMode == 0 || currentMode == 1) {
		setVelocity(abs(message->axes[4]) >= 0.1 ? message->axes[4] : 0, abs(message->axes[3]) >= 0.1 ? message->axes[3] : 0);
	} 
}


void publishStatusTimerEventHandler(const ros::TimerEvent&)
{
  std_msgs::String msg;
  msg.data = "online";
  status_publisher.publish(msg);
}

// Safety precaution. No movement commands - might have lost contact with ROS. Stop the rover.
// Also might no longer be receiving manual movement commands so stop the rover.
void killSwitchTimerEventHandler(const ros::TimerEvent& t)
{
  // No movement commands for killSwitchTime seconds so stop the rover 
  setVelocity(0,0);
  double current_time = ros::Time::now().toSec();
  ROS_INFO("In mobility.cpp:: killSwitchTimerEventHander(): Movement input timeout. Stopping the rover at %6.4f.", current_time);
}

void targetDetectedReset(const ros::TimerEvent& event) {
	targetDetected = false;
	
	std_msgs::Float32 angle;
	angle.data = 0;
	fingerAnglePublish.publish(angle); //close fingers
	wristAnglePublish.publish(angle); //raise wrist
}

void sigintEventHandler(int sig)
{
     // All the default sigint handler does is call shutdown()
     ros::shutdown();
}

void simP(double linearVel, double angularVel)
{

  int sat = 255;
  int Kpv = 255;
  int Kpa = 200;

  //Propotinal
  float PV = Kpv * linearVel; 
  if (PV > sat) //limit the max and minimum output of proportinal
  PV = sat;
  if (PV < -sat)
  PV= -sat;

  //Propotinal
  float PA = Kpa * angularVel; 
  if (PA > sat) //limit the max and minimum output of proportinal
  PA = sat;
  if (PA < -sat)
  PA= -sat;

   float turn = PA/60;  
   float forward = PV/255-(abs(turn)/5);
   if (linearVel >= 0 && forward <= 0)
   {
   forward = 0;
   }
   if (linearVel <= 0 && forward >= 0)
   {
   forward = 0;
   }


  velocity.linear.x = forward, // * 1.5;
  velocity.angular.z = turn; // * 8; //scaling factor for sim; removed by aBridge node
  velocityPublish.publish(velocity);
}
