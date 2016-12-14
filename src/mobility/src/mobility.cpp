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
geometry_msgs::Pose2D centerLocation;

int currentMode = 0;
float mobilityLoopTimeStep = 0.1; //time between the mobility loop calls
float status_publish_interval = 5;
float killSwitchTimeout = 10;
bool targetDetected = false;
bool targetCollected = false;
bool lockTarget = false;
bool timeOut = false;
bool blockBlock = false;
bool centerSeen = false;
bool dropRoute = false;
bool countDropGuard = false; 
bool approach = false;
double blockDist = 0;
double blockYawError = 0;
bool spinWasTrue = false;
std_msgs::String msg;

// state machine states
#define STATE_MACHINE_TRANSFORM	0
#define STATE_MACHINE_ROTATE	1
#define STATE_MACHINE_TRANSLATE	2
#define STATE_MACHINE_PICKUP    3
#define STATE_MACHINE_DROPOFF   4

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
time_t startupDelay; //records time for delays in sequanced actions, 1 second resolution.
time_t dropCheck;
float tDiff = 0;
float tDiff2 = 0;

boost::posix_time::ptime millTimer;

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

	goalLocation.x = 0.5 * cos(goalLocation.theta+3.1415);
	goalLocation.y = 0.5 * sin(goalLocation.theta+3.1415);

        centerLocation.x = 0;
        centerLocation.y = 0;

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
    startupDelay = time(0);
    
    return EXIT_SUCCESS;
}

void mobilityStateMachine(const ros::TimerEvent&) {
    std_msgs::String stateMachineMsg;



        geometry_msgs::PoseStamped mapOrigin;
	mapOrigin.header.stamp = ros::Time::now();
	mapOrigin.header.frame_id = publishedName + "/map";
	mapOrigin.pose.orientation.w = 1;
	geometry_msgs::PoseStamped odomPose;
	string x = "";

		try {
			tfListener->waitForTransform(publishedName + "/map", publishedName + "/odom", ros::Time::now(), ros::Duration(1.0));
			tfListener->transformPose(publishedName + "/odom", mapOrigin, odomPose);
		}

		catch(tf::TransformException& ex) {
			ROS_INFO("Received an exception trying to transform a point from \"map\" to \"odom\": %s", ex.what());
			x = "Exception thrown " + (string)ex.what();
		}

   stringstream ss;
   ss << "map x : y  " << odomPose.pose.position.x << " : " << odomPose.pose.position.y << " : " << currentLocation.x << " : " << currentLocation.y << " : " << x;
   msg.data = ss.str();
   infoLogPublisher.publish(msg);

    
    if (currentMode == 2 || currentMode == 3) { //Robot is in automode

    tDiff = time(0) - startupDelay; 

	if (!targetCollected && !targetDetected)
		{
		   //set gripper
		   std_msgs::Float32 angle;
		   //open fingers
		   angle.data = M_PI_2;
		   fingerAnglePublish.publish(angle);
		   angle.data = 0;
		   wristAnglePublish.publish(angle); //raise wrist
		}

		switch(stateMachineState) {
			
			//Select rotation or translation based on required adjustment
   
			//If no adjustment needed, select new goal
			case STATE_MACHINE_TRANSFORM: {
				stateMachineMsg.data = "TRANSFORMING";
				if(dropRoute)
				{
					goalLocation.x = currentLocation.x;
					goalLocation.y = currentLocation.y;
					goalLocation.theta = currentLocation.theta;
					if (tDiff > 2)
					 {
					   //reset flag
				  	   targetCollected = false;
				  	   targetDetected = false;
					   lockTarget = false;
					   dropRoute = false;
					   countDropGuard = false;
					   spinWasTrue = false;
					   startupDelay = time(0);
					   setVelocity(0.0,0);
				   	   stateMachineState = STATE_MACHINE_TRANSFORM; //move back to transform step
					 }
					else if (tDiff > 0)
					{
					  //open fingers
					  std_msgs::Float32 angle;
					  angle.data = M_PI_2;
					  fingerAnglePublish.publish(angle);
		   			  angle.data = 0;
		   			  wristAnglePublish.publish(angle); //raise wrist
						
					  setVelocity(-0.3,0.0);
					}
					break;
				}
				//If angle between current and goal is significant
				if (fabs(angles::shortest_angular_distance(currentLocation.theta, goalLocation.theta)) > 0.4) //if error in heading is greater than 0.8 radians
				{
					stateMachineState = STATE_MACHINE_ROTATE; //rotate
				}
				//If goal has not yet been reached
				else if (fabs(angles::shortest_angular_distance(currentLocation.theta, atan2(goalLocation.y - currentLocation.y, goalLocation.x - currentLocation.x))) < M_PI_2)
				{
					stateMachineState = STATE_MACHINE_TRANSLATE; //translate
				}
				//If returning with a target
				else if (targetCollected && !centerSeen && !dropRoute) {
					//If goal has not yet been reached
					if (hypot(centerLocation.x - currentLocation.x, centerLocation.y - currentLocation.y) > 0.3) {
				        //set angle to center as goal heading
						goalLocation.theta = atan2(centerLocation.y - currentLocation.y, centerLocation.x - currentLocation.x);
						
						//set center as goal position
						goalLocation.x = centerLocation.x;
						goalLocation.y = centerLocation.y;
						stateMachineState = STATE_MACHINE_ROTATE;
						//spinWasTrue = true; only turn on for random walk to center
					}
					else //spin search for center
					{
					bool randomWalkToCenter = false;
					if (randomWalkToCenter)
					{
					spinWasTrue = true;
					//select new heading to the left to spin and look for the center.
					goalLocation.theta = rng->gaussian(currentLocation.theta, 0.45)-0.05;
					
					//select new position 30 cm from current location
					centerLocation.x = currentLocation.x + 0.3 * cos(goalLocation.theta);
					centerLocation.y = currentLocation.y + 0.3 * sin(goalLocation.theta);
					goalLocation.x = centerLocation.x;
					goalLocation.y = centerLocation.y;
					stateMachineState = STATE_MACHINE_ROTATE;
					}
					else
					{
					 goalLocation.theta = currentLocation.theta + 0.3;
					 centerLocation.x = currentLocation.x + 0.1 * cos(goalLocation.theta);
					 centerLocation.y = currentLocation.y + 0.1 * sin(goalLocation.theta);
					 goalLocation.x = centerLocation.x;
					 goalLocation.y = centerLocation.y;
					 stateMachineState = STATE_MACHINE_ROTATE;					
					}
					}
				}
					//Otherwise, drop off target and select new random uniform heading
				//If no targets have been detected, assign a new goal
				else if (!targetDetected && tDiff > 5) {

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
				float errorYaw = angles::shortest_angular_distance(currentLocation.theta, goalLocation.theta); //calculate the diffrence between current and desired heading in radians.
				
			    if (fabs(angles::shortest_angular_distance(currentLocation.theta, goalLocation.theta)) > 0.4) //if angle is greater than 0.8 radians rotate but dont drive forward.
			    {		
					setVelocity(0.05, errorYaw); //rotate but dont drive
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
				float errorYaw = angles::shortest_angular_distance(currentLocation.theta, goalLocation.theta); //calculate the distance between current and desired heading in radians
				//goal not yet reached drive while maintaining proper heading.
				if (fabs(angles::shortest_angular_distance(currentLocation.theta, atan2(goalLocation.y - currentLocation.y, goalLocation.x - currentLocation.x))) < M_PI_2) 
				{
					setVelocity(0.15, errorYaw/2); //drive and turn simultaniously
				}
				else if (fabs(angles::shortest_angular_distance(currentLocation.theta, goalLocation.theta)) > 0.1) //goal is reached but desired heading is still wrong turn only
				{
					setVelocity(0.0, errorYaw); //rotate but dont drive
			        }
			   	else
				{
					setVelocity(0.0, 0.0); //stop
					
					stateMachineState = STATE_MACHINE_TRANSFORM; //move back to transform step

				}
			    break;
			}
			case STATE_MACHINE_PICKUP: {
			//this is a blocker to prevent transform, rotate, and translate from operating
			//however you can put any code you like in here
			break;
			}
			case STATE_MACHINE_DROPOFF: {
			if (!centerSeen && countDropGuard)
			{
			  dropRoute = true;
			  centerLocation.x = currentLocation.x; //we are sitting on top of the circle so set the center as our location.
			  centerLocation.y = currentLocation.y;
			  stateMachineState = STATE_MACHINE_TRANSFORM;
			  startupDelay = time(0);
			  goalLocation.x = currentLocation.x;
			  goalLocation.y = currentLocation.y;
			  goalLocation.theta = currentLocation.theta;
			  approach = false;
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
        if (currentMode == 1 || currentMode == 0) return;


    //if a target is detected
	if (message->detections.size() > 0 && !dropRoute)
	{
	  centerSeen = false;
	  double count = 0;
	  bool right = false;
	  bool left = false;
	  for (int i = 0; i < message->detections.size(); i++) //this loop is to get the number of center tags
	  {
       	   if (message->detections[i].id == 256) 
       	   {
	     geometry_msgs::PoseStamped cenPose = message->detections[i].pose;
	     if (cenPose.pose.position.x + 0.020 > 0) //checks if tag is on the right or left side of the image
	     {
		right = true;
	     }
	     else
	     {
		left = true;
	     }
             centerSeen = true;
	     count++;
	   }
	  }

	  if (!approach || !countDropGuard) dropCheck = time(0);
	
	  if (centerSeen && targetCollected) //if we have a target and the center is located drive towards it.
	  {
		float mod = 1;
		//if (countDropGuard) mod = -1; //reverse tag rejection when we have seen enough tags that we are on a trajectory in to the square we dont want to follow an edge.
		if (left && right) setVelocity(0.15, 0.0); //otherwise turn till tags on both sides of image then drive straight
		else if (right) setVelocity(0.15, -0.15*mod);
		else setVelocity(0.15, 0.15*mod);

		if (count > 20) //must see greater than this many tags before assuming we are driving into the center and not along an edge.
		{
		countDropGuard = true; //we have driven far enough forward to be in the circle.
		dropCheck = time(0);
		}

		tDiff2 = time(0) - dropCheck; //time since we dropped below countGuard tags

		if (count < 20 && countDropGuard && tDiff2 > 0) centerSeen = false; //we have driven far enough forward to have passed over the circle.
		stateMachineState = STATE_MACHINE_DROPOFF; //go to dropoff mode to prevent velocity overrides.
		approach = true;
		
	  }
	  else if (count > 10 && centerSeen) //reset center location if driving around and its seen.
	  {
		centerLocation.x = currentLocation.x + (0.5 * cos(currentLocation.theta));
		centerLocation.y = currentLocation.y + (0.5 * sin(currentLocation.theta));
	  }
	  if (centerSeen) 
	  {
		if (!targetCollected) //if you want to drive around the center diffrently mod here
		{
		  stateMachineState = STATE_MACHINE_TRANSFORM;
		  if (right)
		  {
		  goalLocation.theta += 0.15; //turn away from the center to the left if just driving around/searching.
		  }
		  else
		  {
		  goalLocation.theta -= 0.15; //turn away from the center to the right if just driving around/searching.
		  }
		  double tmpDist = hypot(goalLocation.x - currentLocation.x, goalLocation.y - currentLocation.y);
		  goalLocation.x = currentLocation.x + (tmpDist * cos(goalLocation.theta));
		  goalLocation.y = currentLocation.y + (tmpDist * sin(goalLocation.theta));
		}
		targetDetected = false;		
		return;
	  }	  	
	}
	else if (approach)
 	{
	  tDiff2 = time(0) - dropCheck;
	  if (tDiff2 > 4) 
	  {
	   stateMachineState = STATE_MACHINE_TRANSFORM;
	   countDropGuard = false;
	   approach = false;
	  }
  	 }


 
	if (message->detections.size() > 0 && !targetCollected && tDiff > 5) 
	{

	targetDetected = true;
	stateMachineState = STATE_MACHINE_PICKUP;
		
		double closest = 100; 
		int target  = 0;
		for (int i = 0; i < message->detections.size(); i++) //this loop selects the closest visible block to makes goals for it
		{
		  geometry_msgs::PoseStamped tagPose = message->detections[i].pose;
		  double test = hypot(hypot(tagPose.pose.position.x, tagPose.pose.position.y), tagPose.pose.position.z); //absolute distance to block from camera lense

		  if (closest > test)
		   {
		     target = i;
		     closest = test;
		     blockDist = hypot(tagPose.pose.position.z, tagPose.pose.position.y); //distance from bottom center of chassis ignoring height.
		     blockDist = sqrt(blockDist*blockDist - 0.195*0.195);
		     blockYawError = atan((tagPose.pose.position.x + 0.020)/blockDist)*1.05; //angle to block from bottom center of chassis on the horizontal.
		   }
		}
		if ( blockYawError > 10) blockYawError = 10; //limits block angle error to prevent overspeed from PID.
		if ( blockYawError < - 10) blockYawError = -10;


		
		geometry_msgs::PoseStamped tagPose = message->detections[target].pose;
		
		//if target is close enough
		boost::posix_time::time_duration Td = boost::posix_time::microsec_clock::local_time() - millTimer; //diffrence between current time and millisecond time
		if (hypot(hypot(tagPose.pose.position.x, tagPose.pose.position.y), tagPose.pose.position.z) < 0.2 && Td.total_milliseconds() < 3800) {
			//assume target has been picked up by gripper
			targetCollected = true;
			stateMachineState = STATE_MACHINE_TRANSFORM;

			goalLocation.theta = atan2(centerLocation.y - currentLocation.y, centerLocation.x - currentLocation.x);
						
			//set center as goal position
			goalLocation.x = centerLocation.x;
			goalLocation.y = centerLocation.y;
			
			//lower wrist to avoid ultrasound sensors
			std_msgs::Float32 angle;
			angle.data = 0.8;
			wristAnglePublish.publish(angle);
			setVelocity(0.0,0);
			return;
		}

		//Otherwise, if no target has been collected, set target pose as goal
		else if (!lockTarget) 
		{

		   //set gripper
		   std_msgs::Float32 angle;
		   //open fingers
		   angle.data = M_PI_2;
		   fingerAnglePublish.publish(angle);
		   //lower wrist
		   angle.data = 1.3;
		   wristAnglePublish.publish(angle);
		}
	}
	
	if (targetDetected && !targetCollected)
	{
		if (!timeOut) millTimer = boost::posix_time::microsec_clock::local_time(); // millisecond time = current time if not in a counting state

		boost::posix_time::time_duration Td = boost::posix_time::microsec_clock::local_time() - millTimer; //diffrence between current time and millisecond time

		if (!(message->detections.size() > 0) && !lockTarget) //if not targets detected and a target has not been locked in
		{
		   if(!timeOut) //if not in a counting state
		   {
		     setVelocity(0.0,0.0);
		     timeOut = true;
		   }
		   else if (Td.total_milliseconds() > 1000 && Td.total_milliseconds() < 2500) //if in a counting state and has been counting for 1 second
		   {
		     setVelocity(-0.2,0.0);
		   }
		}
		else if (blockDist > 0.25 && !lockTarget) //if a target is detected but not locked, and not too close.
		{
		  float vel = blockDist * 0.20;
		  if (vel < 0.1) vel = 0.1;
		  if (vel > 0.2) vel = 0.2;
		  setVelocity(vel,-blockYawError/2);
		  timeOut = false;
		}
		else if (!lockTarget) //if a target hasn't been locked lock it and enter a counting state while slowly driving forward.
		{
		  lockTarget = true;
		  setVelocity(0.18,0);
		  timeOut = true;
		}
		else if (Td.total_milliseconds() > 2400) //raise the wrist
		{
		   setVelocity(-0.25,0);
		   std_msgs::Float32 angle;
		   angle.data = 0;
		   wristAnglePublish.publish(angle); //raise wrist
		}
		else if (Td.total_milliseconds() > 1700) //close the fingers and stop driving
		{
		   setVelocity(-0.1,0);
		   std_msgs::Float32 angle;
		   angle.data = 0;
		   fingerAnglePublish.publish(angle); //close fingers	   
		}

		if (Td.total_milliseconds() > 3800 && timeOut) //if enough time has pasted enter a recovery state to reattempt a pickup
		{
		  if (blockBlock) //if the ultrasound is blocked at less than .12 meters a block has been picked up no new pickup required
		  {
		     //assume target has been picked up by gripper
			targetCollected = true;
			stateMachineState = STATE_MACHINE_TRANSFORM;

			goalLocation.theta = atan2(centerLocation.y - currentLocation.y,centerLocation.x - currentLocation.x);
						
			//set center as goal position
			goalLocation.x = centerLocation.x;
			goalLocation.y = centerLocation.y;
			
			//lower wrist to avoid ultrasound sensors
			std_msgs::Float32 angle;
			angle.data = M_PI_2/4.2;
			wristAnglePublish.publish(angle);
			setVelocity(0.0,0);
		  }
		  else //recover begin looking for targets again
		  {
		  lockTarget = false;
		  setVelocity(-0.15,0);
		  //set gripper
		  std_msgs::Float32 angle;
		  //open fingers
		  angle.data = M_PI_2;
		  fingerAnglePublish.publish(angle);
		  angle.data = 0;
		  wristAnglePublish.publish(angle); //raise wrist
		  }
		}
		if (Td.total_milliseconds() > 5000 && timeOut) //if no targets are found after too long a period go back to search pattern
		{
		  targetDetected = false;
		  lockTarget = false;
		  timeOut = false;
		  stateMachineState = STATE_MACHINE_TRANSFORM;
		  setVelocity(0.0,0);
		  //set gripper
		  std_msgs::Float32 angle;
		  //open fingers
		  angle.data = M_PI_2;
		  fingerAnglePublish.publish(angle);
		}
	
	}



}

void modeHandler(const std_msgs::UInt8::ConstPtr& message) {
	currentMode = message->data;
	setVelocity(0.0, 0.0);
}

void obstacleHandler(const std_msgs::UInt8::ConstPtr& message) {
	if ((!targetDetected || targetCollected) && (message->data > 0)) {
		//obstacle on right side
		if (message->data == 1) {
			//select new heading 0.2 radians to the left
			goalLocation.theta = currentLocation.theta - 0.4;
		}
		
		//obstacle in front or on left side
		else if (message->data == 2) {
			//select new heading 0.2 radians to the right
			goalLocation.theta = currentLocation.theta - 0.3;
		}
							
		//select new position 50 cm from current location
		goalLocation.x = currentLocation.x + (0.5 * cos(goalLocation.theta));
		goalLocation.y = currentLocation.y + (0.5 * sin(goalLocation.theta));
		if (spinWasTrue)
		{
			centerLocation.x = goalLocation.x;
			centerLocation.y = goalLocation.y;
		}
		
		//switch to transform state to trigger collision avoidance
		stateMachineState = STATE_MACHINE_TRANSFORM;
	}


	if (message->data == 4)
	{
		blockBlock = true;
	}
	else
	{
		blockBlock = false;
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
   float forward = PV/355-(abs(turn)/5);
   if (linearVel >= 0 && forward <= 0)
   {
   forward = 0;
   }
   if (linearVel <= 0 && forward >= 0)
   {
   forward = 0;
   }
   /*std_msgs::String msg;
   stringstream ss;
   ss << "";
   msg.data = ss.str();
   infoLogPublisher.publish(msg);*/


  velocity.linear.x = forward, // * 1.5;
  velocity.angular.z = turn; // * 8; //scaling factor for sim; removed by aBridge node
  velocityPublish.publish(velocity);
}




//This is code for map link to odom link
/*
		geometry_msgs::PoseStamped mapOrigin;
		geometry_msgs::PoseStamped odomPose;

		try {
			tfListener->waitForTransform(publishedName + "/map", publishedName + "/odom", ros::Time(0), ros::Duration(1.0));
			tfListener->transformPose(publishedName + "/odom", mapOrigin, odomPose);
		}

		catch(tf::TransformException& ex) {
			ROS_INFO("Received an exception trying to transform a point from \"map\" to \"odom\": %s", ex.what());
		}

		//x coord = odomPose.pose.position.x;
*/


//This is code for camera link to odom link
/*
	    	tagPose.header.stamp = ros::Time(0);
		geometry_msgs::PoseStamped odomPose;
		try {
		tfListener->waitForTransform(publishedName + "/odom", publishedName + "/camera_link", ros::Time(0), ros::Duration(1.0));
		tfListener->transformPose(publishedName + "/odom", tagPose, odomPose);
		}
		catch(tf::TransformException& ex) {
		ROS_INFO("Received an exception trying to transform a point from \"odom\" to \"camera_link\": %s", ex.what());
		}
		//x coord = odomPose.pose.position.x;
*/
