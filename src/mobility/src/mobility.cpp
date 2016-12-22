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
void sendDriveCommand(double linearVel, double angularVel);
void openFingers(); // Open fingers to 90 degrees
void closeFingers();// Close fingers to 0 degrees
void raiseWrist();  // Return wrist back to 0 degrees
void lowerWrist();  // Lower wrist to 50 degrees
void mapAverage();  //constantly averages last 100 positions from map


//Numeric Variables
geometry_msgs::Pose2D currentLocation;
geometry_msgs::Pose2D currentLocationMap;
geometry_msgs::Pose2D currentLocationAverage;
geometry_msgs::Pose2D goalLocation;
geometry_msgs::Pose2D centerLocation;
geometry_msgs::Pose2D centerLocationMap;
geometry_msgs::Pose2D mapLocation[100];

int currentMode = 0;
float mobilityLoopTimeStep = 0.1; //time between the mobility loop calls
float status_publish_interval = 5;
bool targetDetected = false; 
bool targetCollected = false;

//set true when the target block is less than targetDist so we continue attempting to pick it up rather than
//switching to another block that is in view
bool lockTarget = false; 

// Failsafe state. No legitimate behavior state. If in this state for too long return to searching as default behavior.
bool timeOut = false;

//set to true when the center ultrasound reads less than 0.14m. Usually means a picked up cube is in the way
bool blockBlock = false; 

//central collection point has been seen (aka the nest)
bool centerSeen = false;

//set true when we are insie the center circle and we need to drop the block, back out, and reset the boolean cascade.
bool reachedCollectionPoint = false;

//we have seen enough central collection tags to be certain we are either in or driving towards the nest.
bool seenEnoughCenterTags = false; 

//set to true when we are entering the center circle
bool centerApproach = false;

//keep track of progression around a circle when driving in a circle
float spinner = 0;

//is driving in a circle to find the nest
bool circularCenterSearching = false;

//used for calling code once but not in main
bool init = false;

//used to remember place in mapAverage array
int mapCount = 0;

float searchVelocity = 0.15;



std_msgs::String msg;

// state machine states
#define STATE_MACHINE_TRANSFORM	0
#define STATE_MACHINE_ROTATE	1
#define STATE_MACHINE_DIFFERENTIAL_DRIVE	2
#define STATE_MACHINE_PICKUP    3
#define STATE_MACHINE_DROPOFF   4

int stateMachineState = STATE_MACHINE_TRANSFORM;

geometry_msgs::Twist velocity;
char host[128];
string publishedName;
char prev_state_machine[128];

//Publishers
ros::Publisher stateMachinePublish;
ros::Publisher status_publisher;
ros::Publisher fingerAnglePublish;
ros::Publisher wristAnglePublish;
ros::Publisher infoLogPublisher;
ros::Publisher driveControlPublish;

//Subscribers
ros::Subscriber joySubscriber;
ros::Subscriber turnDirectioneSubscriber;
ros::Subscriber targetSubscriber;
ros::Subscriber obstacleSubscriber;
ros::Subscriber odometrySubscriber;
ros::Subscriber mapSubscriber;


//Timers
ros::Timer stateMachineTimer;
ros::Timer publish_status_timer;
ros::Timer targetDetectedTimer;
time_t timerStartTime; //records time for delays in sequanced actions, 1 second resolution.
time_t timeWithoutSeeingEnoughCenterTags;
float timerTimeElapsed = 0;
float timeElapsedSinceTimeSinceSeeingEnoughCenterTags = 0;

boost::posix_time::ptime millTimer;

//Transforms
tf::TransformListener *tfListener;

// OS Signal Handler
void sigintEventHandler(int signal);

//Callback handlers
void joyCmdHandler(const sensor_msgs::Joy::ConstPtr& message);
void turnDirectioneHandler(const std_msgs::UInt8::ConstPtr& message);
void targetHandler(const apriltags_ros::AprilTagDetectionArray::ConstPtr& tagInfo);
void obstacleHandler(const std_msgs::UInt8::ConstPtr& message);
void odometryHandler(const nav_msgs::Odometry::ConstPtr& message);
void mapHandler(const nav_msgs::Odometry::ConstPtr& message);
void mobilityStateMachine(const ros::TimerEvent&);
void publishStatusTimerEventHandler(const ros::TimerEvent& event);
void targetDetectedReset(const ros::TimerEvent& event);


int main(int argc, char **argv) {

    gethostname(host, sizeof (host));
    string hostname(host);

    rng = new random_numbers::RandomNumberGenerator(); //instantiate random number generator
    goalLocation.theta = rng->uniformReal(0, 2 * M_PI); //set initial random heading

    //select initial search position 50 cm from center (0,0)

	goalLocation.x = 0.5 * cos(goalLocation.theta+M_PI);
	goalLocation.y = 0.5 * sin(goalLocation.theta+M_PI);

    centerLocation.x = 0;
    centerLocation.y = 0;

	for (int i = 0; i < 100; i++)
	{
	  mapLocation[i].x = 0;
	  mapLocation[i].y = 0;
	  mapLocation[i].theta = 0;
	}

    if (argc >= 2) {
        publishedName = argv[1];
        cout << "Welcome to the world of tomorrow " << publishedName << "!  Mobility turnDirectionule started." << endl;
    } else {
        publishedName = hostname;
        cout << "No Name Selected. Default is: " << publishedName << endl;
    }

    // NoSignalHandler so we can catch SIGINT ourselves and shutdown the node
    ros::init(argc, argv, (publishedName + "_MOBILITY"), ros::init_options::NoSigintHandler);
    ros::NodeHandle mNH;

    signal(SIGINT, sigintEventHandler); // Register the SIGINT event handler so the node can shutdown properly

    joySubscriber = mNH.subscribe((publishedName + "/joystick"), 10, joyCmdHandler);
    turnDirectioneSubscriber = mNH.subscribe((publishedName + "/turnDirectione"), 1, turnDirectioneHandler);
    targetSubscriber = mNH.subscribe((publishedName + "/targets"), 10, targetHandler);
    obstacleSubscriber = mNH.subscribe((publishedName + "/obstacle"), 10, obstacleHandler);
    odometrySubscriber = mNH.subscribe((publishedName + "/odom/filtered"), 10, odometryHandler);
    mapSubscriber = mNH.subscribe((publishedName + "/odom/ekf"), 10, mapHandler);

    status_publisher = mNH.advertise<std_msgs::String>((publishedName + "/status"), 1, true);
    stateMachinePublish = mNH.advertise<std_msgs::String>((publishedName + "/state_machine"), 1, true);
    fingerAnglePublish = mNH.advertise<std_msgs::Float32>((publishedName + "/fingerAngle/cmd"), 1, true);
    wristAnglePublish = mNH.advertise<std_msgs::Float32>((publishedName + "/wristAngle/cmd"), 1, true);
    infoLogPublisher = mNH.advertise<std_msgs::String>("/infoLog", 1, true);
    driveControlPublish = mNH.advertise<geometry_msgs::Twist>((publishedName + "/driveControl"), 10);

    publish_status_timer = mNH.createTimer(ros::Duration(status_publish_interval), publishStatusTimerEventHandler);
    stateMachineTimer = mNH.createTimer(ros::Duration(mobilityLoopTimeStep), mobilityStateMachine);
    targetDetectedTimer = mNH.createTimer(ros::Duration(0), targetDetectedReset, true);
    
    tfListener = new tf::TransformListener();

    std_msgs::String msg;
    msg.data = "Log Started";
    infoLogPublisher.publish(msg);
    
    timerStartTime = time(0);
        
    ros::spin();

    
    return EXIT_SUCCESS;
}

void mobilityStateMachine(const ros::TimerEvent&) {
    std_msgs::String stateMachineMsg;
    float rotateOnlyAngleTolerance = 0.4;
    int returnToSearchDelay = 5;

    mapAverage(); //calls the averaging function, also responsible for transform from Map frame to odom frame.
    
    if (currentMode == 2 || currentMode == 3) { //Robot is in autoturnDirectione

    timerTimeElapsed = time(0) - timerStartTime; //time since timerStartTime was set to current time

	if (!init) //initiliation code goes here. (code that runs only once at start of auto turnDirectione but wont work in main goes here)
	{
	  centerLocationMap.x = currentLocationAverage.x; // set the location of the center circle location in the map frame
	  centerLocationMap.y = currentLocationAverage.y; // based upon our current average location on the map.
	  centerLocationMap.theta = currentLocationAverage.theta;
	  init = true; //initiliation has run
	}

	if (!targetCollected && !targetDetected) //if no collected or detected blocks set fingers to open wide and raised position.
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
				//if we are in the routine for exciting the circle once we have droppeda block off and reseting all our flags 
				//to resart our search.
				if(reachedCollectionPoint) 
				{
				    //set goalLocation to currentLocation so we can drive how we want to instead of using differential drive and rotate
					goalLocation.x = currentLocation.x; 
					goalLocation.y = currentLocation.y;
					goalLocation.theta = currentLocation.theta;
					//timerStartTime was reset before we entered reachedCollectionPoint so 
					//we can now use it for our timeing of 2 seconds 
					
					if (timerTimeElapsed >= 3) 
					 {
					   //reset flag
				  	   targetCollected = false;
				  	   targetDetected = false;
					   lockTarget = false;
					   reachedCollectionPoint = false;
					   seenEnoughCenterTags = false;
					   circularCenterSearching = false;
					   timerStartTime = time(0);
					   sendDriveCommand(0.0,0);
				   	   stateMachineState = STATE_MACHINE_TRANSFORM; //move back to transform step
					 }
					else if (timerTimeElapsed >= 1)
					{
					  //open fingers
					  std_msgs::Float32 angle;
					  angle.data = M_PI_2;
					  fingerAnglePublish.publish(angle);
		   			  angle.data = 0;
		   			  wristAnglePublish.publish(angle); //raise wrist
						
					  sendDriveCommand(-0.3,0.0); //drive backwards out of the circle
					}
					break;
				}
				//If angle between current and goal is significant
				//if error in heading is greater than 0.4 radians
				if (fabs(angles::shortest_angular_distance(currentLocation.theta, goalLocation.theta)) > rotateOnlyAngleTolerance) 
				{
					stateMachineState = STATE_MACHINE_ROTATE; //rotate
				}
				//If goal has not yet been reached drive and maintane heading
				else if (fabs(angles::shortest_angular_distance(currentLocation.theta, atan2(goalLocation.y - currentLocation.y, goalLocation.x - currentLocation.x))) < M_PI_2) //pi/2
				{
					stateMachineState = STATE_MACHINE_DIFFERENTIAL_DRIVE; //differential drive
				}
				//If returning with a target
				else if (targetCollected && !centerSeen && !reachedCollectionPoint) {
					//If goal has not yet been reached
					float collectionPointVisualDistance = 0.3; //in meters
					
					//calculate the euclidean distance between centerLocation and currentLocation
					if (hypot(centerLocation.x - currentLocation.x, centerLocation.y - currentLocation.y) > collectionPointVisualDistance && !circularCenterSearching) {
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
					//sets a goal that is 60cm from the centerLocation and spinner 
					//radians counterclockwise from being purly along the x-axis.
					 goalLocation.x = centerLocation.x + 0.6 * cos(spinner); 
					 goalLocation.y = centerLocation.y + 0.6 * sin(spinner);
					 goalLocation.theta = atan2(goalLocation.y - currentLocation.y, goalLocation.x - currentLocation.x);
					 
					 spinner += 45*(M_PI/180); //add 45 degrees in radians to spinner.
					 if (spinner > 2*M_PI)
					 {
					    spinner -= 2*M_PI;
					 }
					 circularCenterSearching = true; 
					 //safety flag to prevent us trying to drive back to the 
					 //center since we have a block with us and the above point is 
					 //greater than collectionPointVisualDistance from the center.
					 stateMachineState = STATE_MACHINE_ROTATE;		
					}
				}
			    //Otherwise, drop off target and select new random uniform heading
				//If no targets have been detected, assign a new goal
				else if (!targetDetected && timerTimeElapsed > returnToSearchDelay) {

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
				//calculate the diffrence between current and desired heading in radians.
				float errorYaw = angles::shortest_angular_distance(currentLocation.theta, goalLocation.theta); 
				//if angle is greater than 0.4 radians rotate but dont drive forward.
			    if (fabs(angles::shortest_angular_distance(currentLocation.theta, goalLocation.theta)) > rotateOnlyAngleTolerance) 
			    {		
					sendDriveCommand(0.05, errorYaw); //rotate but dont drive  0.05 is to prevent turning in reverse
					break;
			    }
				else {
					stateMachineState = STATE_MACHINE_DIFFERENTIAL_DRIVE; //move to differential drive step
					//fall through on purpose.
				}

			}
			
			//Calculate angle between currentLocation.x/y and goalLocation.x/y
			//Drive forward
			//Stay in this state until angle is at least PI/2
			case STATE_MACHINE_DIFFERENTIAL_DRIVE: {
				stateMachineMsg.data = "TRANSLATING";
				//calculate the distance between current and desired heading in radians
				float errorYaw = angles::shortest_angular_distance(currentLocation.theta, goalLocation.theta);
				//goal not yet reached drive while maintaining proper heading.
				if (fabs(angles::shortest_angular_distance(currentLocation.theta, atan2(goalLocation.y - currentLocation.y, goalLocation.x - currentLocation.x))) < M_PI_2) {
					sendDriveCommand(searchVelocity, errorYaw/2); //drive and turn simultaniously
				}
				//goal is reached but desired heading is still wrong turn only
				else if (fabs(angles::shortest_angular_distance(currentLocation.theta, goalLocation.theta)) > 0.1) {
					sendDriveCommand(0.0, errorYaw); //rotate but dont drive
			    }
			   	else {
					sendDriveCommand(0.0, 0.0); //stop
					
					stateMachineState = STATE_MACHINE_TRANSFORM; //move back to transform step

				}
			    break;
			}
			case STATE_MACHINE_PICKUP: {
			//this is a blocker to prevent transform, rotate, and differential drive from operating
			//this allows the target handler to control movement directly
			break;
			}
			case STATE_MACHINE_DROPOFF: {
			if (!centerSeen && seenEnoughCenterTags)
			{
			  reachedCollectionPoint = true;
			  stateMachineState = STATE_MACHINE_TRANSFORM;
			  timerStartTime = time(0);
			  goalLocation.x = currentLocation.x;
			  goalLocation.y = currentLocation.y;
			  goalLocation.theta = currentLocation.theta;
			  centerApproach = false;
			}
			break;
			}
		
			default: {
			    break;
			}
		}
	}

    else { // turnDirectione is NOT auto

        // publish current state for the operator to see
        stateMachineMsg.data = "WAITING";
    }

    // publish state machine string for user, only if it has changed, though
    if (strcmp(stateMachineMsg.data.c_str(), prev_state_machine) != 0) {
        stateMachinePublish.publish(stateMachineMsg);
        sprintf(prev_state_machine, "%s", stateMachineMsg.data.c_str());
    }
}

void sendDriveCommand(double linearVel, double angularError) 
{
  velocity.linear.x = linearVel,
  velocity.angular.z = angularError;
  driveControlPublish.publish(velocity); //publish the drive commands
}

/***********************
 * ROS CALLBACK HANDLERS
 ************************/

void targetHandler(const apriltags_ros::AprilTagDetectionArray::ConstPtr& message) {

        // If in manual turnDirectione do not try to automatically pick up the target
        if (currentMode == 1 || currentMode == 0) return;

    //yaw error to target block 
    double blockYawError = 0;
    
    //distance to target block from front of robot
    double blockDist = 0;

    //if a target is detected and we are looking for center tags
	if (message->detections.size() > 0 && !reachedCollectionPoint)
	{
	  centerSeen = false;
	  double count = 0;
	  bool right = false;
	  bool left = false;
	  float cameraOffsetCorrection = 0.020; //meters
	  float centeringTurn = 0.15; //radians
	  int seenEnoughCenterTagsCount = 20;
	  
	  for (int i = 0; i < message->detections.size(); i++) //this loop is to get the number of center tags
	  {
       	if (message->detections[i].id == 256) 
       	{
	     geometry_msgs::PoseStamped cenPose = message->detections[i].pose;
	     if (cenPose.pose.position.x + cameraOffsetCorrection > 0) //checks if tag is on the right or left side of the image
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

      //reset timeWithoutSeeingEnoughCenterTags timout timer to current time
	  if (!centerApproach || !seenEnoughCenterTags) timeWithoutSeeingEnoughCenterTags = time(0);
	
	  if (centerSeen && targetCollected) //if we have a target and the center is located drive towards it.
	  {
		float turnDirection = 1;
		//below is only usfull for the new center design as opposed to the current circle.
		//reverse tag rejection when we have seen enough tags that we are on a 
		//trajectory in to the square we dont want to follow an edge.
		//if (seenEnoughCenterTags) turnDirection = -1; 
		//otherwise turn till tags on both sides of image then drive straight
		if (left && right) sendDriveCommand(searchVelocity, 0.0); 
		else if (right) sendDriveCommand(searchVelocity, -centeringTurn*turnDirection);
		else sendDriveCommand(searchVelocity, centeringTurn*turnDirection);
    
        //must see greater than this many tags before assuming we are driving into the center and not along an edge.
		if (count > seenEnoughCenterTagsCount) 
		{
		seenEnoughCenterTags = true; //we have driven far enough forward to be in the circle.
		timeWithoutSeeingEnoughCenterTags = time(0);
		}
        //time since we dropped below countGuard tags
		timeElapsedSinceTimeSinceSeeingEnoughCenterTags = time(0) - timeWithoutSeeingEnoughCenterTags; 

        //we have driven far enough forward to have passed over the circle.
		if (count < seenEnoughCenterTagsCount && seenEnoughCenterTags && timeElapsedSinceTimeSinceSeeingEnoughCenterTags > 0) centerSeen = false; 
		stateMachineState = STATE_MACHINE_DROPOFF; //go to dropoff turnDirectione to prevent velocity overrides.
		centerApproach = true;
		
	  }
	  else if (centerSeen) 
	  {
	     //if you want to avoid the center diffrently turnDirection here
	     //this code keeps the robot from driving over the center when searching for blocks
		 stateMachineState = STATE_MACHINE_TRANSFORM;
		 if (right)
		 {
		 goalLocation.theta += centeringTurn; //turn away from the center to the left if just driving around/searching.
		 }
		 else
		 {
		 goalLocation.theta -= centeringTurn; //turn away from the center to the right if just driving around/searching.
		 }
		 //remainingGoalDist avoids magic numbers by calculating the dist
		 double remainingGoalDist = hypot(goalLocation.x - currentLocation.x, goalLocation.y - currentLocation.y); 
		 //this of course assumes random walk continuation. Change for diffrent search methods.
		 goalLocation.x = currentLocation.x + (remainingGoalDist * cos(goalLocation.theta)); 
		 goalLocation.y = currentLocation.y + (remainingGoalDist * sin(goalLocation.theta));
		
		targetDetected = false;		
		return;
	  }	  	
	}
	else if (centerApproach) //was on approach to center and did not seenEnoughCenterTags 
	                         //for maxTimeAllowedWithoutSeeingCenterTags seconds so reset.
 	{
 	  int maxTimeAllowedWithoutSeeingCenterTags = 4; //seconds
 	  
	  timeElapsedSinceTimeSinceSeeingEnoughCenterTags = time(0) - timeWithoutSeeingEnoughCenterTags;
	  if (timeElapsedSinceTimeSinceSeeingEnoughCenterTags > maxTimeAllowedWithoutSeeingCenterTags) 
	  {
	   //go back to drive to center base location instead of drop off attempt
	   stateMachineState = STATE_MACHINE_TRANSFORM;
	   seenEnoughCenterTags = false;
	   centerApproach = false;
	  }
  	 }
  	 //end found target and looking for center tags

 
    //found a target and looking for april cubes
	if (message->detections.size() > 0 && !targetCollected && timerTimeElapsed > 5) 
	{

	targetDetected = true;
	stateMachineState = STATE_MACHINE_PICKUP; //pickup state so target handler can take overdriving.
		
		double closest = std::numeric_limits<double>::max(); 
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
			sendDriveCommand(0.0,0);
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
	
	if (targetDetected && !targetCollected) //we see a block andhave not picked one up yet
	{
	    //threshold distance to be from the target block before attempting pickup
	    float targetDist = 0.25; //meters
	
	    // millisecond time = current time if not in a counting state
		if (!timeOut) millTimer = boost::posix_time::microsec_clock::local_time(); 

        //diffrence between current time and millisecond time
		boost::posix_time::time_duration Td = boost::posix_time::microsec_clock::local_time() - millTimer; 

		if (!(message->detections.size() > 0) && !lockTarget) //if not targets detected and a target has not been locked in
		{
		   if(!timeOut) //if not in a counting state
		   {
		     sendDriveCommand(0.0,0.0);
		     timeOut = true;
		   }
		   //if in a counting state and has been counting for 1 second
		   else if (Td.total_milliseconds() > 1000 && Td.total_milliseconds() < 2500) 
		   {
		     sendDriveCommand(-0.2,0.0);
		   }
		}
		else if (blockDist > targetDist && !lockTarget) //if a target is detected but not locked, and not too close.
		{
		  float vel = blockDist * 0.20;
		  if (vel < 0.1) vel = 0.1;
		  if (vel > 0.2) vel = 0.2;
		  sendDriveCommand(vel,-blockYawError/2);
		  timeOut = false;
		}
		else if (!lockTarget) //if a target hasn't been locked lock it and enter a counting state while slowly driving forward.
		{
		  lockTarget = true;
		  sendDriveCommand(0.18,0);
		  timeOut = true;
		}
		else if (Td.total_milliseconds() > 2400) //raise the wrist
		{
		   sendDriveCommand(-0.25,0);
		   std_msgs::Float32 angle;
		   angle.data = 0;
		   wristAnglePublish.publish(angle); //raise wrist
		}
		else if (Td.total_milliseconds() > 1700) //close the fingers and stop driving
		{
		   sendDriveCommand(-0.1,0);
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
			sendDriveCommand(0.0,0);
		  }
		  else //recover begin looking for targets again
		  {
		  lockTarget = false;
		  sendDriveCommand(-0.15,0);
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
		  sendDriveCommand(0.0,0);
		  //set gripper
		  std_msgs::Float32 angle;
		  //open fingers
		  angle.data = M_PI_2;
		  fingerAnglePublish.publish(angle);
		}
	
	}



}

void turnDirectioneHandler(const std_msgs::UInt8::ConstPtr& message) {
	currentMode = message->data;
	sendDriveCommand(0.0, 0.0);
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
		
		//switch to transform state to trigger collision avoidanc	e
		stateMachineState = STATE_MACHINE_TRANSFORM;
	}


	if (message->data == 4) //the front ultrasond is blocked very closely. 0.14m currently 
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

void mapHandler(const nav_msgs::Odometry::ConstPtr& message) {
	//Get (x,y) location directly from pose
	currentLocationMap.x = message->pose.pose.position.x;
	currentLocationMap.y = message->pose.pose.position.y;
	
	//Get theta rotation by converting quaternion orientation to pitch/roll/yaw
	tf::Quaternion q(message->pose.pose.orientation.x, message->pose.pose.orientation.y, message->pose.pose.orientation.z, message->pose.pose.orientation.w);
	tf::Matrix3x3 m(q);
	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);
	currentLocationMap.theta = yaw;
}

void joyCmdHandler(const sensor_msgs::Joy::ConstPtr& message) {
	if (currentMode == 0 || currentMode == 1) {
		sendDriveCommand(abs(message->axes[4]) >= 0.1 ? message->axes[4] : 0, abs(message->axes[3]) >= 0.1 ? message->axes[3] : 0);
	} 
}


void publishStatusTimerEventHandler(const ros::TimerEvent&)
{
  std_msgs::String msg;
  msg.data = "online";
  status_publisher.publish(msg);
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

void mapAverage()
{
	mapLocation[mapCount] = currentLocationMap; //store currentLocation in the averaging array
	mapCount++;
	
	if (mapCount >= 100) {mapCount = 0;}

	double x = 0;
	double y = 0;
	double theta = 0;
	for (int i = 0; i < 100; i++) //add up all the positions in the array
	{
	  x += mapLocation[i].x;
	  y += mapLocation[i].y;
	  theta += mapLocation[i].theta;
	}
	x = x/100; //find the average
	y = y/100;
	theta = theta/100;//Get theta rotation by converting quaternion orientation to pitch/roll/yaw
	currentLocationAverage.x = x;
	currentLocationAverage.y = y;
	currentLocationAverage.theta = theta;


	if (init) //only run below code if a centerLocation has been set by initilization
	{
		geometry_msgs::PoseStamped mapPose; //map frame
		mapPose.header.stamp = ros::Time::now(); //setup msg to represent the center location in map frame
		mapPose.header.frame_id = publishedName + "/map";
		mapPose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, centerLocationMap.theta);
		mapPose.pose.position.x = centerLocationMap.x;
		mapPose.pose.position.y = centerLocationMap.y;
		geometry_msgs::PoseStamped odomPose;
		string x = "";

		try { //attempt to get the transform of the center point in map frame to odom frame.
			tfListener->waitForTransform(publishedName + "/map", publishedName + "/odom", ros::Time::now(), ros::Duration(1.0));
			tfListener->transformPose(publishedName + "/odom", mapPose, odomPose);
		}

		catch(tf::TransformException& ex) {
			ROS_INFO("Received an exception trying to transform a point from \"map\" to \"odom\": %s", ex.what());
			x = "Exception thrown " + (string)ex.what();
			 std_msgs::String msg;
  			 stringstream ss;
  			 ss << "Exception in mapAverage() " + (string)ex.what();
   			 msg.data = ss.str();
  			 infoLogPublisher.publish(msg);
		}

		centerLocation.x = odomPose.pose.position.x; //set centerLocation in odom frame
		centerLocation.y = odomPose.pose.position.y;
	}
}




//This is code for map link to odom link
/*
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
	x = odomPose.pose.position.x;

*/


//This is code for camera link to odom link
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

/*
   std_msgs::String msg;
   stringstream ss;
   ss << "map center " << centerLocationMap.x << " : " << centerLocationMap.y << " centerLocation " << centerLocation.x << " : " << centerLocation.y << " currentLlocation " << currentLocation.x << " : " << currentLocation.y << " currentLocationAverage " << currentLocationAverage.x << " : " << currentLocationAverage.y << "curMap " << currentLocationMap.x << " : " << currentLocationMap.y;
   msg.data = ss.str();
   infoLogPublisher.publish(msg);
   */
