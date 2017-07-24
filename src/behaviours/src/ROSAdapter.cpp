#include <ros/ros.h>

// ROS libraries
#include <angles/angles.h>
#include <random_numbers/random_numbers.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

// ROS messages
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

// Include Controllers
#include "LogicController.h"
#include <vector>

// To handle shutdown signals so the node quits
// properly in response to "rosnode kill"
#include <ros/ros.h>
#include <signal.h>


using namespace std;

// Random number generator
random_numbers::RandomNumberGenerator* rng;

// Create logic controller
LogicController logicController;

// Behaviours Logic Functions
void sendDriveCommand(double linearVel, double angularVel);
void openFingers(); // Open fingers to 90 degrees
void closeFingers();// Close fingers to 0 degrees
void raiseWrist();  // Return wrist back to 0 degrees
void lowerWrist();  // Lower wrist to 50 degrees
void resultHandler();


// Numeric Variables for rover positioning
geometry_msgs::Pose2D currentLocation;
geometry_msgs::Pose2D currentLocationMap;
geometry_msgs::Pose2D currentLocationAverage;

geometry_msgs::Pose2D centerLocation;
geometry_msgs::Pose2D centerLocationMap;
geometry_msgs::Pose2D centerLocationOdom;

int currentMode = 0;
const float behaviourLoopTimeStep = 0.1; // time between the behaviour loop calls
const float status_publish_interval = 1;
const float heartbeat_publish_interval = 2;
const float waypointTolerance = 0.1; //10 cm tolerance.

// used for calling code once but not in main
bool initilized = false;

float linearVelocity = 0;
float angularVelocity = 0;

float prevWrist = 0;
float prevFinger = 0;

Result result;

std_msgs::String msg;


geometry_msgs::Twist velocity;
char host[128];
string publishedName;
char prev_state_machine[128];

// Publishers
ros::Publisher stateMachinePublish;
ros::Publisher status_publisher;
ros::Publisher fingerAnglePublish;
ros::Publisher wristAnglePublish;
ros::Publisher infoLogPublisher;
ros::Publisher driveControlPublish;
ros::Publisher heartbeatPublisher;

// Subscribers
ros::Subscriber joySubscriber;
ros::Subscriber modeSubscriber;
ros::Subscriber targetSubscriber;
ros::Subscriber odometrySubscriber;
ros::Subscriber mapSubscriber;


// Timers
ros::Timer stateMachineTimer;
ros::Timer publish_status_timer;
ros::Timer publish_heartbeat_timer;

// records time for delays in sequanced actions, 1 second resolution.
time_t timerStartTime;

// An initial delay to allow the rover to gather enough position data to 
// average its location.
unsigned int startDelayInSeconds = 1;
float timerTimeElapsed = 0;

//Transforms
tf::TransformListener *tfListener;

// OS Signal Handler
void sigintEventHandler(int signal);

//Callback handlers
void joyCmdHandler(const sensor_msgs::Joy::ConstPtr& message);
void modeHandler(const std_msgs::UInt8::ConstPtr& message);
void targetHandler(const apriltags_ros::AprilTagDetectionArray::ConstPtr& tagInfo);
void odometryHandler(const nav_msgs::Odometry::ConstPtr& message);
void mapHandler(const nav_msgs::Odometry::ConstPtr& message);
void behaviourStateMachine(const ros::TimerEvent&);
void publishStatusTimerEventHandler(const ros::TimerEvent& event);
void publishHeartBeatTimerEventHandler(const ros::TimerEvent& event);
void sonarHandler(const sensor_msgs::Range::ConstPtr& sonarLeft, const sensor_msgs::Range::ConstPtr& sonarCenter, const sensor_msgs::Range::ConstPtr& sonarRight);

// Converts the time passed as reported by ROS (which takes Gazebo simulation rate into account) into milliseconds as an integer.
long int getROSTimeInMilliSecs();

int main(int argc, char **argv) {
  
  gethostname(host, sizeof (host));
  string hostname(host);

  if (argc >= 2) {
    publishedName = argv[1];
    cout << "Welcome to the world of tomorrow " << publishedName
         << "!  Behaviour turnDirectionule started." << endl;
  } else {
    publishedName = hostname;
    cout << "No Name Selected. Default is: " << publishedName << endl;
  }

  // NoSignalHandler so we can catch SIGINT ourselves and shutdown the node
  ros::init(argc, argv, (publishedName + "_BEHAVIOUR"), ros::init_options::NoSigintHandler);
  ros::NodeHandle mNH;

  // Register the SIGINT event handler so the node can shutdown properly
  signal(SIGINT, sigintEventHandler);

  joySubscriber = mNH.subscribe((publishedName + "/joystick"), 10, joyCmdHandler);
  modeSubscriber = mNH.subscribe((publishedName + "/mode"), 1, modeHandler);
  targetSubscriber = mNH.subscribe((publishedName + "/targets"), 10, targetHandler);
  odometrySubscriber = mNH.subscribe((publishedName + "/odom/filtered"), 10, odometryHandler);
  mapSubscriber = mNH.subscribe((publishedName + "/odom/ekf"), 10, mapHandler);
  message_filters::Subscriber<sensor_msgs::Range> sonarLeftSubscriber(mNH, (publishedName + "/sonarLeft"), 10);
  message_filters::Subscriber<sensor_msgs::Range> sonarCenterSubscriber(mNH, (publishedName + "/sonarCenter"), 10);
  message_filters::Subscriber<sensor_msgs::Range> sonarRightSubscriber(mNH, (publishedName + "/sonarRight"), 10);

  status_publisher = mNH.advertise<std_msgs::String>((publishedName + "/status"), 1, true);
  stateMachinePublish = mNH.advertise<std_msgs::String>((publishedName + "/state_machine"), 1, true);
  fingerAnglePublish = mNH.advertise<std_msgs::Float32>((publishedName + "/fingerAngle/cmd"), 1, true);
  wristAnglePublish = mNH.advertise<std_msgs::Float32>((publishedName + "/wristAngle/cmd"), 1, true);
  infoLogPublisher = mNH.advertise<std_msgs::String>("/infoLog", 1, true);
  driveControlPublish = mNH.advertise<geometry_msgs::Twist>((publishedName + "/driveControl"), 10);
  heartbeatPublisher = mNH.advertise<std_msgs::String>((publishedName + "/behaviour/heartbeat"), 1, true);

  publish_status_timer = mNH.createTimer(ros::Duration(status_publish_interval), publishStatusTimerEventHandler);
  stateMachineTimer = mNH.createTimer(ros::Duration(behaviourLoopTimeStep), behaviourStateMachine);

  publish_heartbeat_timer = mNH.createTimer(ros::Duration(heartbeat_publish_interval), publishHeartBeatTimerEventHandler);

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Range, sensor_msgs::Range, sensor_msgs::Range> sonarSyncPolicy;

  message_filters::Synchronizer<sonarSyncPolicy> sonarSync(sonarSyncPolicy(10), sonarLeftSubscriber, sonarCenterSubscriber, sonarRightSubscriber);
  sonarSync.registerCallback(boost::bind(&sonarHandler, _1, _2, _3));

  tfListener = new tf::TransformListener();
  std_msgs::String msg;
  msg.data = "Log Started";
  infoLogPublisher.publish(msg);

  stringstream ss;
  ss << "Rover start delay set to " << startDelayInSeconds << " seconds";
  msg.data = ss.str();
  infoLogPublisher.publish(msg);

  timerStartTime = time(0);

  ros::spin();

  return EXIT_SUCCESS;
}


// This is the top-most logic control block organised as a state machine.
// This function calls the dropOff, pickUp, and search controllers.
// This block passes the goal location to the proportional-integral-derivative
// controllers in the abridge package.
void behaviourStateMachine(const ros::TimerEvent&) {
  std_msgs::String stateMachineMsg;

  // time since timerStartTime was set to current time
  timerTimeElapsed = time(0) - timerStartTime;

  // init code goes here. (code that runs only once at start of
  // auto mode but wont work in main goes here)
  if (!initilized) {
    if (timerTimeElapsed > startDelayInSeconds) {
      // initialization has run
      initilized = true;
    } else {
      return;
    }

  }


  // Robot is in automode
  if (currentMode == 2 || currentMode == 3) {

    Point centerOdom;
    centerOdom.x = centerLocation.x;
    centerOdom.y = centerLocation.y;
    centerOdom.theta = centerLocation.theta;
    logicController.SetCenterLocationOdom(centerOdom);

    Point centerMap;
    centerMap.x = centerLocationMap.x;
    centerMap.y = centerLocationMap.y;
    centerMap.theta = centerLocationMap.theta;
    logicController.SetCenterLocationMap(centerMap);

    logicController.SetCurrentTimeInMilliSecs( getROSTimeInMilliSecs() );

    result = logicController.DoWork();

    bool skip = false;
    if (result.type == behavior) {
      if (result.b = wait) {
        skip = true;
      }
    }
    if (skip) {
      sendDriveCommand(0.0,0.0);
      std_msgs::Float32 angle;

      angle.data = prevFinger;
      fingerAnglePublish.publish(angle);
      angle.data = prevWrist;
      wristAnglePublish.publish(angle);
    }
    else {

      sendDriveCommand(result.pd.left,result.pd.right);

      std_msgs::Float32 angle;
      if (result.fingerAngle != -1) {
        angle.data = result.fingerAngle;
        fingerAnglePublish.publish(angle);
        prevFinger = result.fingerAngle;
      }
      if (result.wristAngle != -1) {
        angle.data = result.wristAngle;
        wristAnglePublish.publish(angle);
        prevWrist = result.wristAngle;
      }
    }

    //publishHandeling here
    //logicController.getPublishData() suggested;
    cout << endl;
  }
  // mode is NOT auto
  else {
    // publish current state for the operator to see
    stateMachineMsg.data = "WAITING";
  }


  // publish state machine string for user, only if it has changed, though
  if (strcmp(stateMachineMsg.data.c_str(), prev_state_machine) != 0) {
    stateMachinePublish.publish(stateMachineMsg);
    sprintf(prev_state_machine, "%s", stateMachineMsg.data.c_str());
  }
}

void sendDriveCommand(double left, double right)
{
  velocity.linear.x = left,
      velocity.angular.z = right;

  // publish the drive commands
  driveControlPublish.publish(velocity);
}

/*************************
 * ROS CALLBACK HANDLERS *
 *************************/

void targetHandler(const apriltags_ros::AprilTagDetectionArray::ConstPtr& message) {

  if (message->detections.size() > 0) {
    vector<TagPoint> tags;

    for (int i = 0; i < message->detections.size(); i++) {

      TagPoint loc;
      loc.id = message->detections[i].id;
      geometry_msgs::PoseStamped tagPose = message->detections[i].pose;
      loc.x = tagPose.pose.position.x;
      loc.y = tagPose.pose.position.y;
      loc.z = tagPose.pose.position.z;
      //loc.theta =
      tags.push_back(loc);
    }
    logicController.SetAprilTags(tags);
  }

}

void modeHandler(const std_msgs::UInt8::ConstPtr& message) {
  currentMode = message->data;
  sendDriveCommand(0.0, 0.0);
}

void sonarHandler(const sensor_msgs::Range::ConstPtr& sonarLeft, const sensor_msgs::Range::ConstPtr& sonarCenter, const sensor_msgs::Range::ConstPtr& sonarRight) {

  logicController.SetSonarData(sonarLeft->range, sonarCenter->range, sonarRight->range);

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

  linearVelocity = message->twist.twist.linear.x;
  angularVelocity = message->twist.twist.angular.z;


  Point currentLoc;
  currentLoc.x = currentLocation.x;
  currentLoc.y = currentLocation.y;
  currentLoc.theta = currentLocation.theta;
  logicController.SetPositionData(currentLoc);
  logicController.SetVelocityData(linearVelocity, angularVelocity);
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

  linearVelocity = message->twist.twist.linear.x;
  angularVelocity = message->twist.twist.angular.z;

  Point currentLoc;
  currentLoc.x = currentLocation.x;
  currentLoc.y = currentLocation.y;
  currentLoc.theta = currentLocation.theta;
  logicController.SetPositionData(currentLoc);
  logicController.SetMapVelocityData(linearVelocity, angularVelocity);
}

void joyCmdHandler(const sensor_msgs::Joy::ConstPtr& message) {
  if (currentMode == 0 || currentMode == 1) {
    sendDriveCommand(abs(message->axes[4]) >= 0.1 ? message->axes[4] : 0, abs(message->axes[3]) >= 0.1 ? message->axes[3] : 0);
  }
}


void publishStatusTimerEventHandler(const ros::TimerEvent&) {
  std_msgs::String msg;
  msg.data = "online";
  status_publisher.publish(msg);
}

void sigintEventHandler(int sig) {
  // All the default sigint handler does is call shutdown()
  ros::shutdown();
}

void publishHeartBeatTimerEventHandler(const ros::TimerEvent&) {
  std_msgs::String msg;
  msg.data = "";
  heartbeatPublisher.publish(msg);
}

long int getROSTimeInMilliSecs()
{
  // Get the current time according to ROS (will be zero for simulated clock until the first time message is recieved).
  ros::Time t = ros::Time::now();

  // Convert from seconds and nanoseconds to milliseconds.
  return t.sec*1e3 + t.nsec/1e6;
  
}
