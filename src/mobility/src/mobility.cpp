#include <ros/ros.h>

// ROS libraries
#include <angles/angles.h>
#include <random_numbers/random_numbers.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

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
#include "PickUpController.h"
#include "DropOffController.h"
#include "SearchController.h"

// To handle shutdown signals so the node quits
// properly in response to "rosnode kill"
#include <ros/ros.h>
#include <signal.h>


using namespace std;

// Random number generator
random_numbers::RandomNumberGenerator* rng;

// Create controllers
PickUpController pickUpController;
DropOffController dropOffController;
SearchController searchController;

// Mobility Logic Functions
void sendDriveCommand(double linearVel, double angularVel);
void openFingers(); // Open fingers to 90 degrees
void closeFingers();// Close fingers to 0 degrees
void raiseWrist();  // Return wrist back to 0 degrees
void lowerWrist();  // Lower wrist to 50 degrees

// Numeric Variables for rover positioning
geometry_msgs::Pose2D currentLocation;
geometry_msgs::Pose2D currentLocationMap;
geometry_msgs::Pose2D currentLocationAverage;
geometry_msgs::Pose2D goalLocation;

geometry_msgs::Pose2D centerLocation;
geometry_msgs::Pose2D centerLocationMap;
geometry_msgs::Pose2D centerLocationOdom;

int currentMode = 0;
float mobilityLoopTimeStep = 0.1; // time between the mobility loop calls
float status_publish_interval = 1;
float heartbeat_publish_interval = 2;



std_msgs::String msg;

// state machine states
#define STATE_MACHINE_TRANSFORM 0
#define STATE_MACHINE_ROTATE 1
#define STATE_MACHINE_SKID_STEER 2
#define STATE_MACHINE_PICKUP 3
#define STATE_MACHINE_DROPOFF 4

#define PROCCESE_LOOP_SEARCHING 0
#define PROCCESE_LOOP_TARGETCOLLECTED 1

#define FAST_PID 0 //quickest turn reasponse time
#define SLOW_PID 1 //slower turn reasponse time
#define CONST_PID 2 //constant angular turn rate


int stateMachineState = STATE_MACHINE_TRANSFORM;

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
ros::Subscriber obstacleSubscriber;
ros::Subscriber odometrySubscriber;
ros::Subscriber mapSubscriber;


// Timers
ros::Timer stateMachineTimer;
ros::Timer publish_status_timer;
ros::Timer targetDetectedTimer;
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
void obstacleHandler(const std_msgs::UInt8::ConstPtr& message);
void odometryHandler(const nav_msgs::Odometry::ConstPtr& message);
void mapHandler(const nav_msgs::Odometry::ConstPtr& message);
void mobilityStateMachine(const ros::TimerEvent&);
void publishStatusTimerEventHandler(const ros::TimerEvent& event);
void targetDetectedReset(const ros::TimerEvent& event);
void publishHeartBeatTimerEventHandler(const ros::TimerEvent& event);

int main(int argc, char **argv) {
    
    gethostname(host, sizeof (host));
    string hostname(host);
    
    if (argc >= 2) {
        publishedName = argv[1];
        cout << "Welcome to the world of tomorrow " << publishedName
             << "!  Mobility turnDirectionule started." << endl;
    } else {
        publishedName = hostname;
        cout << "No Name Selected. Default is: " << publishedName << endl;
    }
    
    // NoSignalHandler so we can catch SIGINT ourselves and shutdown the node
    ros::init(argc, argv, (publishedName + "_MOBILITY"), ros::init_options::NoSigintHandler);
    ros::NodeHandle mNH;
    
    // Register the SIGINT event handler so the node can shutdown properly
    signal(SIGINT, sigintEventHandler);
    
    joySubscriber = mNH.subscribe((publishedName + "/joystick"), 10, joyCmdHandler);
    modeSubscriber = mNH.subscribe((publishedName + "/mode"), 1, modeHandler);
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
    heartbeatPublisher = mNH.advertise<std_msgs::String>((publishedName + "/mobility/heartbeat"), 1, true);
    
    publish_status_timer = mNH.createTimer(ros::Duration(status_publish_interval), publishStatusTimerEventHandler);
    stateMachineTimer = mNH.createTimer(ros::Duration(mobilityLoopTimeStep), mobilityStateMachine);
    targetDetectedTimer = mNH.createTimer(ros::Duration(0), targetDetectedReset, true);
    
    publish_heartbeat_timer = mNH.createTimer(ros::Duration(heartbeat_publish_interval), publishHeartBeatTimerEventHandler);
    
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
void mobilityStateMachine(const ros::TimerEvent&) {
    
    
    
    // Robot is in automode
    if (currentMode == 2 || currentMode == 3) {
        
        switch(stateMachineState) {
        
        
        //Handlers and the final state of STATE_MACHINE are the only parts allowed to call INTERUPT
        //This should be done as little as possible. I Suggest to Use timeouts to set control bools false.
        //Then only call INTERUPT if bool switches to true.
        case STATE_MACHINE_INTERUPT: {
            stateMachineMsg.data = "INTERUPT";
            
            if (procceseLoopState == PROCCESE_LOOP_SEARCHING) {
                if (targetsFound) {
                    PickUpResult result = pickUpController.run();
                    
                    if (result.changeBehaviour != " ") {
                        if (result.changeBehaviour == "target pickedup") {
                            procceseLoopState = PROCCESE_LOOP_TARGETCOLLECTED;
                        }
                        else if (result.changeBehaviour == "target lost") {
                            targetsFound = false;
                        }
                    }
                    else if (!result.waypoint) {
                        if (result.PID == FAST_PID){
                            fastPID(result.cmdVel,result.cmdError); 
                        }
                        else if (result.PID == SLOW_PID) {
                            slowPID(result.cmdVel,result.cmdError);
                        }
                        else if (result.PID == CONST_PID) {
                            constPID(result.cmdVel,result.cmdAngular);
                        }
                    }
                }
            }
            
            
            if (obstacleDetected) {
                
                if (result.changeBehaviour) {
                    
                }
                else if (result.waypoint) {
                    for (int i = result.pointCount; i > 0; i--) {
                        goalLocation.push_front(result.waypoint[i]);
                    }
                }
                else if (!result.waypoint) {
                    if (result.PID == FAST_PID){
                        fastPID(result.cmdVel,result.cmdError); 
                    }
                    else if (result.PID == SLOW_PID) {
                        slowPID(result.cmdVel,result.cmdError);
                    }
                    else if (result.PID == CONST_PID) {
                        constPID(esult.cmdVel,result.cmdAngular);
                    }
                }
                
            }
            
            if (procceseLoopState == PROCCESE_LOOP_TARGETCOLLECTED) {
                
                if (targetsCollected) {
                    PickUpResult result = pickUpController.run();
                    
                    if (result.changeBehaviour != " ") {
                        if (result.changeBehaviour == "target dropped") {
                            
                        }
                        else if (result.changeBehaviour == "target returned") {
                            
                        }
                    }
                    else if (!result.waypoint) {
                        if (result.PID == FAST_PID){
                            fastPID(result.cmdVel,result.cmdError); 
                        }
                        else if (result.PID == SLOW_PID) {
                            slowPID(result.cmdVel,result.cmdError);
                        }
                        else if (result.PID == CONST_PID) {
                            constPID(result.cmdVel,result.cmdAngular);
                        }
                    }
                }
            }
            
            if (waypointsAvalible) {
                
            }
            
            if (pathPlanningRequired) {
                
            }
            
            
        }
            
            // Calculate angle between currentLocation.theta and goalLocation.theta
            // Rotate left or right depending on sign of angle
            // Stay in this state until angle is minimized
        case STATE_MACHINE_ROTATE: {
            stateMachineMsg.data = "ROTATING";
            
            // Calculate the diffrence between current and desired heading in radians.
            float errorYaw = angles::shortest_angular_distance(currentLocation.theta, goalLocation.theta);
            
            // If angle > rotateOnlyAngleTolerance radians rotate but dont drive forward.
            if (fabs(angles::shortest_angular_distance(currentLocation.theta, goalLocation.theta)) > rotateOnlyAngleTolerance) {
                // rotate but dont drive.
                sendDriveCommand(0.0, errorYaw);
                break;
            } else {
                // move to differential drive step
                stateMachineState = STATE_MACHINE_SKID_STEER;
                //fall through on purpose.
            }
        }
            
            
            
            
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
}

void sendDriveCommand(double linearVel, double angularError)
{
    velocity.linear.x = linearVel,
            velocity.angular.z = angularError;
    
    // publish the drive commands
    driveControlPublish.publish(velocity);
}

/*************************
 * ROS CALLBACK HANDLERS *
 *************************/

void targetHandler(const apriltags_ros::AprilTagDetectionArray::ConstPtr& message) {
    
    targetsFound = pickUpController.sendData(message);
    
}

void modeHandler(const std_msgs::UInt8::ConstPtr& message) {
    currentMode = message->data;
    sendDriveCommand(0.0, 0.0);
}

void obstacleHandler(const std_msgs::UInt8::ConstPtr& message) {
    
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


void publishStatusTimerEventHandler(const ros::TimerEvent&) {
    std_msgs::String msg;
    msg.data = "online";
    status_publisher.publish(msg);
}


void targetDetectedReset(const ros::TimerEvent& event) {
    targetDetected = false;
    
    std_msgs::Float32 angle;
    angle.data = 0;
    
    // close fingers
    fingerAnglePublish.publish(angle);
    
    // raise wrist
    wristAnglePublish.publish(angle);
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

