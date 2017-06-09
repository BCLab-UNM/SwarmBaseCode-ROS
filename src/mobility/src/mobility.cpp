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
#include "PickUpController.h"
#include "DropOffController.h"
#include "SearchController.h"
#include "ObstacleController.h"

#include "StandardVars.h"
#include "PID.h"
#include <vector>

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
ObstacleController obstacleController;

// Mobility Logic Functions
void sendDriveCommand(double linearVel, double angularVel);
void openFingers(); // Open fingers to 90 degrees
void closeFingers();// Close fingers to 0 degrees
void raiseWrist();  // Return wrist back to 0 degrees
void lowerWrist();  // Lower wrist to 50 degrees
void resultHandler();


//PID configs************************
PIDConfig fastVelConfig();
PIDConfig fastYawConfig();

void fastPID(float vel,float yaw, float setPointVel, float setPointYaw);
PID fastVelPID(fastVelConfig());
PID fastYawPID(fastYawConfig());


// Numeric Variables for rover positioning
geometry_msgs::Pose2D currentLocation;
geometry_msgs::Pose2D currentLocationMap;
geometry_msgs::Pose2D currentLocationAverage;
vector<geometry_msgs::Pose2D> waypoints;

geometry_msgs::Pose2D centerLocation;
geometry_msgs::Pose2D centerLocationMap;
geometry_msgs::Pose2D centerLocationOdom;

int currentMode = 0;
const float mobilityLoopTimeStep = 0.1; // time between the mobility loop calls
const float status_publish_interval = 1;
const float heartbeat_publish_interval = 2;
const float waypointTolerance = 0.1; //10 cm tolerance.

// used for calling code once but not in main
bool initilized = false;

bool targetsFound = false;

bool obstacleDetected = false;

bool targetsCollected = false;

bool waypointsAvalible = false;

bool pathPlanningRequired = false;

float searchVelocity = 0.2; // meters/second

bool avoidingObstacle = false;

bool dropOffWayPoints = false;
bool dropOffPrecision = false;

Result result;


std_msgs::String msg;

// state machine states
enum StateMachineStates {
    STATE_MACHINE_INTERRUPT,
    STATE_MACHINE_ROTATE,
    STATE_MACHINE_SKID_STEER,
    STATE_MACHINE_PICKUP,
    STATE_MACHINE_DROPOFF,
    STATE_MACHINE_WAYPOINTS
};

enum ProcessLoopStates {
    PROCCESS_LOOP_SEARCHING,
    PROCCESS_LOOP_TARGET_PICKEDUP
};


int stateMachineState = STATE_MACHINE_INTERRUPT;
int procceseLoopState = PROCCESS_LOOP_SEARCHING;


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
void mobilityStateMachine(const ros::TimerEvent&);
void publishStatusTimerEventHandler(const ros::TimerEvent& event);
void publishHeartBeatTimerEventHandler(const ros::TimerEvent& event);
void sonarHandler(const sensor_msgs::Range::ConstPtr& sonarLeft, const sensor_msgs::Range::ConstPtr& sonarCenter, const sensor_msgs::Range::ConstPtr& sonarRight);



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
    heartbeatPublisher = mNH.advertise<std_msgs::String>((publishedName + "/mobility/heartbeat"), 1, true);
    
    publish_status_timer = mNH.createTimer(ros::Duration(status_publish_interval), publishStatusTimerEventHandler);
    stateMachineTimer = mNH.createTimer(ros::Duration(mobilityLoopTimeStep), mobilityStateMachine);
    
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
void mobilityStateMachine(const ros::TimerEvent&) {
    std_msgs::String stateMachineMsg;
    float rotateOnlyAngleTolerance = 0.4;
    
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
        
        switch(stateMachineState) {
        
        
        //Handlers and the final state of STATE_MACHINE are the only parts allowed to call INTERUPT
        //This should be d one as little as possible. I Suggest to Use timeouts to set control bools false.
        //Then only call INTERUPT if bool switches to true.
        case STATE_MACHINE_INTERRUPT: {
            stateMachineMsg.data = "INTERRUPT";
            
            if (procceseLoopState == PROCCESS_LOOP_SEARCHING) { //we will listen to this interupt section only when searching
                if (targetsFound) {
                    result = pickUpController.CalculateResult();
                    if (result.type == behavior) {
                        if (result.b == targetPickedUp) {
                            procceseLoopState = PROCCESS_LOOP_TARGET_PICKEDUP;
                            dropOffController.SetTargetPickedUp();
                        }
                        else if (result.b == targetLost) {
                            targetsFound = false;
                        }
                    }
                    else {
                        resultHandler();
                    }
                }
            }
            
            
            if (obstacleDetected) {
               result = obstacleController.CalculateResult();
                if (result.type == behavior) {
                    
                }
                else {
                    resultHandler();
                }                
            }
            
            if (procceseLoopState == PROCCESS_LOOP_TARGET_PICKEDUP) { //we will listen to this interupt section only when target collected
                
                if (targetsCollected) {
                    Result result = dropOffController.CalculateResult();
                    
                    if (result.type == behavior) {
                        if (result.b == targetDropped) {
                            procceseLoopState = PROCCESS_LOOP_SEARCHING;
                            targetsCollected = false;
                            dropOffWayPoints = false;
                             dropOffPrecision = false;
                        }
                        else if (result.b == targetReturned) {
                            procceseLoopState = PROCCESS_LOOP_SEARCHING;
                            targetsCollected = false;
                            dropOffWayPoints = false;
                        }
                    }
                    else {
                        resultHandler();
                    }
                }
            }
            
            if (waypointsAvalible) {
                stateMachineState = STATE_MACHINE_WAYPOINTS;
                
            }
            
            if (pathPlanningRequired) {
                //result = searchController.makeDecision();
                if (result.type == behavior) {

                }
                else {
                    resultHandler();
                }
            }
            
            
        }
            
            //Handles route planning and navigation as well as makeing sure all waypoints are valid.
        case STATE_MACHINE_WAYPOINTS: {
             stateMachineMsg.data = "MANAGE_WAYPOINTS";
                          
             bool toClose = true;
             while (!waypoints.empty() && toClose) {
                 if (hypot(waypoints.back().x-currentLocation.x, waypoints.back().y-currentLocation.y) < waypointTolerance) {
                     waypoints.pop_back();   
                 }
                 else {
                     toClose = false;
                 }
             }
             if (waypoints.empty()) {
                 stateMachineState = STATE_MACHINE_INTERRUPT;
                 waypointsAvalible = false;
                 break;
             }
             else {
                 waypoints.back().theta = atan2(waypoints.back().y - currentLocation.y, waypoints.back().x - currentLocation.x);
                 stateMachineState = STATE_MACHINE_ROTATE;
                 //fall through on purpose
             }
             
             
        }            
            
            // Calculate angle between currentLocation.theta and waypoints.front().theta
            // Rotate left or right depending on sign of angle
            // Stay in this state until angle is minimized
        case STATE_MACHINE_ROTATE: {
            stateMachineMsg.data = "ROTATING";
            
            // Calculate the diffrence between current and desired heading in radians.
            float errorYaw = angles::shortest_angular_distance(currentLocation.theta, waypoints.back().theta);
            
            // If angle > rotateOnlyAngleTolerance radians rotate but dont drive forward.
            if (fabs(angles::shortest_angular_distance(currentLocation.theta, waypoints.back().theta)) > rotateOnlyAngleTolerance) {
                // rotate but dont drive.
                sendDriveCommand(0.0, errorYaw);
                break;
            } else {
                // move to differential drive step
                stateMachineState = STATE_MACHINE_SKID_STEER;
                //fall through on purpose.
            }
        }
            // Calculate angle between currentLocation.x/y and waypoints.back().x/y
            // Drive forward
            // Stay in this state until angle is at least PI/2
        case STATE_MACHINE_SKID_STEER: {
            stateMachineMsg.data = "SKID_STEER";
            
            // calculate the distance between current and desired heading in radians
            float errorYaw = angles::shortest_angular_distance(currentLocation.theta, waypoints.back().theta);
            
            // goal not yet reached drive while maintaining proper heading.
            if (fabs(angles::shortest_angular_distance(currentLocation.theta, atan2(waypoints.back().y - currentLocation.y, waypoints.back().x - currentLocation.x))) < M_PI_2) {
                // drive and turn simultaniously
                sendDriveCommand(searchVelocity, errorYaw/2);
            }
            // goal is reached but desired heading is still wrong turn only
            else if (fabs(angles::shortest_angular_distance(currentLocation.theta, waypoints.back().theta)) > 0.1) {
                // rotate but dont drive
                sendDriveCommand(0.0, errorYaw);
            }
            else {
                // stop
                sendDriveCommand(0.0, 0.0);
                avoidingObstacle = false;
                
                // move back to transform step
                stateMachineState = STATE_MACHINE_WAYPOINTS;
            }
            
            break;
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
    
    dropOffController.SetLocationData(currentLocation, centerLocation); //send location data to dropOffController
    
    if (!dropOffWayPoints) { //check if we have triggered this interupt already if so ignore it
        dropOffWayPoints = dropOffController.ShouldInterrupt(); //trigger waypoints interupt for drop off flag is true
        stateMachineState = STATE_MACHINE_INTERRUPT;
        dropOffPrecision = false; //we cannot precision drive if we want to waypoint drive.
    }
    
    searchController.UpdateData(currentLocation, centerLocation);
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
    
    if (procceseLoopState == PROCCESS_LOOP_SEARCHING) {
        pickUpController.UpdateData(message); //send april tag data to pickUpController
        if (!targetsFound) {
            targetsFound = pickUpController.ShouldInterrupt(); //set this flag to indicate we found a target and thus handle the interupt
            stateMachineState = STATE_MACHINE_INTERRUPT;
        }
    }
    
    dropOffController.UpdateData(message); //send april tag data to dropOffController
    if (!dropOffPrecision) { //if we have called this interupt ignore it
        if (dropOffController.IsChangingMode()) {
            stateMachineState = STATE_MACHINE_INTERRUPT;
            dropOffWayPoints = false; //set this repeat hold flag false to allow a new interupt in the future
            dropOffPrecision = true; //set is repeat hold flag true to prevent multiple calls
        }
    }
    
}

void modeHandler(const std_msgs::UInt8::ConstPtr& message) {
    currentMode = message->data;
    sendDriveCommand(0.0, 0.0);
}

void sonarHandler(const sensor_msgs::Range::ConstPtr& sonarLeft, const sensor_msgs::Range::ConstPtr& sonarCenter, const sensor_msgs::Range::ConstPtr& sonarRight) {

    obstacleController.UpdateData(sonarLeft->range, sonarCenter->range, sonarRight->range, currentLocation);
    if (!obstacleDetected) {
    obstacleDetected = obstacleController.ShouldInterrupt();
    stateMachineState = STATE_MACHINE_INTERRUPT;
    }
    
    if (sonarCenter->range < 0.1) {
        pickUpController.SetUltraSoundData(true);
        dropOffController.SetBlockBlockingUltrasound(true);
    }
    else {
        pickUpController.SetUltraSoundData(false);
        dropOffController.SetBlockBlockingUltrasound(false);
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


void publishStatusTimerEventHandler(const ros::TimerEvent&) {
    std_msgs::String msg;
    msg.data = "online";
    status_publisher.publish(msg);
}

void sigintEventHandler(int sig) {
    // All the default sigint handler does is call shutdown()
    ros::shutdown();
}


void resultHandler()
{
        std_msgs::Float32 angle;
    if (result.wristAngle != -1) {
        angle.data = result.wristAngle;
        wristAnglePublish.publish(angle);
    }
    if (result.fingerAngle != -1) {
        angle.data = result.fingerAngle;
        fingerAnglePublish.publish(angle); 
    }
  
    if (result.type == waypoint) {
        if (!result.wpts.waypoints.empty()) {
            waypoints.insert(waypoints.end(),result.wpts.waypoints.begin(), result.wpts.waypoints.end());
            waypointsAvalible = true;
        }
    }
    else if (!result.type == waypoint) {
        if (result.PIDMode == FAST_PID){
            fastPID(result.pd.cmdVel,result.pd.cmdAngularError, result.pd.setPointVel, result.pd.setPointYaw); //needs declaration
        }
        else if (result.PIDMode == SLOW_PID) {
            //slowPID(result.cmdVel,result.cmdAngularError); needs declaration
        }
        else if (result.PIDMode == CONST_PID) {
            //constPID(result.cmdVel,result.cmdAngular); needs declaration
        }
    }      
}


void fastPID(float vel, float yaw , float setPointVel, float setPointYaw) {
    float velOut = fastVelPID.PIDOut(vel, setPointVel);
    float yawOut = fastYawPID.PIDOut(yaw, setPointYaw);
    
    int left = velOut - yawOut;
    int right = velOut + yawOut;
    
    int sat = 255;
    if (left  >  sat) {left  =  sat;}
    if (left  < -sat) {left  = -sat;}
    if (right >  sat) {right =  sat;}
    if (right < -sat) {right = -sat;}
    
    sendDriveCommand(left,right);
}

PIDConfig fastVelConfig() {
    PIDConfig config;
    
    config.Kp = 140;
    config.Ki = 20;
    config.Kd = 15;
    config.satUpper = 255; 
    config.satLower = -255; 
    config.antiWindup = config.satUpper/2; 
    config.errorHistLength = 4;
    config.alwaysIntegral = false; 
    config.resetOnSetpoint = true;
    config.feedForwardMultiplier = 320; //gives 127 pwm at 0.4 commandedspeed
    config.integralDeadZone = 0.01;
    config.integralErrorHistoryLength = 10000;
    config.integralMax = config.satUpper/2;
    
    return config;
    
}

PIDConfig fastYawConfig() {
    PIDConfig config;
    
    config.Kp = 200;
    config.Ki = 15;
    config.Kd = 15;
    config.satUpper = 255; 
    config.satLower = -255; 
    config.antiWindup = config.satUpper/2; 
    config.errorHistLength = 4;
    config.alwaysIntegral = false; 
    config.resetOnSetpoint = true;
    config.feedForwardMultiplier = 320; //gives 127 pwm at 0.4 commandedspeed
    config.integralDeadZone = 0.01;
    config.integralErrorHistoryLength = 10000;
    config.integralMax = config.satUpper/2;
    
    return config;
    
}


void publishHeartBeatTimerEventHandler(const ros::TimerEvent&) {
    std_msgs::String msg;
    msg.data = "";
    heartbeatPublisher.publish(msg);
}

