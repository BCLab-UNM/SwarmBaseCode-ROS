/*
 * Author: Matthew W. Nugent
 * Maintainer: Matthew W. Nugent
 * Email: matthew.w.nugent@nasa.gov 
 * NASA Center: Kennedy Space Center
 * Mail Stop: NE-C1
 * 
 * Project Name: Swarmie Robotics NASA Center Innovation Fund
 * Principal Investigator: Kurt Leucht
 * Email: kurt.leucht@nasa.gov
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
 * -Digital Fence COMPLETE 9/5/2014 MWN
 * -Go Home COMPLETE 9/5/2014 MWN
 * -Directed Walk COMPLETE 9/11/2014 MWN
 * -Target Harvest Service COMPLETE 9/12/2014 MWN
 * -Move hardcoded params to UI or param server COMPLETE 9/15/2014 MWN
 * -Site Decision (fidelity, lay pheromone) COMPLETE 9/15/2014 MWN
 */

/*TODO:
 * -Onboard Pheromone (implementation & ignore chance)
 * -investigate map cell to meter in mapping  
 * -change stuck checker to IMU data? and onTargetDegrees?
 * -change walk duration to IMU based distance vs time?
 * -check last state and only publish if value is new
 * -make as much as possible event driven, not time driven (obstacle detect and mapping)
 */

#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <sstream>
#include <vector>
#include <math.h>

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/UInt64.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Pose2D.h>
#include "rover_onboard_target_detection/ATag.h"
#include "rover_onboard_target_detection/harvest.h"

using namespace std;

//Mobility Logic Functions
void move();
int avoidObstacle(int mode);
void processTarget();
void increaseFrenzy();
void sootheFrenzy();
void fullStop();
void setLinearSpeed(float speed);
void setRotationSpeed(float speed);
bool onTargetDegrees(float myCurrentHeading, float requestedHeading);
void depositPheromones(bool depositPheromones);

//Utility Functions
float randProb();
int randSign();
float normalRandom(float mean, float stdDev);
float exponentialDecay(float quantity, float time, float lambda);
float exponentialGrowth(float quantity, float time, float lambda);
float constrain(float value, float minVal, float maxVal);
float convertToHeadingDegrees(float requestedHeading);
float angleBetweenHeadings(float a1, float a2);
float rad2deg(float radian);
float deg2rad(float degree);

//ROS Timers
void randomWalk(const ros::TimerEvent&);
void stuckChecker(const ros::TimerEvent&);
void loadParameters(const ros::TimerEvent&);

//ROS Callback Handler Functions
void poseHandler(const geometry_msgs::Pose2D::ConstPtr& message);
void locationHandler(const geometry_msgs::Pose2D::ConstPtr& message);
void joyCmdHandler(const sensor_msgs::Joy::ConstPtr& message);
void modeHandler(const std_msgs::UInt8::ConstPtr& message);
void parameterHandler(const std_msgs::Float32MultiArray message);
void linearVelHandler(const std_msgs::Float32 message);
void angularVelHandler(const std_msgs::Float32 message);
void targetHandler(const rover_onboard_target_detection::ATag tagInfo);
void leftHandler(const sensor_msgs::Range::ConstPtr& message);
void centerHandler(const sensor_msgs::Range::ConstPtr& message);
void rightHandler(const sensor_msgs::Range::ConstPtr& message);
void setMode();

//Numeric Variables
float desiredHeading = 0;
float headingDifferential = 0;
float lastHeadingDifferential = 360;
float lastPosX = 0.0;
float lastPosY = 0.0;
float targetPosX = 0.0;
float targetPosY = 0.0;
float targetStartHeading = 0;
uint overshootAttempts = 0;
uint numTargetsDetected = 0;
uint targetDetected = 0;
uint currentMode = 0;
int turnDirection = 1; //intended turn direction
int differentialDirection; //actual direction to new heading
float searchCorrelation = 0;
float leftVal = 500.0;
float centerVal = 500.0;
float rightVal = 500.0;
float currentHeading = 0;
float currentPosX = 0;
float currentPosY = 0;
float polarRadius = 0;
float polarTheta = 0;
double pheromDecay;
double uninformedCorr;
double informedCorr;
double travelGiveUp;
double searchGiveUp;
double pheromLaying;
double chargeLeave;
double chargeReturn;
double siteFidelity;
float searchTime = 0; //counts the number of times through main loop
float walkingTime = 0; //tracks how long the robot has been walking this leg
float frenzyTimer = 0.0; //how long the robot has been failing to do what it wants
float frenzyLevel = 0.0; //how frantic the robot is while trying to clear obstacles
float obstacleActionTimer = 0; //how long robot has been attempting an obstacle avoidance action
float attemptedTurnTimer = 0; //how long the robot has been attempting the current turn

//Status Flags
bool isTraveling = true;
bool isHeadingToTarget = false;
bool isGoingHome = false;
bool isHome = true;
bool finishedTurning = true;
bool finishedWalking = true;
bool finishedAvoidingObstacle = true;
bool isProcessingTargets = false;
bool completedHalfSpin = false;
bool finishedObstacleAction = true;
bool wantsSiteFidelity = false; //robot wants to go back to a site to improve the fidelity of the information about that site

//Default hardcoded parameters, consider moving these to be UI driven or from Parameter Server
float turnAccuracy = 3; //tolerance needed to "match" new heading
float senseRange = .25; //meters the ultrasonic detectors look for obstacles
float digitalFenceRadius = 100; //how many meters the robot is allowed to travel from home
float walkingEndTime = 2.0; //how long to walk each random walk segment
float walkingSpeed = 0.3; //how fast to walk each random walk segment
float turningSpeed = 0.2; //how fast to turn each random walk segment
float frenzyBuildRate = 0.07; //how quickly the robot gets distressed if it can't clear an obstacle
float obstacleActionTimeLimit = 5; //try each obstacle avoidance action for this many seconds
float linearStuckTolerance = 5; //Robot is "stuck" if it hasn't moved this many meters within stuckTime
float foundTargetTolerance = 3; //how many meters away robot can be and still have "arrived" at target
float turnTimeLimit = 120; //how long a robot will attempt a turn before giving up, in seconds

float stepTimer = 0.25; //length of step in random walk (s)
float stuckTime = 10.0; //number of seconds delay to check if the robot is "stuck"
float parameterTime = 1.0; //how often to check for new parameters

//ROS Data Types
geometry_msgs::Twist mobility;
std_msgs::UInt8 obstacleMode;
char host[128];
string publishedName;

//ROS Components
ros::Subscriber joySubscriber;
ros::Subscriber modeSubscriber;
ros::Subscriber parameterSubscriber;
ros::Subscriber linearVelSubscriber;
ros::Subscriber angularVelSubscriber;
ros::Subscriber targetSubscriber;
ros::Subscriber poseSubscriber;
ros::Subscriber locationSubscriber;
ros::Subscriber leftSubscriber;
ros::Subscriber centerSubscriber;
ros::Subscriber rightSubscriber;
ros::Publisher obstaclePublish;
ros::Publisher mobilityPublish;
ros::Publisher pheromonePublish;
ros::Timer walkTimer;
ros::Timer stuckTimer;
ros::Timer parameterTimer;
ros::ServiceClient harvestClient;
rover_onboard_target_detection::harvest harvestMessage;

int main(int argc, char **argv) {

    gethostname(host, sizeof (host));
    string hostname(host);

    srand(time(NULL));

    ros::init(argc, argv, (hostname + "_MOBILITY"));

    ros::NodeHandle mNH;

    if (argc >= 2) {
        publishedName = argv[1];
        cout << "Welcome to the world of tomorrow " << publishedName << "!  Mobility module started." << endl;
    } else {
        publishedName = hostname;
        cout << "No Name Selected. Default is: " << publishedName << endl;
    }

    ros::TimerEvent actNow;
    loadParameters(actNow);
    
    joySubscriber = mNH.subscribe((publishedName + "/joystick"), 10, joyCmdHandler);
    modeSubscriber = mNH.subscribe((publishedName + "/mode"), 10, modeHandler);
    parameterSubscriber = mNH.subscribe((publishedName + "/parameters"), 1, parameterHandler);
    linearVelSubscriber = mNH.subscribe((publishedName + "/linearVel"), 1, linearVelHandler);
    angularVelSubscriber = mNH.subscribe((publishedName + "/angularVel"), 1, angularVelHandler);
    targetSubscriber = mNH.subscribe((publishedName + "/targets"), 1, targetHandler);
    
    leftSubscriber = mNH.subscribe((publishedName + "/USLeft"), 10, leftHandler);
    centerSubscriber = mNH.subscribe((publishedName + "/USCenter"), 10, centerHandler);
    rightSubscriber = mNH.subscribe((publishedName + "/USRight"), 10, rightHandler);
    
    poseSubscriber = mNH.subscribe((publishedName + "/pose2d"), 10, poseHandler);
    locationSubscriber = mNH.subscribe((publishedName + "/location"), 10, locationHandler);

    mobilityPublish = mNH.advertise<geometry_msgs::Twist>((publishedName + "/mobility"), 100);
    obstaclePublish = mNH.advertise<std_msgs::UInt8>((publishedName + "/obstacle"), 10);
    pheromonePublish = mNH.advertise<std_msgs::Bool>((publishedName + "/pheromone"), 100);
        
    walkTimer = mNH.createTimer(ros::Duration(stepTimer), randomWalk);
    stuckTimer = mNH.createTimer(ros::Duration(stuckTime), stuckChecker);
    parameterTimer = mNH.createTimer(ros::Duration(parameterTime), loadParameters);

    harvestClient = mNH.serviceClient<rover_onboard_target_detection::harvest>("harvestSrv");

    while (ros::ok()) {
       ros::spin();
    }

    return EXIT_SUCCESS;
}

/***********************
 * MOBILITY LOGIC
************************/

void randomWalk(const ros::TimerEvent&) {
    if(currentMode == 2 || currentMode == 3 ) { //Robot is in automode
        if(finishedAvoidingObstacle) {        
            //If at home, robot is just starting out, pick a direction and go
            if(isHome) {
                if(wantsSiteFidelity) {
                    isHeadingToTarget = true;
                    wantsSiteFidelity = false;
                } else {
                    isTraveling = true;
                }
                searchTime = 0;
                isHome = !isHome;
                depositPheromones(false);
            }

           if(isGoingHome) {
                //if robot is headed home, check if robot has reached home
                if(polarRadius < foundTargetTolerance) {
                    cout << endl << "Robot made it home" << endl;
                    isGoingHome = false;
                    isHome = true;
                }
            } else {
               //robot is not going home, decide if robot should go home
                if(randProb() < searchGiveUp || randProb() < chargeReturn) {
                    cout << endl << "Robot is going home" << endl;
                    isGoingHome = true;
                }
           }

           if(isHeadingToTarget) {
               //check if robot has reached Target
               if( abs(targetPosX - currentPosX) < foundTargetTolerance && abs(targetPosY - currentPosY) < foundTargetTolerance )
                   cout << endl << "Robot has reached Target coordinates" << endl;
                   isHeadingToTarget = false;
                   searchTime = 0;
           }

           if(isTraveling) {
               //if Traveling, decide when to give up Traveling
               if(finishedTurning) {isTraveling = (randProb() > travelGiveUp);}//dont check param till first turn is done
               cout << "Still Traveling" << endl;
               searchTime = 0;
           } else {
               //Robot is done Traveling, respond to Targets
               if(numTargetsDetected > 0 && !isGoingHome && !isHeadingToTarget) {
                   //Target was found, kick out to process target
                   processTarget();
               }
           }

           if(!isProcessingTargets) { move(); }
           sootheFrenzy();
        } else {
           increaseFrenzy();
        }
    } 
    
    searchTime = searchTime + stepTimer;
    obstacleActionTimer = obstacleActionTimer + stepTimer;
}

void move() {
    if(finishedTurning && finishedWalking){
        //this is a new turn, calculate a new direction
        cout << endl << "New movement: ";
        overshootAttempts = 0;
        finishedTurning = false;
        lastHeadingDifferential = 360;

        //Decide which direction to go
        if(isGoingHome) {
            cout << "Headed home" << endl;
            desiredHeading = polarTheta + 180;
            searchCorrelation = uninformedCorr;
        } else if (polarRadius > digitalFenceRadius) {
            cout << "Robot is outside digital fence" << endl;
            desiredHeading = polarTheta + 180;
            searchCorrelation = 20;
        } else if(isTraveling) {
            cout << "Still Traveling" << endl;
            desiredHeading = currentHeading;
            searchCorrelation = uninformedCorr;
        } else if(isHeadingToTarget) {
            cout << "Seeking Target" << endl;
            desiredHeading = atan2(targetPosX - currentPosX, targetPosY - currentPosY);
            searchCorrelation = uninformedCorr;
        } else {
            cout << "Random Turn" << endl;
            desiredHeading = currentHeading;
            searchCorrelation = uninformedCorr + exponentialDecay(720.0, searchTime, informedCorr);
        }

        //as frenzy increases, robot cares less about getting somewhere specific. Increase randomness of desired heading.
        searchCorrelation = searchCorrelation + (180 * frenzyLevel);
        searchCorrelation = constrain(searchCorrelation, 0, 180);
        desiredHeading = normalRandom(desiredHeading, searchCorrelation); //randomize desired heading
        desiredHeading = convertToHeadingDegrees(desiredHeading); //convert heading to proper 360 degree form
        headingDifferential = angleBetweenHeadings(currentHeading, desiredHeading);
        turnDirection = headingDifferential/abs(headingDifferential);

        cout << "Frenzy Level: " << frenzyLevel << ", Search Correlation: " << searchCorrelation << endl;
        cout << "Current Heading: " << currentHeading << ", Desired Heading: " << desiredHeading << endl;        
        setRotationSpeed(turnDirection * turningSpeed);
    }

    if(finishedWalking) {
        //turn is in progress
        if(onTargetDegrees(currentHeading, desiredHeading) || overshootAttempts >= 3 ) {
            //stop turning when robot is pointing the right way and start walking
            cout << "Finished Turn, Start Linear Walk" << endl;
            finishedTurning = true;
            finishedWalking = false;
            setLinearSpeed(walkingSpeed);
        } else {
            //keep turning but correct for overshoot, only attemp a correction 3 times
            headingDifferential = angleBetweenHeadings(currentHeading, desiredHeading);
            differentialDirection = headingDifferential/abs(headingDifferential);
            if(differentialDirection != turnDirection){
                //robot overshot and missed the desired heading, turn back
                overshootAttempts++;
                turnDirection = -1 * turnDirection;
                setRotationSpeed(turnDirection * turningSpeed);
                cout << "Overshoot Attempt " << overshootAttempts << endl;
            }
        }
    } else {
        //walk is in progress
        walkingTime = walkingTime + stepTimer;
        if(walkingTime >= walkingEndTime && !isTraveling) {
            //stop walking after walkingEndTime seconds
            finishedWalking = true;
            walkingTime = 0;
        }
    }
}

int avoidObstacle( int mode ){
    //cout << endl << "Obstacle Detected" << endl;
    if(mode == 1) {
        //obstacle cleared
        cout << "Obstacle Cleared" << endl;
        fullStop();
        setLinearSpeed(walkingSpeed);
        finishedAvoidingObstacle = true;
        finishedObstacleAction = true;
        obstacleActionTimer = 0;
    } else {
        //still seeing obstacle
        finishedAvoidingObstacle = false;
                
        if(finishedObstacleAction) {
            //start a new action to clear obstacle
            finishedObstacleAction = false;
            obstacleActionTimer = 0;

            //bias toward attempting turns but build up to a 25/75 shot of walking vs turning at full frenzy level
            if(randProb() < (frenzyLevel / 4)) {
                //walk, tending towards backwards away from obstacle
                setLinearSpeed(-walkingSpeed);
            } else {
                //turn
                if (mode == 2 || mode == 3 || mode == 5 || mode == 7 || mode == 8 ) {
                    //Obstacles are on the left, turn right
                    cout << "Turning right to avoid Obstacle" << endl;
                    setRotationSpeed(-turningSpeed);
                } else {
                    //Obstacles are on the right, turn left
                    cout << "Turning left to avoid Obstacle" << endl;
                    setRotationSpeed(turningSpeed);
                }
            }
        }

        //attempt an obstacle avoidance action for a duration that is biased towards shorter times as frenzy level increases
        if(obstacleActionTimer >= constrain( normalRandom(obstacleActionTimeLimit*(1-frenzyLevel), obstacleActionTimeLimit*frenzyLevel), 1, obstacleActionTimeLimit) ){
            finishedObstacleAction = true;
        }
    }
}

void processTarget(){
    //if this is a new target do the below stuff
    if(!isProcessingTargets){
        cout << endl << "New Target Detected, start Sweep" << endl;
        fullStop();

        //request target detection module to mark this tag as harvested
        harvestMessage.request.tagToHarvest = targetDetected;
        harvestClient.call(harvestMessage);

        isProcessingTargets = true;
        finishedTurning = false;
        completedHalfSpin = false;
        targetStartHeading = currentHeading;
        desiredHeading = convertToHeadingDegrees(targetStartHeading + 180);
        setRotationSpeed(randSign() * turningSpeed);
    }
     
    if(onTargetDegrees(currentHeading,desiredHeading)) {
        if(!completedHalfSpin) {
            //robot has spun halfway, contine to 360 degrees
            completedHalfSpin = true;
            desiredHeading = targetStartHeading;
        } else {
            //robot has spun a full circle
            cout << "Finished Target Sweep" << endl;
            isProcessingTargets = false;
            fullStop();

            //decide if robot wants site fidelity, or should lay pheromone trail on way home and go home
            isGoingHome = true;
            if( randProb() < siteFidelity) {
                cout << "Robot wants site fidelity" << endl;
                wantsSiteFidelity = true;
                targetPosX = currentPosX;
                targetPosY = currentPosY;
            } else if( randProb() < pheromLaying) {
                cout << "Robot will place pheromones on way home" << endl;
                depositPheromones(true);
            }
        }
    }
    
    headingDifferential = angleBetweenHeadings(currentHeading, desiredHeading);        
}

void stuckChecker(const ros::TimerEvent&) {
    if((currentMode == 2 || currentMode == 3) && finishedAvoidingObstacle ) { //If robot is in auto mode
        bool stuck = false;
        
        if(finishedTurning) {
            if((abs(currentPosX - lastPosX) < linearStuckTolerance) || (abs(currentPosY - lastPosY) < linearStuckTolerance)) {
            cout << "Stuck while walking" << endl;
            stuck = true;
            }
        }

        if(finishedWalking){
            if(abs(lastHeadingDifferential) <= abs(headingDifferential)) {
                cout << "Stuck while turning. Last Differential: " << lastHeadingDifferential << ", Current Differential: " << headingDifferential << endl;
                stuck = true;
            }
        }
        
        lastHeadingDifferential = headingDifferential;
        lastPosX = currentPosX;
        lastPosY = currentPosY;
        
        if(stuck) {
            for(int i=0; i < stuckTime/stepTimer; i++) {
                increaseFrenzy();
            }
        } 
    }
}

void increaseFrenzy() {
    if(frenzyLevel < 1) { frenzyTimer = frenzyTimer + stepTimer; }
    frenzyLevel = constrain( exponentialGrowth(0.01, frenzyTimer, frenzyBuildRate), 0, 1);
}

void sootheFrenzy() {
    frenzyTimer = max((float) 0 , frenzyTimer - stepTimer);
    frenzyLevel = constrain( exponentialGrowth(0.01, frenzyTimer, frenzyBuildRate), 0, 1);
}

void setRotationSpeed(float speed) {
    mobility.linear.x = 0;
    mobility.angular.z = 0;
    mobilityPublish.publish(mobility);
    mobility.linear.x = 0;
    speed = constrain(normalRandom(speed, frenzyLevel*2),-1,1);
    mobility.angular.z = speed;
    mobilityPublish.publish(mobility);
    finishedWalking = true;
    finishedTurning = false;
    cout << "Set Rotation Speed " << speed << endl;
}

void setLinearSpeed(float speed) {
    mobility.linear.x = 0;
    mobility.angular.z = 0;
    mobilityPublish.publish(mobility);
    speed = constrain(normalRandom(speed, frenzyLevel*2),-1,1);
    mobility.linear.x = speed;
    mobility.angular.z = 0;
    mobilityPublish.publish(mobility);
    finishedWalking = false;
    finishedTurning = true;
    cout << "Set Linear Speed " << speed << endl;
 }

void fullStop() {
    cout << "Full Stop" << endl;
    finishedWalking = true;
    finishedTurning = true;
    mobility.linear.x = 0;
    mobility.angular.z = 0;
    mobilityPublish.publish(mobility);
}

bool onTargetDegrees(float myCurrentHeading, float requestedHeading) {
    float headingUpperLimit = requestedHeading + turnAccuracy;
    float headingLowerLimit = requestedHeading - turnAccuracy;
    float head1 = myCurrentHeading;
    float head2 = myCurrentHeading + 360;
    float head3 = myCurrentHeading - 360;

    bool onTarget = (head1 <= headingUpperLimit && head1 >= headingLowerLimit)
            || (head2 <= headingUpperLimit && head2 >= headingLowerLimit)
            || (head3 <= headingUpperLimit && head3 >= headingLowerLimit);

    onTarget = onTarget || attemptedTurnTimer > turnTimeLimit;

    attemptedTurnTimer = onTarget ? 0 : (attemptedTurnTimer + stepTimer);

    return onTarget;
}

void depositPheromones(bool deposit) {
    std_msgs::Bool pheromoneMessage;
    pheromoneMessage.data = deposit;
    pheromonePublish.publish(pheromoneMessage);
}

/***********************
 * UTILITY FUNCTIONS
************************/

float randProb() {
    return (float)rand()/(float)RAND_MAX;
}

int randSign() {
    return rand() % 2 * 2 -1;
}

float normalRandom(float mean, float stdDev) {
    static bool normal_is_valid = false;
    static float normal_x;
    static float normal_y;
    static float normal_rho;
    if(!normal_is_valid)
    {
      normal_x = randProb();
      normal_y = randProb();
      normal_rho = sqrt(-2. * log(1.0-normal_y));
      normal_is_valid = true;
    }
    else
      normal_is_valid = false;

    if(normal_is_valid)
      return normal_rho*cos(2.*M_PI*normal_x)*stdDev+mean;
    else
      return normal_rho*sin(2.*M_PI*normal_x)*stdDev+mean;
}

float exponentialDecay(float quantity, float time, float lambda) {
    return (quantity * exp(-lambda*time));
}

float exponentialGrowth(float quantity, float time, float lambda) {
    return (quantity * exp(lambda*time));
}

float constrain(float value, float minVal, float maxVal) {
    return max(minVal,(float) min(maxVal,value));
}

float convertToHeadingDegrees(float requestedHeading) {
    if (requestedHeading > 360) { requestedHeading = requestedHeading - 360;}
    if (requestedHeading < 0) { requestedHeading = 360 + requestedHeading;}
    return requestedHeading;
}

float angleBetweenHeadings(float a1, float a2) {
    float opt1 = (a1-a2)<0?a1-a2+360.0:a1-a2 ;
    float opt2 = (a2-a1)<0?a2-a1+360.0:a2-a1 ;
    float sign = (opt1-opt2)<0?1.0:-1.0;
    return sign * min(opt1, opt2);

}

float rad2deg(float radian){
    return (radian * (180/M_PI));
}

float deg2rad(float degree){
    return (degree * (M_PI/180));
}

/***********************
 * ROS CALLBACK HANDLERS
************************/

void targetHandler(const rover_onboard_target_detection::ATag tagInfo) {
    numTargetsDetected = tagInfo.tagsFound;
    if(tagInfo.tagID.size() > 0) {
        targetDetected = *tagInfo.tagID.begin();
    } else {
        targetDetected = 0;
    }
}

void poseHandler(const geometry_msgs::Pose2D::ConstPtr& message) {
    currentHeading = (float) message->theta;
}

void locationHandler(const geometry_msgs::Pose2D::ConstPtr& message) {
    currentPosX = (float) message->x;
    currentPosY = (float) message->y;
    polarRadius = sqrt(currentPosX*currentPosX + currentPosY*currentPosY);
    polarTheta = rad2deg(atan2(currentPosX, currentPosY));
    polarTheta = (polarTheta < 0) ? (360 + polarTheta) : polarTheta; //correct the quadrant
}

void linearVelHandler(const std_msgs::Float32 message) {
    walkingSpeed = message.data;
}

void angularVelHandler(const std_msgs::Float32 message) {
    turningSpeed = message.data;
}

void modeHandler(const std_msgs::UInt8::ConstPtr& message) {
    currentMode = message->data;
    fullStop();
}

void parameterHandler(const std_msgs::Float32MultiArray message) {
    pheromDecay = message.data.at(0);
    uninformedCorr = message.data.at(1);
    informedCorr = message.data.at(2);
    travelGiveUp = message.data.at(3);
    searchGiveUp = message.data.at(4);
    pheromLaying = message.data.at(5);
    chargeLeave = message.data.at(6);
    chargeReturn = message.data.at(7);
    siteFidelity = message.data.at(8);
}

void leftHandler(const sensor_msgs::Range::ConstPtr& message) {
    leftVal = message->range;
    setMode();
}

void centerHandler(const sensor_msgs::Range::ConstPtr& message) {
    centerVal = message->range;
    setMode();
}

void rightHandler(const sensor_msgs::Range::ConstPtr& message) {
    rightVal = message->range;
    setMode();
}

int lastObstacleMode;
    
void setMode() {
    
    if (leftVal > senseRange && centerVal < senseRange && rightVal > senseRange) {
        obstacleMode.data = 2;
    } else if (leftVal < senseRange && centerVal > senseRange && rightVal > senseRange) {
        obstacleMode.data = 3;
    } else if (leftVal > senseRange && centerVal > senseRange && rightVal < senseRange) {
        obstacleMode.data = 4;
    } else if (leftVal < senseRange && centerVal < senseRange && rightVal > senseRange) {
        obstacleMode.data = 5;
    } else if (leftVal > senseRange && centerVal < senseRange && rightVal < senseRange) {
        obstacleMode.data = 6;
    } else if (leftVal < senseRange && centerVal > senseRange && rightVal < senseRange) {
        obstacleMode.data = 7;
    } else if (leftVal < senseRange && centerVal < senseRange && rightVal < senseRange) {
        obstacleMode.data = 8;
    } else {
        obstacleMode.data = 1;
    }
    
    if(obstacleMode.data != lastObstacleMode) { 
        //only Publish if value has changed
        obstaclePublish.publish(obstacleMode);
        finishedAvoidingObstacle = false;
        avoidObstacle(obstacleMode.data);
        lastObstacleMode = obstacleMode.data;
    }
}

void loadParameters(const ros::TimerEvent&) {
/*
 * Load parameters from parameter server. ROS doesn't seem to follow latest C++11
 * standard so while it says it supports floats, it doesn't because floats 
 * can't be passed to templates in C++11. Doubles can, so passing everything as
 * a double and casting it back to a float.
 * 
 * Or I just can't figure it out....
 * 
 */    
    double param;

    ros::param::param((publishedName + "/mobility/turnAccuracy"), param, (double) turnAccuracy); turnAccuracy = (float) param;
    ros::param::param((publishedName + "/mobility/senseRange"), param, (double) senseRange); senseRange = (float) param;
    ros::param::param((publishedName + "/mobility/digitalFenceRadius"), param, (double) digitalFenceRadius); digitalFenceRadius = (float) param;
    ros::param::param((publishedName + "/mobility/stepTimer"), param, (double) stepTimer); stepTimer = (float) param;
    ros::param::param((publishedName + "/mobility/stuckTime"), param, (double) stuckTime); stuckTime = (float) param;
    ros::param::param((publishedName + "/mobility/walkingEndTime"), param, (double) walkingEndTime); walkingEndTime = (float) param;
    //ros::param::param((publishedName + "/mobility/walkingSpeed"), param, (double) walkingSpeed); walkingSpeed = (float) param;
    //ros::param::param((publishedName + "/mobility/turningSpeed"), param, (double) turningSpeed); turningSpeed = (float) param;
    ros::param::param((publishedName + "/mobility/frenzyBuildRate"), param, (double) frenzyBuildRate); frenzyBuildRate = (float) param;
    ros::param::param((publishedName + "/mobility/obstacleActionTimeLimit"), param, (double) obstacleActionTimeLimit); obstacleActionTimeLimit = (float) param;
    ros::param::param((publishedName + "/mobility/linearStuckTolerance"), param, (double) linearStuckTolerance); linearStuckTolerance = (float) param;
    ros::param::param((publishedName + "/mobility/foundTargetTolerance"), param, (double) foundTargetTolerance); foundTargetTolerance = (float) param;
    ros::param::param((publishedName + "/mobility/turnTimeLimit"), param, (double) turnTimeLimit); turnTimeLimit = (float) param;
}

void joyCmdHandler(const sensor_msgs::Joy::ConstPtr& message) {
    if(currentMode == 0){
        //Manual mode with Simulated Bot, added gain to compenste for slow Sim
        mobility.angular.z = message->axes[0] * 7;
        mobility.linear.x = message->axes[1] * 2;
        mobilityPublish.publish(mobility);
    } else if(currentMode == 1){
        //Manual mode with Physical Bot
        mobility.angular.z = message->axes[0];
        mobility.linear.x = message->axes[1];
        mobilityPublish.publish(mobility);
    } else{
        //Auto modes or other
        //Do nothing, i.e. do not publish mobility commands based on joystick messages if in Auto
        //This empty case is left in place for adding other potential future manual modes
    }
}


