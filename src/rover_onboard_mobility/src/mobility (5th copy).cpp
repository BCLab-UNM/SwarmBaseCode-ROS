/*
 * CONFIDENTIAL: Please contact http://iant.cs.unm.edu
 *
 * Author: Matthew W. Nugent & Kurt W. Leucht
 * Maintainer: Kurt W. Leucht
 * Email: kurt.leucht@nasa.gov 
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
 * CONFIDENTIAL: Please contact http://iant.cs.unm.edu
 * 
 * Revision Log:
 * -Digital Fence COMPLETE 9/5/2014 MWN
 * -Go Home COMPLETE 9/5/2014 MWN
 * -Directed Walk COMPLETE 9/11/2014 MWN
 * -Target Harvest Service COMPLETE 9/12/2014 MWN
 * -Move hardcoded params to UI or param server COMPLETE 9/15/2014 MWN
 * -Site Decision (fidelity, lay pheromone) COMPLETE 9/15/2014 MWN 
 * -Onboard Pheromone (implementation & ignore chance) COMPLETE 9/17/2014 MWN 
 * -Added state machine to keep things from happening when they are not supposed to 11/20/2014 KWL
 */

/*TODO:
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
#include <set>

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/UInt64.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Pose2D.h>
#include "rover_onboard_target_detection/ATag.h"
#include "rover_onboard_target_detection/harvest.h"
#include "rover_driver_world_state/trackPheromone.h"

using namespace std;

//Mobility Logic Functions
void move();
int avoidObstacle();
void processTarget();
void increaseFrenzy();
void sootheFrenzy();
void resetFrenzy();
void fullStop();
void setLinearSpeed(float speed);
void setRotationSpeed(float speed);
bool onTargetDegrees(float myCurrentHeading, float requestedHeading);
void depositPheromones(bool deposit);
void lookForPheromones();

//Utility Functions
float randProb();
int randSign();
float normalRandom(float mean, float stdDev);
float poissonCDF(float k, float lambda);
float exponentialDecay(float quantity, float time, float lambda);
float exponentialGrowth(float quantity, float time, float lambda);
float constrain(float value, float minVal, float maxVal);
float normalizeHeading(float heading);
float angleBetweenHeadings(float a1, float a2);
float rad2deg(float radian);
float deg2rad(float degree);

//ROS Timers
void mobilityStateMachine(const ros::TimerEvent&);
void probabilityLoop(const ros::TimerEvent&);
void stuckChecker(const ros::TimerEvent&);
void loadParameters(const ros::TimerEvent&);

//ROS Callback Handler Functions
//void poseHandler(const geometry_msgs::Pose2D::ConstPtr& message);
void realLocHandler(const geometry_msgs::Pose2D::ConstPtr& message);
void imageLocHandler(const geometry_msgs::Pose2D::ConstPtr& message);
void joyCmdHandler(const sensor_msgs::Joy::ConstPtr& message);
void modeHandler(const std_msgs::UInt8::ConstPtr& message);
void pheromoneHeadingHandler(const std_msgs::Float32 message);
void parameterHandler(const std_msgs::Float32MultiArray message);
void obstacleHandler(const std_msgs::UInt8::ConstPtr& message);
void linearVelHandler(const std_msgs::Float32 message);
void angularVelHandler(const std_msgs::Float32 message);
void targetHandler(const rover_onboard_target_detection::ATag tagInfo);
void setMode();

//Numeric Variables
float desiredHeading = 0;
float lastHeading = 0;
float headingDifferential = 0;
//float lastHeadingDifferential = 360;
float travelHeading = 0; // this is the heading the robot has decided to travel in
float lastPosX = 0.0; // old position for use by stuck checker
float lastPosY = 0.0; // old position for use by stuck checker
float odomPosX = 0.0; // current X position as calculated by wheel rotations
float odomPosY = 0.0; // current Y position as calculated by wheel rotations
float targetPosX = 0.0; //X position of desired target
float targetPosY = 0.0; //Y position of desired target
float targetStartHeading = 0;
uint overshootAttempts = 0;
uint currentMode = 0;
uint obstacleMode = 1;
int turnDirection = 1; //intended turn direction
int differentialDirection; //actual direction to new heading
float searchCorrelation = 0;
float investigationCorrelation = 0;
float randomChargeLeave = 0;
float randomTravelGiveUp = 0;
float randomSearchGiveUp = 0;
float randomChargeReturn = 0;
float realHeading = 0;
float realPositionX = 0;
float realPositionY = 0;
float polarRadius = 0;
float polarTheta = 0;
float imageHeading = 0;
int imagePositionX = 0;
int imagePositionY = 0;
double pheromDecay;
double travelGiveUp;
double searchGiveUp;
double siteFidelity;
double uninformedCorr;
double informedCorr;
double pheromLaying;
double pheromFollowing;
double chargeLeave;
double chargeReturn;
float missionTime = 0; //counts the number of seconds the current mission has lasted
float searchTime = 0; //counts the number of seconds the current search segment has lasted
float walkingTime = 0; //tracks how long the robot has been walking this leg
float frenzyTimer = 0.0; //how long the robot has been failing to do what it wants
float frenzyLevel = 0.0; //how frantic the robot is while trying to clear obstacles
float obstacleActionTimer = 0; //how long robot has been attempting an obstacle avoidance action
float attemptedTurnTimer = 0; //how long the robot has been attempting the current turn
uint numTargetsDetected = 0; //how many targets robot is seeing right now
uint targetDetected = 0xFFFFFFFF; //id of the target detected and being harvested
float pheromoneHeading = 0; //heading to local highest intensity of pheromones

set<int> sweptTargetList;
uint sweptTargetCount = 0; //how many unique targets robot saw during sweep

//Status Flags
bool isTraveling = false;
bool isHeadingToTarget = false;
bool isGoingHome = false;
bool isInvestigatingArea = false;
bool isFollowingPheromones = false;
bool isLayingPheromones = false;
bool needsCharging = false;
bool isCharging = false;
bool isHome = true;
bool finishedTurning = true;
bool finishedWalking = true;
bool finishedAvoidingObstacle = true;
bool isProcessingTargets = false;
bool completedHalfSpin = false;
bool finishedObstacleAction = true;
bool wantsSiteFidelity = false; //robot wants to go back to a site to improve the fidelity of the information about that site
bool arrivedViaPheremonesOrViaFidelity = false;

//Default parameter values, can be undated from parameter server
float turnAccuracy = 20; //degree tolerance needed to "match" new heading
float digitalFenceSize = 10; //how many meters the robot is allowed to travel in x or y direction, square fence
float stepSizeSeconds = 1.0; //how long to walk each random walk segment, in seconds
float walkingSpeed = 0.3, defaultWalkingSpeed = 0.3; //how fast to walk each random walk segment
float turningSpeed = 0.2, defaultTurningSpeed = 0.2; //how fast to turn each random walk segment
float minWalkingSpeed = 0.0; // Kurt deleted this minimum in favor of a default
float minTurningSpeed = 0.0; // Kurt deleted this minimum in favor of a default
float frenzyBuildRate = 0.07; //how quickly the robot gets distressed if it can't clear an obstacle
float obstacleActionTimeLimit = 5; //try each obstacle avoidance action for this many seconds
float linearStuckTolerance = 0.800; //Robot is "stuck" if it hasn't moved this many meters within stuckLoopTimeStep
float angularStuckTolerance = 30; //Robot is "stuck" if it hasn't turned this many degrees within stuckLoopTimeStep
float stuckLoopTimeStep = 10.0; //number of seconds delay to check if the robot is "stuck"
float foundTargetTolerance = 1.0; //how many meters away robot can be and still have "arrived" at target
float turnTimeLimit = 120; //how long a robot will attempt a turn before giving up, in seconds

float mobilityLoopTimeStep = 0.125; //time between the mobility loop calls
//float probabilityLoopTimeStep = 0.5; //time between the probability loop calls
float probabilityLoopTimeStep = stepSizeSeconds; //time between the probability loop calls
float parameterServerLoopTimeStep = 1.0; //how often to check for new parameter server values

// state machine states
#define STATE_MACHINE_HOME	0
#define STATE_MACHINE_HEADING	1
#define STATE_MACHINE_TRAVEL	2
#define STATE_MACHINE_SEARCH	3
#define STATE_MACHINE_HARVEST	4
#define STATE_MACHINE_RETURN	5
int stateMachineState = STATE_MACHINE_HOME;
//int travelCounter = 0;
int searchCounter = 0;
//float missionTimeStartedTurning = 0;
//float missionTimeStartedWalking = 0;
//bool walkedForTimerAmount = false;
//bool turnedForTimerAmount = false;
int totalTripsCount = 0;
int totalTargetsHarvestedCount = 0;
int totalSiteFidelityCount = 0;
int totalPheremonesDepositedCount = 0;
int totalPheremonesFollowedCount = 0;
int totalSearchGiveUpsCount = 0;
int totalBatteryReturnsCount = 0;
int totalDigitalFenceCount = 0;
float lifetimeSearchGiveUpTime = 0;
float lifetimeTravelGiveUpTime = 0;
bool isOutsideDigitalFence = false;
bool warnFrenzyLatch = false;

geometry_msgs::Twist mobility;
char host[128];
string publishedName;
char prev_state_machine[128];

//ROS Components
ros::Subscriber joySubscriber;
ros::Subscriber modeSubscriber;
ros::Subscriber parameterSubscriber;
ros::Subscriber linearVelSubscriber;
ros::Subscriber angularVelSubscriber;
ros::Subscriber targetSubscriber;
ros::Subscriber poseSubscriber;
ros::Subscriber realLocSubscriber;
ros::Subscriber imageLocSubscriber;
ros::Subscriber obstacleSubscriber;
ros::Subscriber pheromoneHeadingSubscriber;
ros::Publisher mobilityPublish;
ros::Publisher pheromonePublish;
ros::Publisher harvestedPublish;
ros::Publisher stateMachinePublish;
ros::Timer stateMachineTimer;
ros::Timer probabilityTimer;
ros::Timer stuckTimer;
ros::Timer parameterTimer;
ros::ServiceClient harvestClient;
ros::ServiceClient mapHarvestClient;
ros::ServiceClient obstacleCountClient;
ros::ServiceClient pheromoneTrackClient;
rover_onboard_target_detection::harvest harvestMessage;
rover_onboard_target_detection::harvest obstacleMessage;
rover_driver_world_state::trackPheromone trackPheromoneMessage;

//Battery Variables, functions, and misc. I will put these in the correct location when we are sure my code works
void batteryHandler(const std_msgs::Float32& message); //Reads the battery voltage being published from /battery and makes decisions based on the battery level

float batteryLevel; //percentage of battery level left
float batteryDeadPercent = 0.65; //Percentage of battery level that is considered "dead"
int totalDeadRovers = 0; // total number of dead rovers, will eventually be published to a topic on the MCP
bool isDead = false; //Did this rover's battery percentage reach batteryDeadPercent?
std_msgs::Bool chargingState; //Is the rover charging? published on the /chargingState topic
#define STATE_MACHINE_DEAD 6 //Dead state. Halt all movement and add a dead rover to the counter

ros::Subscriber batterySubscriber; //Subscribe to a battery topic
ros::Publisher chargingStatePublisher; //Publishes charging state

int main(int argc, char **argv) {

    gethostname(host, sizeof (host));
    string hostname(host);

    srand(time(NULL));

    if (argc >= 2) {
        publishedName = argv[1];
        cout << "Welcome to the world of tomorrow " << publishedName << "!  Mobility module started." << endl;
    } else {
        publishedName = hostname;
        cout << "No Name Selected. Default is: " << publishedName << endl;
    }

    ros::init(argc, argv, (publishedName + "_MOBILITY"));
    ros::NodeHandle mNH;

    ros::TimerEvent actNow;
    loadParameters(actNow);

    joySubscriber = mNH.subscribe((publishedName + "/joystick"), 10, joyCmdHandler);
    modeSubscriber = mNH.subscribe((publishedName + "/mode"), 1, modeHandler);
    parameterSubscriber = mNH.subscribe((publishedName + "/parameters"), 1, parameterHandler);
    linearVelSubscriber = mNH.subscribe((publishedName + "/linearVel"), 1, linearVelHandler);
    angularVelSubscriber = mNH.subscribe((publishedName + "/angularVel"), 1, angularVelHandler);
    targetSubscriber = mNH.subscribe((publishedName + "/targets"), 10, targetHandler);
    obstacleSubscriber = mNH.subscribe((publishedName + "/obstacle"), 10, obstacleHandler);
    batterySubscriber = mNH.subscribe((publishedName + "/battery"), 1, batteryHandler); //Reads the battery level being published by the battery module

    //poseSubscriber = mNH.subscribe((publishedName + "/pose2d"), 1, poseHandler);
    realLocSubscriber = mNH.subscribe((publishedName + "/location_real"), 10, realLocHandler);
    imageLocSubscriber = mNH.subscribe((publishedName + "/location_image"), 10, imageLocHandler); /// this image location coordinate system is used by pheremone following

    mobilityPublish = mNH.advertise<geometry_msgs::Twist>((publishedName + "/mobility"), 10);
    pheromonePublish = mNH.advertise<std_msgs::Bool>((publishedName + "/pheromone"), 1, true);
    chargingStatePublisher = mNH.advertise<std_msgs::Bool>((publishedName + "/chargingState"), 1, true); //publishes whether or not the rover is currently at home charging its batteries.
    harvestedPublish = mNH.advertise<std_msgs::UInt64>((publishedName + "/harvested"), 1, true); //publishes the targets harvested per rover
    stateMachinePublish = mNH.advertise<std_msgs::String>((publishedName + "/state_machine"), 1, true); //publishes the current state of the state machine, in human readable form

    stateMachineTimer = mNH.createTimer(ros::Duration(mobilityLoopTimeStep), mobilityStateMachine);
    probabilityTimer = mNH.createTimer(ros::Duration(probabilityLoopTimeStep), probabilityLoop);
    stuckTimer = mNH.createTimer(ros::Duration(stuckLoopTimeStep), stuckChecker);
    parameterTimer = mNH.createTimer(ros::Duration(parameterServerLoopTimeStep), loadParameters);

    harvestClient = mNH.serviceClient<rover_onboard_target_detection::harvest>("harvestSrv");
    mapHarvestClient = mNH.serviceClient<rover_onboard_target_detection::harvest>(publishedName + "/mapHarvestSrv"); // robot specific service, not world state service!!!
    obstacleCountClient = mNH.serviceClient<rover_onboard_target_detection::harvest>("obstacleCountSrv");
    pheromoneTrackClient = mNH.serviceClient<rover_driver_world_state::trackPheromone>("pheromoneTrackSrv");

    while (ros::ok()) {
        ros::spin();
    }
// TODO: I don't think the forever loop needs to be wrapped around the spin forever call.

    return EXIT_SUCCESS;
}

/***********************
 * MOBILITY LOGIC
 ************************/
void mobilityStateMachine(const ros::TimerEvent&) {
    std_msgs::String stateMachineMsg;
    if (currentMode == 2 || currentMode == 3) { //Robot is in automode

	// New state machine.  Only one state is valid at a time.
	if (stateMachineState == STATE_MACHINE_HOME) {

		depositPheromones(false); // stop depositing when home is reached

                if (needsCharging) {
	            // publish current state for the operator to see
                    stateMachineMsg.data = "HOME CHARGE";
                    //stateMachinePublish.publish(stateMachineMsg);


                    cout << publishedName << " is charging. Don't bother me." << endl;
                    isCharging = true;
                    chargingState.data = true;
                    chargingStatePublisher.publish(chargingState);
		}
		else {
	            // publish current state for the operator to see
                    stateMachineMsg.data = "HOME";
                    //stateMachinePublish.publish(stateMachineMsg);

			// this is the normal home state that resets and goes directly into HEADING state
			float avgTravelGiveUpTime = 0;
			float avgSearchGiveUpTime = 0;
			if (totalTripsCount) {
				avgTravelGiveUpTime = lifetimeTravelGiveUpTime /  totalTripsCount;
			}
			if (totalSearchGiveUpsCount) {
				avgSearchGiveUpTime = lifetimeSearchGiveUpTime /  totalSearchGiveUpsCount;
			}

			cout <<    "Lifetime Statistics (" << publishedName << "):" << endl 
				<< "  Total Trips: " << totalTripsCount 
				<< ", Targets Harvested: " << totalTargetsHarvestedCount 
				<< ", Search GiveUps: " << totalSearchGiveUpsCount
				<< ", Battery Charges: " << totalBatteryReturnsCount   << endl;
			cout    << "  Pheremones Deposited: " << totalPheremonesDepositedCount 
				<< ", Pheremones Followed: " << totalPheremonesFollowedCount 
				<< ", Site Fidelitys: " << totalSiteFidelityCount
				<< ", Digital Fences: " << totalDigitalFenceCount 
				<< endl;
			cout    << "  Avg Travel GiveUp Time: " << avgTravelGiveUpTime // in seconds
				<< ", Avg Search GiveUp Time: " << avgSearchGiveUpTime // in seconds
				<< ", Step Size: " << stepSizeSeconds << " seconds" 
				<< endl;

			// reset variables for next mission
		        missionTime = 0;
			totalTripsCount++;
		        //depositPheromones(false);
			//travelCounter = 0;
			searchCounter = 0;
			resetFrenzy();
			isHome = !isHome;

			if (wantsSiteFidelity) {
				cout << "I want site fidelity." << endl;
				isHeadingToTarget = true;
				wantsSiteFidelity = false;
				arrivedViaPheremonesOrViaFidelity = true;
			} else {
				//cout << "Checking for pheremones." << endl;
				lookForPheromones();
				if (isFollowingPheromones) {
					totalPheremonesFollowedCount++;
					cout << "I've found a pheremone trail to follow!" << endl;
					desiredHeading = pheromoneHeading;
					arrivedViaPheremonesOrViaFidelity = true;
				}
				else {
					cout << "No pheremones were found, so picking random heading." << endl;
				}
			}

			stateMachineState = STATE_MACHINE_HEADING;
			cout << "Transition to HEADING state. MET: " << missionTime << " seconds" << endl;

		} // end else normal non-charging home state

	}
	else if (stateMachineState == STATE_MACHINE_HEADING) {
	    // publish current state for the operator to see
            stateMachineMsg.data = "HEADING";
            //stateMachinePublish.publish(stateMachineMsg);

		searchCorrelation = 180;

                move();

		if (finishedTurning) {
                	//cout << "Heading turn is complete." << endl;
			resetFrenzy();
			travelHeading = realHeading;
			stateMachineState = STATE_MACHINE_TRAVEL;
			cout << "Transition to TRAVEL state.  Heading: " 
			     << travelHeading << ", MET: " << missionTime << " seconds" << endl;
		}

	}
	else if (stateMachineState == STATE_MACHINE_TRAVEL) {
		isTraveling = true;

		if (isHeadingToTarget) {
	    	    // publish current state for the operator to see
            	    stateMachineMsg.data = "TRAVEL TARGET";
            	    //stateMachinePublish.publish(stateMachineMsg);

			//check if robot has reached Target
			if (	(abs(targetPosX - realPositionX) < foundTargetTolerance) && 
				(abs(targetPosY - realPositionY) < foundTargetTolerance)		) {
				cout << "Robot has reached Target coordinates." << endl;
				isHeadingToTarget = false;

				isTraveling = false;
				isInvestigatingArea = true;
				resetFrenzy();
				searchTime = 0;
				stateMachineState = STATE_MACHINE_SEARCH;
				cout << "Transition to SEARCH state. MET: " << missionTime << " seconds" << endl;
				//cout << "My location:  x = " << realPositionX << ", y = " << realPositionY << endl;

			}
		}
		else if (isFollowingPheromones) {
		    // publish current state for the operator to see
	            stateMachineMsg.data = "TRAVEL FOLLOW";
	            //stateMachinePublish.publish(stateMachineMsg);

			// do nothing
			// definitely don't roll the dice for travel give up
		}
                // decide when to give up traveling (do not give up probabilistically when heading to a target, though)
		else if (randomTravelGiveUp < travelGiveUp) {
			cout << "Done travelling. MET: " << missionTime << " sec" << endl;
			cout << "Straight line distance from home: " << polarRadius << " meters" << endl;
			lifetimeTravelGiveUpTime = lifetimeTravelGiveUpTime + missionTime;
			isTraveling = false;
			isInvestigatingArea = true;
			resetFrenzy();
			searchTime = 0;
			stateMachineState = STATE_MACHINE_SEARCH;
			cout << "Transition to SEARCH state. MET: " << missionTime << " seconds" << endl;
			//cout << "My location:  x = " << realPositionX << ", y = " << realPositionY << endl;
		}
		else {
	    		// publish current state for the operator to see
            		stateMachineMsg.data = "TRAVEL";
            		//stateMachinePublish.publish(stateMachineMsg);
		}

               move();
	}
	else if (stateMachineState == STATE_MACHINE_SEARCH) {
	    // publish current state for the operator to see
            stateMachineMsg.data = "SEARCH";
            //stateMachinePublish.publish(stateMachineMsg);

		searchCounter++;
		searchTime = searchTime + mobilityLoopTimeStep;

                // respond to Targets if found
                if (numTargetsDetected > 0) {
			harvestMessage.request.tagToHarvest = targetDetected;
			if (harvestMessage.request.tagToHarvest < 0) {
				cout << "ERROR: This is the bug!" << endl;
				cout << "tag ID to harvest is " << harvestMessage.request.tagToHarvest 
				     << ", number tags found is " << numTargetsDetected << endl;
			}

			isInvestigatingArea = false;// TODO: This flag can now just be the state machine state, no need for this variable anymore
			cout << "I've found a target!" << endl;
			stateMachineState = STATE_MACHINE_HARVEST;
			//resetFrenzy();
			cout << "Transition to HARVEST state. MET: " << missionTime << " seconds" << endl;
			totalTargetsHarvestedCount++;

        		// added for publishing harvested count per rover
        		std_msgs::UInt64 msg;
        		msg.data = totalTargetsHarvestedCount;
        		harvestedPublish.publish(msg);

                }
                // decide if robot should give up search
                else if (randomSearchGiveUp < searchGiveUp) {
			totalSearchGiveUpsCount++;
			cout << "Giving up the search. Search time: " << searchTime << 
				" sec, MET: " << missionTime << " sec" << endl;
			isGoingHome = true;
			isInvestigatingArea = false;
			lifetimeSearchGiveUpTime = lifetimeSearchGiveUpTime + searchTime;
			searchTime = 0;
			cout << "Transition to RETURN state. MET: " << missionTime << " seconds" << endl;
			stateMachineState = STATE_MACHINE_RETURN;

                } 

                if (numTargetsDetected == 0) {
			move();
		}
	}
	else if (stateMachineState == STATE_MACHINE_HARVEST) {
	    // publish current state for the operator to see
            stateMachineMsg.data = "HARVEST";
            //stateMachinePublish.publish(stateMachineMsg);

		processTarget();

                //move();
	}
	else if (stateMachineState == STATE_MACHINE_RETURN) {
	    // publish current state for the operator to see
	    if (isLayingPheromones && !wantsSiteFidelity) {
                stateMachineMsg.data = "RETURN LAYING";
                //stateMachinePublish.publish(stateMachineMsg);
	    }
	    else if (isLayingPheromones && wantsSiteFidelity) {
                stateMachineMsg.data = "RETURN LAY/FID";
                //stateMachinePublish.publish(stateMachineMsg);
	    }
	    else if (!isLayingPheromones && wantsSiteFidelity) {
                stateMachineMsg.data = "RETURN FIDEL";
                //stateMachinePublish.publish(stateMachineMsg);
	    }
	    else {
                stateMachineMsg.data = "RETURN";
                //stateMachinePublish.publish(stateMachineMsg);
	    }



		isProcessingTargets = false;
		isGoingHome = true;

                move();

	    if (isLayingPheromones && finishedTurning) {
		depositPheromones(true); // TODO: Sort of a hack to publish this at 8 Hz, but fixes issue with laying pheremones while turning towards home
	    }

                // check if robot has reached home
                if (polarRadius < foundTargetTolerance) {
                cout << publishedName << " made it home!" << endl;
                    fullStop();
                    isGoingHome = false;
                    isHome = true;
			arrivedViaPheremonesOrViaFidelity = false;

			cout << "Transition to HOME state. MET: " << missionTime << " seconds" << endl;
			stateMachineState = STATE_MACHINE_HOME;
                }

        } else if (stateMachineState == STATE_MACHINE_DEAD) {
	    // publish current state for the operator to see
            stateMachineMsg.data = "DEAD";
            //stateMachinePublish.publish(stateMachineMsg);

            fullStop();
            //The swarmie died doing what he loved
	}
	else {
	    // publish current state for the operator to see
            stateMachineMsg.data = "UNKNOWN";
            //stateMachinePublish.publish(stateMachineMsg);

	    cout << "State Machine: INVALID" << endl;

            fullStop();
	}

    }  // mode is auto
    else {
        // mode is NOT auto

        // publish current state for the operator to see
        stateMachineMsg.data = "WAITING";
        //stateMachinePublish.publish(stateMachineMsg);

    }

    // publish state machine string for user, only if it has changed, though
    if (strcmp(stateMachineMsg.data.c_str(), prev_state_machine) != 0) {
        stateMachinePublish.publish(stateMachineMsg);
        sprintf(prev_state_machine, "%s", stateMachineMsg.data.c_str());
    }

    missionTime = missionTime + mobilityLoopTimeStep;

}

/***********************
 * Probability loop 
 * calculates probabilities slower than mobility loop evaluates the environment
 ************************/
void probabilityLoop(const ros::TimerEvent&) {
	randomChargeLeave = randProb();
	randomTravelGiveUp = randProb();
	randomSearchGiveUp = randProb();
	randomChargeReturn = randProb();
	
	investigationCorrelation = exponentialDecay(720.0, searchTime, informedCorr);
	//cout << "SC: " << searchCorrelation << " "; 
	//cout << "IC: " << investigationCorrelation << " "; 

	//if (stateMachineState == STATE_MACHINE_SEARCH) {
            //cout << "Std Deviation: " << searchCorrelation;
            //cout << ", Heading Differential: " << headingDifferential;
            //cout << ", Frenzy Level: " << frenzyLevel << endl;
 
            //cout << "bools: " << finishedTurning;
            //cout << ", " << finishedWalking;
            //cout << ", " << isGoingHome;
            //cout << ", " << isTraveling;
            //cout << ", " << isHeadingToTarget;
            //cout << ", " << isInvestigatingArea << endl;

	//}

	//cout << "Calculated Probabilities:" << endl; 
	//cout << " Charge Leave:" << randomChargeLeave;
	//cout << ", Charge Return:" << randomChargeReturn;
 	//cout << ", Travel Give Up:" << randomTravelGiveUp; 
	//cout << ", Search Give Up:" << randomSearchGiveUp; 
	//cout << ", Investig Correl:" << investigationCorrelation << endl; 
}

/***********************
 * convenience function that calculates which direction and speed 
 * the robot should be moving based on flags and variables
 ************************/
void move() {
    if ((obstacleMode > 1 && obstacleMode < 9) || !finishedAvoidingObstacle) {
	// obstacle was detected
        avoidObstacle();
    } else {

	// need to check for digital fence, no matter which state we are in
	//if (polarRadius > digitalFenceSize) {
	if ( (abs(realPositionX) > digitalFenceSize) || (abs(realPositionY) > digitalFenceSize) ) {
            //overshootAttempts = 0;
            //lastHeadingDifferential = 360;

		if (!isOutsideDigitalFence) { 
			// robot just left the arena!
			isOutsideDigitalFence = true;
			totalDigitalFenceCount++; 
                	cout << publishedName << " has encountered the digital fence" << endl;

	    		travelHeading = travelHeading + 30; // shift travel heading for later after fence is cleared
            		travelHeading = normalizeHeading(travelHeading); 
                	cout << "New travel heading: " << travelHeading << endl;

            		desiredHeading = polarTheta + 180; // turn the robot towards home temporarily to get back inside the fence
            		desiredHeading = normalizeHeading(desiredHeading); 
            		cout << "Setting desired heading to " << desiredHeading << endl;

            		headingDifferential = angleBetweenHeadings(realHeading, desiredHeading);
            		turnDirection = headingDifferential / abs(headingDifferential);
            		setRotationSpeed(turnDirection * turningSpeed);
		}

	} 

	// check for back inside the digital fence
	if ( (abs(realPositionX) < (digitalFenceSize - 1.0) ) && (abs(realPositionY) < (digitalFenceSize - 1.0) ) ) {
		// return has a 1 meter buffer due to GPS sensor noise
		if (isOutsideDigitalFence) { 
                        cout << publishedName << " is back inside the digital fence" << endl;
		}

		isOutsideDigitalFence = false; 
	}

	if (isHeadingToTarget && !isOutsideDigitalFence) {
		//cout << "Seeking Target " << endl;
		//desiredHeading = rad2deg(atan2(targetPosX - realPositionX, targetPosY - realPositionY));
		//searchCorrelation = uninformedCorr;


		    // atan2 uses y, x instead of not x, y
		    desiredHeading = rad2deg(atan2(targetPosY - realPositionY, targetPosX - realPositionX)); 
		    // result from atan2() is between -pi to +pi, 
		    // so it needs to be translated to our real coordinate system with zero to the right and 90 down
		    if (desiredHeading < 0) {
			// if negative, just change the sign to positive
			desiredHeading = desiredHeading * -1;
		    }
		    else {
			// if positive, subtract 360 then change the sign
			desiredHeading = desiredHeading - 360;
			desiredHeading = desiredHeading * -1;
		    }
// TODO: That entire atan2 and translation section is used twice in this file.  Turn it into a helper subroutine

		    cout << "Calculated heading to target " << targetPosX << ", " << targetPosY << " is " << desiredHeading << endl;

	}

        if (finishedTurning && finishedWalking && !isOutsideDigitalFence) { 
            //this is a new turn, calculate a new direction

            //cout << "New movement: ";
            overshootAttempts = 0;
            finishedTurning = false;
            //lastHeadingDifferential = 360;

            //Decide which direction to go
            if (isGoingHome) {
                //cout << publishedName << " is headed home" << endl;
                desiredHeading = polarTheta + 180;
                //searchCorrelation = uninformedCorr;
            } else if (isFollowingPheromones) {
                //cout << "Following Pheromone Trail" << endl;
                lookForPheromones();
                desiredHeading = pheromoneHeading;
                //searchCorrelation = uninformedCorr;
            } else if (isInvestigatingArea) {
                //cout << "Investigating Area" << endl;
                desiredHeading = realHeading;
		if (arrivedViaPheremonesOrViaFidelity){
			// searching here because of site fidelity or followed pheremones
			// need to force an informed search (wildly turning) with decay
                	searchCorrelation = uninformedCorr + investigationCorrelation;
		} else {
			// searching here just because
			// need to force an uninformed search (nearly straight)
                	searchCorrelation = uninformedCorr;
		}

            } else if (isTraveling) {
                //cout << "Still Traveling" << endl;
                desiredHeading = travelHeading;  
                //searchCorrelation = 180;
            } else {
		//cout << "Now what?" << endl;
	    }

            //as frenzy increases, robot cares less about getting somewhere specific. Increase randomness of desired heading.
            searchCorrelation = searchCorrelation + (180 * frenzyLevel);
            searchCorrelation = constrain(searchCorrelation, 0, 180);
            //randomize desired heading when searching or finding a new heading at home
	    if(	(stateMachineState == STATE_MACHINE_SEARCH) || 
		(stateMachineState == STATE_MACHINE_HEADING) 	)
	    {
		desiredHeading = normalRandom(desiredHeading, searchCorrelation); 
	    }
	    if (isFollowingPheromones) {
                desiredHeading = pheromoneHeading;
            }
            desiredHeading = normalizeHeading(desiredHeading); 
            headingDifferential = angleBetweenHeadings(realHeading, desiredHeading);
            turnDirection = headingDifferential / abs(headingDifferential);

	    /* cout << "Turning.  current heading: " << realHeading << 
		    ", desired heading: " << desiredHeading <<
		    ", polar theta: " << polarTheta << endl; */

	    /* cout << "Position: " << realPositionX <<
		    ", " << realPositionY << endl; */

            //cout << "Search Correlation: " << searchCorrelation;
            //cout << ", Frenzy Level: " << frenzyLevel << endl;

            //cout << "Frenzy Level: " << frenzyLevel << ", Search Correlation: " << searchCorrelation << ", Search Time: " << searchTime << endl;
            // cout << "Current Heading: " << realHeading << ", Desired Heading: " << desiredHeading << endl;
            setRotationSpeed(turnDirection * turningSpeed);
        } // end new turn logic

        if (finishedWalking) {
            //cout << "Done walking: Current Heading: " << realHeading << ", Desired Heading: " << desiredHeading << endl;
            //turn is in progress
            if ( onTargetDegrees(realHeading, desiredHeading) || (overshootAttempts >= 3) ) {
                //stop turning when robot is pointing the right way and start walking
                //cout << "Finished Turn" << endl;
                //finishedTurning = true;
                //finishedWalking = false;
    		//missionTimeStartedWalking = missionTime;
                setLinearSpeed(walkingSpeed);
		//cout << "Done turning." << endl;
            } else {
                //keep turning but correct for overshoot, only attemp a correction 3 times
                headingDifferential = angleBetweenHeadings(realHeading, desiredHeading);
                differentialDirection = headingDifferential / abs(headingDifferential);
                if (differentialDirection != turnDirection) {
                    //robot overshot and missed the desired heading, turn back
                    overshootAttempts++;
                    turnDirection = -1 * turnDirection;
                    //cout << "Overshoot Attempt " << overshootAttempts << endl;
                    setRotationSpeed(turnDirection * turningSpeed);
                }
            }
        } else {
            //walk is in progress
            //cout << "Walking: Current Heading: " << realHeading << ", Desired Heading: " << desiredHeading << endl;
            walkingTime = walkingTime + mobilityLoopTimeStep;
            if (walkingTime >= stepSizeSeconds) {
                //stop walking after stepSizeSeconds seconds
                finishedWalking = true;  // TODO, this is set to true in setRotationSpeed.  Is that redundant?
		//cout << "Done walking." << endl;
                walkingTime = 0;
            }
        }
    }

}

int avoidObstacle() {
    //cout << endl << "Obstacle Detected" << endl;
    if (obstacleMode == 1 || obstacleMode == 9) {
        //obstacle cleared
        cout << "Obstacle Cleared" << endl;
        fullStop();
    	//missionTimeStartedWalking = missionTime;
        setLinearSpeed(walkingSpeed);
        finishedAvoidingObstacle = true;
        finishedObstacleAction = true;
        obstacleActionTimer = 0;
	sootheFrenzy();
// TODO: This section is never getting called because the caller never calls this when in mode 1 or 9, thus robot sticks in slow speed.
    } else {
        //still seeing obstacle
        finishedAvoidingObstacle = false;

        if (finishedObstacleAction) {
            //start a new action to clear obstacle
            finishedObstacleAction = false;
            obstacleActionTimer = 0;
// TODO: Why is robot calling an obstacle world service?  what is it needed for?
            obstacleMessage.request.tagToHarvest = targetDetected; //place holder message, message content is irrelevant
            obstacleCountClient.call(obstacleMessage); //inform world state new obstacle action happened

            //bias toward attempting turns but build up to a 25/75 shot of walking vs turning at full frenzy level
            if (randProb() < (frenzyLevel / 4)) {
                //walk, tending towards backwards away from obstacle
    		//missionTimeStartedWalking = missionTime;
                setLinearSpeed(-walkingSpeed);
            } else {
                //turn
                if (obstacleMode == 2 || obstacleMode == 3 || obstacleMode == 5 || obstacleMode == 7 || obstacleMode == 8) {
                    //Obstacles are on the left, turn right
                    //cout << "Turning right to avoid Obstacle" << endl;
                    setRotationSpeed(-1 * turningSpeed);
                } else {
                    //Obstacles are on the right, turn left
                    //cout << "Turning left to avoid Obstacle" << endl;
                    setRotationSpeed(turningSpeed);
                }
            }
        }

        //attempt an obstacle avoidance action for a duration that is biased towards shorter times as frenzy level increases
        if (obstacleActionTimer >= constrain(normalRandom(obstacleActionTimeLimit * (1 - frenzyLevel), obstacleActionTimeLimit * frenzyLevel), 1, obstacleActionTimeLimit)) {
            finishedObstacleAction = true;
        }

        obstacleActionTimer = obstacleActionTimer + mobilityLoopTimeStep;
    }
}

void processTarget() {
// TODO: This is a bit messy and may have some bugs.  Needs to be lean and mean and clean.

    // check for new target
    if (!isProcessingTargets) {
	// grab ID.  this is a problem because process target is called at 8Hz, 
	// but targetDetected is updated as fast the sensor can update
        //harvestMessage.request.tagToHarvest = targetDetected;
	//if (harvestMessage.request.tagToHarvest < 0) {
	//	cout << "ERROR: This is the bug!" << endl;
	//	cout << "tag ID to harvest is " << harvestMessage.request.tagToHarvest 
	//	     << ", number tags found is " << numTargetsDetected << endl;
	//}

        fullStop();
	isProcessingTargets = true;

        //cout << "New Target Detected, start Sweep" << endl;
	//totalTargetsHarvestedCount++;

        //harvestMessage.request.tagToHarvest = targetDetected;  // this was in Caylyne's fix, not sure if it's old or not


// TODO: This should be one single call to world state and world state can update both the topic and the map, right?
        harvestClient.call(harvestMessage); //inform world state of new tag
        mapHarvestClient.call(harvestMessage); //inform mapping of new tag

        sweptTargetList.clear();
        finishedTurning = false;
        completedHalfSpin = false;
        targetStartHeading = realHeading;
        desiredHeading = normalizeHeading(targetStartHeading + 180);
        setRotationSpeed(randSign() * turningSpeed);
    }

    if (onTargetDegrees(realHeading, desiredHeading)) {
        if (!completedHalfSpin) {
            //robot has spun halfway, contine to 360 degrees
            completedHalfSpin = true;
            desiredHeading = targetStartHeading;
// TODO: This is buggy.  Sometimes it spins multiple 360 degree turns while looking for more targets
        } else {
            //robot has spun a full circle
            cout << publishedName << " found " << sweptTargetCount << " other target(s) in the immediate area" << endl;

            fullStop();

		//cout << "Rolling the dice for pheremone laying." << endl;
		float prob = randProb();
		if (poissonCDF(sweptTargetCount, pheromLaying) > prob) {
                	cout << publishedName << " won the Pheremone Laying lottery! (" << pheromLaying <<
				") Search time: " << searchTime << 
				" sec, MET: " << missionTime << " sec" << endl;
			//depositPheromones(true);
			isLayingPheromones = true;
			totalPheremonesDepositedCount++;
		} else {
                	cout << publishedName << " will not lay pheremones this time." << endl;
		}

 		//cout << "Rolling the dice for site fidelity." << endl;
		prob = randProb();
		if (poissonCDF(sweptTargetCount, siteFidelity) > prob) {
                	cout << publishedName << " won the Site Fidelity lottery! (" << siteFidelity <<
				") Search time: " << searchTime << 
				" sec, MET: " << missionTime << " sec" << endl;
			wantsSiteFidelity = true;
			targetPosX = realPositionX;
			targetPosY = realPositionY;
			totalSiteFidelityCount++;
                	cout << publishedName << " saved target position as " << targetPosX << ", " << targetPosY << endl;
		} else {
                	cout << publishedName << " will not return to this site (unless via pheremones)." << endl;
 		}

		cout << "Transition to RETURN state. MET: " << missionTime << " seconds" << endl;
		stateMachineState = STATE_MACHINE_RETURN;
		//isProcessingTargets = false;
		//isGoingHome = true;

        }
    }

    headingDifferential = angleBetweenHeadings(realHeading, desiredHeading);
}

// 
// This routine looks for either a rotation or a location change every timer amount
// but only in auto mode
//
// This routine will get robot unstuck from an obstacle or similar, 
// but will not notice if it is in a bug trap going in circles missing the exit hole
//
void stuckChecker(const ros::TimerEvent&) {
    //cout << "Checking for stuck robot in " << stuckLoopTimeStep << " second loop." << endl;

    //if ((currentMode == 2 || currentMode == 3) && finishedAvoidingObstacle) { //If robot is in auto mode
    if ( (currentMode == 2 || currentMode == 3) ) { //If robot is in auto mode
   	float totalMoveLineLength = sqrt( 
					(abs(realPositionX - lastPosX) * abs(realPositionX - lastPosX)) + 
					(abs(realPositionY - lastPosY) * abs(realPositionY - lastPosY)) 
					);
	//cout << "Linear move: " << totalMoveLineLength << endl;
        bool hasWalked = linearStuckTolerance < totalMoveLineLength;

	float totalTurnAngle = abs(realHeading - lastHeading);
	//cout << "Angular move: " << totalTurnAngle << endl;
        bool hasTurned = angularStuckTolerance < totalTurnAngle;

        lastPosX = realPositionX;
        lastPosY = realPositionY;
        lastHeading = realHeading;

	if(hasWalked || hasTurned) {
		//cout << "Robot is not stuck." << endl;
		sootheFrenzy();
	}
	else {
                //cout << "Robot appears to be stuck.  Increasing frenzy level." << endl;
                increaseFrenzy();
	}

    } // end auto mode
}

void increaseFrenzy() {
    frenzyTimer = frenzyTimer + stuckLoopTimeStep;
    
    //frenzyLevel = exponentialGrowth(0.01, frenzyTimer, frenzyBuildRate);
    frenzyLevel = exponentialGrowth(0.07, frenzyTimer, frenzyBuildRate);
    frenzyLevel = constrain(frenzyLevel, 0, 1);

    if ( (frenzyLevel > 0.3) && !warnFrenzyLatch) {
        cout << "WARNING: Frenzy level is elevated!" << endl;
	warnFrenzyLatch = true;
    }

    // cout << "Frenzy level was increased to " << frenzyLevel << endl;
}

void sootheFrenzy() {
    frenzyTimer = frenzyTimer - (3 * stuckLoopTimeStep);

    //frenzyLevel = exponentialGrowth(0.01, frenzyTimer, frenzyBuildRate);
    frenzyLevel = exponentialGrowth(0.07, frenzyTimer, frenzyBuildRate);
    frenzyLevel = constrain(frenzyLevel, 0, 1);

    if ( (frenzyLevel < 0.1) && warnFrenzyLatch) {
        cout << "Frenzy level has settled down again." << endl;
	warnFrenzyLatch = false;
    }

    if (frenzyLevel < 0.01) {
	frenzyLevel = 0;
    }

    // cout << "Frenzy level was soothed to " << frenzyLevel << endl;
}

void resetFrenzy() {
    frenzyTimer = 0;
    frenzyLevel = 0;

    if ( (frenzyLevel < 0.1) && warnFrenzyLatch) {
        cout << "Frenzy level has settled down again." << endl;
	warnFrenzyLatch = false;
    }

    cout << "Frenzy level was reset to zero." << endl;
}

void setRotationSpeed(float speed) {
    finishedWalking = true;
    finishedTurning = false;
    //missionTimeStartedTurning = missionTime;
    mobility.linear.x = 0;
    mobility.angular.z = 0;
    mobilityPublish.publish(mobility);
    //mobility.linear.x = 0;
    float allowableSpeed = max((float) 1.0, turningSpeed);
    speed = normalRandom(speed, frenzyLevel * 2); 
// TODO: This random speed affects stuck calculation and can give false positives
    speed = speed / abs(speed) * constrain(abs(speed), minTurningSpeed, allowableSpeed);
    mobility.linear.x = 0;
    // multiply speed for simulation.  remove this factor in abridge for real robots.
    //mobility.angular.z = speed;
    mobility.angular.z = speed * 8;
    mobilityPublish.publish(mobility);
    //cout << "Set Rotation Speed " << speed << endl;
}

void setLinearSpeed(float speed) {
    finishedWalking = false;
    finishedTurning = true;
    //missionTimeStartedWalking = missionTime;
    mobility.linear.x = 0;
    mobility.angular.z = 0;
    mobilityPublish.publish(mobility);
    float allowableSpeed = 1;
    speed = normalRandom(speed, frenzyLevel * 2);
// TODO: This random speed affects stuck calculation and can give false positives
    speed = speed / abs(speed) * constrain(abs(speed), minWalkingSpeed, allowableSpeed);
    // multiply speed for simulation.  remove this factor in abridge for real robots.
    //mobility.linear.x = speed;
    mobility.linear.x = speed * 1.5;
    mobility.angular.z = 0;
    mobilityPublish.publish(mobility);
    //cout << "Set Linear Speed " << speed << endl;
    //cout << "Walking." << endl;
}

void fullStop() {
    finishedWalking = true;
    finishedTurning = true;
    isProcessingTargets = false;
    mobility.linear.x = 0;
    mobility.angular.z = 0;
    mobilityPublish.publish(mobility);
    //cout << "Full Stop" << endl;
}

bool onTargetDegrees(float myCurrentHeading, float requestedHeading) {
    float headingUpperLimit;
    float headingLowerLimit;

    if (isFollowingPheromones) {
        headingUpperLimit = requestedHeading + (turnAccuracy / 2);
        headingLowerLimit = requestedHeading - (turnAccuracy / 2);
    }
    else {
        headingUpperLimit = requestedHeading + turnAccuracy;
        headingLowerLimit = requestedHeading - turnAccuracy;
    }

    float head1 = myCurrentHeading;
    float head2 = myCurrentHeading + 360;
    float head3 = myCurrentHeading - 360;

// TODO: Take a look at this.  This might be buggy causing weird turns.
    bool onTarget = (head1 <= headingUpperLimit && head1 >= headingLowerLimit)
            || (head2 <= headingUpperLimit && head2 >= headingLowerLimit)
            || (head3 <= headingUpperLimit && head3 >= headingLowerLimit);

    onTarget = onTarget || attemptedTurnTimer > turnTimeLimit;

    attemptedTurnTimer = onTarget ? 0 : (attemptedTurnTimer + mobilityLoopTimeStep);

    return onTarget;
}

void depositPheromones(bool deposit) {
    isLayingPheromones = deposit; // TODO: Sort of a hack to set this every time when only needed when turning false
    std_msgs::Bool pheromoneMessage;
    pheromoneMessage.data = deposit;
    pheromonePublish.publish(pheromoneMessage);
}

void lookForPheromones() {
	//cout << "Pheremone query. X: " << imagePositionX << ", Y: " << imagePositionY << ", Heading: " << imageHeading << endl;

	trackPheromoneMessage.request.X = imagePositionX;
	trackPheromoneMessage.request.Y = imagePositionY;
	trackPheromoneMessage.request.currentHeading = imageHeading;

	// cone angle was added to call so we can force a 360 degree search at the nest and a smaller cone after that.  
	// TODO: May be a better way to do it, though.
	if (stateMachineState == STATE_MACHINE_TRAVEL) {
    		trackPheromoneMessage.request.coneAngle = 25;
// TODO: This is too simplistic.  Too wide a cone and we get to the end of a trail and turn around and follow it backwards.  Too narrow of a cone and we can't follow a trail around simple obstacles.
	}
	else {
    		trackPheromoneMessage.request.coneAngle = 179;
	}
	pheromoneTrackClient.call(trackPheromoneMessage);

	pheromoneHeading = trackPheromoneMessage.response.newHeading;
	pheromoneHeading = normalizeHeading(pheromoneHeading); 
	bool pheromFound = trackPheromoneMessage.response.pheromFound;

	//cout << "Pheremone response: " << pheromFound << ", Heading: " << pheromoneHeading << endl;

    //if following pheromones, handle if the trail is lost
    if (isFollowingPheromones) {
        //isInvestigatingArea = not(pheromFound);// TODO: make this more clear!
        //missionTime = 0;
	if (!pheromFound) {
		cout << "Pheremone trail has ended (" << realPositionX << ", " 
		     << realPositionY << ").  Need to search here. " << endl;

		isTraveling = false;
		isInvestigatingArea = true;
		resetFrenzy();
		searchTime = 0;
		stateMachineState = STATE_MACHINE_SEARCH;
		cout << "Transition to SEARCH state. MET: " << missionTime << " seconds" << endl;

	}
    }

    isFollowingPheromones = pheromFound;

}

/***********************
 * UTILITY FUNCTIONS
 ************************/

float randProb() {
    return (float) rand() / (float) RAND_MAX;
}

int randSign() {
    return rand() % 2 * 2 - 1;
}

float normalRandom(float mean, float stdDev) {
    static bool normal_is_valid = false;
    static float normal_x;
    static float normal_y;
    static float normal_rho;
    if (!normal_is_valid) {
        normal_x = randProb();
        normal_y = randProb();
        normal_rho = sqrt(-2. * log(1.0 - normal_y));
        normal_is_valid = true;
    } else
        normal_is_valid = false;

    if (normal_is_valid)
        return normal_rho * cos(2. * M_PI * normal_x) * stdDev + mean;
    else
        return normal_rho * sin(2. * M_PI * normal_x) * stdDev + mean;
}

float poissonCDF(float k, float lambda) {
    float sumAccumulator = 1;
    float factorialAccumulator = 1;

    for (int i = 1; i <= floor(k); i++) {
        factorialAccumulator *= i;
        sumAccumulator += pow(lambda, i) / factorialAccumulator;
    }

    return (exp(-lambda) * sumAccumulator);
}

float exponentialDecay(float quantity, float time, float lambda) {
    return (quantity * exp(-lambda * time));
}

float exponentialGrowth(float quantity, float time, float lambda) {
    return (quantity * exp(lambda * time));
}

float constrain(float value, float minVal, float maxVal) {
    return max(minVal, (float) min(maxVal, value));
}

float normalizeHeading(float oldHeading) { 
    float correctedHeading = oldHeading;
    if (oldHeading >= 360) {
        correctedHeading = oldHeading - 360;
    }
    else if (oldHeading < 0) {
        correctedHeading = 360 + oldHeading;
    }
    return correctedHeading;
}

float angleBetweenHeadings(float a1, float a2) {
    float opt1 = (a1 - a2) < 0 ? a1 - a2 + 360.0 : a1 - a2;
    float opt2 = (a2 - a1) < 0 ? a2 - a1 + 360.0 : a2 - a1;
    float sign = (opt1 - opt2) < 0 ? 1.0 : -1.0;
    return sign * min(opt1, opt2);

}

float rad2deg(float radian) {
    return (radian * (180 / M_PI));
}

float deg2rad(float degree) {
    return (degree * (M_PI / 180));
}

/***********************
 * ROS CALLBACK HANDLERS
 ************************/

void targetHandler(const rover_onboard_target_detection::ATag tagInfo) {

    if (isProcessingTargets) {
        if (numTargetsDetected != tagInfo.tagsFound) {
            //cout << "Inserting target(s) into swept list. was: " << sweptTargetCount << ", ";
            sweptTargetList.insert(tagInfo.tagID.begin(), tagInfo.tagID.end());
            sweptTargetCount = sweptTargetList.size();
            //cout << "is now: " << sweptTargetCount << endl;
        }
    }

    numTargetsDetected = tagInfo.tagsFound;
    if (numTargetsDetected > 0) {
        targetDetected = *tagInfo.tagID.begin(); // grab first one
    } else {
        targetDetected = 0xFFFFFFFF; 
    }
}

//void poseHandler(const geometry_msgs::Pose2D::ConstPtr& message) {
//    realHeading = (float) message->theta;
//}

void realLocHandler(const geometry_msgs::Pose2D::ConstPtr& message) {
    realHeading = (float) message->theta;
    realPositionX = (float) message->x;
    realPositionY = (float) message->y;
    polarRadius = sqrt( (realPositionX * realPositionX) + (realPositionY * realPositionY) );
    // atan2 uses y, x instead of not x, y
    polarTheta = rad2deg(atan2(realPositionY, realPositionX)); 
    // result from atan2() is between -pi to +pi, 
    // so it needs to be translated to our real coordinate system with zero to the right and 90 down
    if (polarTheta < 0) {
	// if negative, just change the sign to positive
	polarTheta = polarTheta * -1;
    }
    else {
	// if positive, subtract 360 then change the sign
	polarTheta = polarTheta - 360;
	polarTheta = polarTheta * -1;
    }

}

void imageLocHandler(const geometry_msgs::Pose2D::ConstPtr& message) {
    imageHeading = (float) message->theta;
    imagePositionX = (int) message->x;
    imagePositionY = (int) message->y;
}

void linearVelHandler(const std_msgs::Float32 message) {
    walkingSpeed = message.data;
    defaultWalkingSpeed = message.data;

    //if (currentMode == 2) {
        //Set the minimum walking speed to that of an iAnt in the Gazebo Simulation
        //minWalkingSpeed = 0.35;
    //} else if (currentMode == 3) {
        //Set the minimum walking speed to that of an iAnt in the real world
        //minWalkingSpeed = 0.18;
    //}
}

void angularVelHandler(const std_msgs::Float32 message) {
    turningSpeed = message.data;
    defaultTurningSpeed = message.data;

    //if (currentMode == 2 || currentMode == 0) {
        //turningSpeed = defaultTurningSpeed * 10;
    //} else {
        //turningSpeed = defaultTurningSpeed;
    //}
}

void modeHandler(const std_msgs::UInt8::ConstPtr& message) {
	currentMode = message->data;
	cout << "MCP mode has changed to " << currentMode << endl;
	resetFrenzy();
	fullStop();
	// artificially force stuck checker to be unstuck on first cycle, since the time of first cycle may be infinitely short
	// this keeps robots from immediately being frenzied up when started
        lastPosX = -100;
        lastPosY = -100;
        lastHeading = 900;
}

void parameterHandler(const std_msgs::Float32MultiArray message) {
    pheromDecay = message.data.at(0);
    float temp = message.data.at(1); // radians
    informedCorr = message.data.at(2);
    travelGiveUp = message.data.at(3);
    searchGiveUp = message.data.at(4);
    pheromLaying = message.data.at(5);
    chargeLeave = message.data.at(6);
    chargeReturn = message.data.at(7);
    siteFidelity = message.data.at(8);
    pheromFollowing = message.data.at(9);

    uninformedCorr = rad2deg(temp); // all this code assumes degrees
}

void obstacleHandler(const std_msgs::UInt8::ConstPtr& message) {
    obstacleMode = message->data;
    walkingSpeed = (obstacleMode > 1) ? (defaultWalkingSpeed / 2) : defaultWalkingSpeed; //set linear speed based on caution distance
    turningSpeed = (obstacleMode > 1) ? (defaultTurningSpeed / 2) : defaultTurningSpeed; //set turning speed based on caution distance
    //if (currentMode == 2 || currentMode == 0) {
        //turningSpeed = defaultTurningSpeed * 10;
    //} else {
        turningSpeed = defaultTurningSpeed;
    //}
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

    ros::param::param((publishedName + "/mobility/turnAccuracy"), param, (double) turnAccuracy);
    turnAccuracy = (float) param;
    ros::param::param((publishedName + "/mobility/digitalFenceSize"), param, (double) digitalFenceSize);
    digitalFenceSize = (float) param;
    ros::param::param((publishedName + "/mobility/mobilityLoopTimeStep"), param, (double) mobilityLoopTimeStep);
    mobilityLoopTimeStep = (float) param;
    ros::param::param((publishedName + "/mobility/stuckLoopTimeStep"), param, (double) stuckLoopTimeStep);
    stuckLoopTimeStep = (float) param;
    ros::param::param((publishedName + "/mobility/stepSizeSeconds"), param, (double) stepSizeSeconds);
    stepSizeSeconds = (float) param;
    ros::param::param((publishedName + "/mobility/minWalkingSpeed"), param, (double) minWalkingSpeed);
    minWalkingSpeed = (float) param;
    ros::param::param((publishedName + "/mobility/minTurningSpeed"), param, (double) minTurningSpeed);
    minTurningSpeed = (float) param;
    ros::param::param((publishedName + "/mobility/frenzyBuildRate"), param, (double) frenzyBuildRate);
    frenzyBuildRate = (float) param;
    ros::param::param((publishedName + "/mobility/obstacleActionTimeLimit"), param, (double) obstacleActionTimeLimit);
    obstacleActionTimeLimit = (float) param;
    ros::param::param((publishedName + "/mobility/linearStuckTolerance"), param, (double) linearStuckTolerance);
    linearStuckTolerance = (float) param;
    ros::param::param((publishedName + "/mobility/foundTargetTolerance"), param, (double) foundTargetTolerance);
    foundTargetTolerance = (float) param;
    ros::param::param((publishedName + "/mobility/turnTimeLimit"), param, (double) turnTimeLimit);
    turnTimeLimit = (float) param;
}

void joyCmdHandler(const sensor_msgs::Joy::ConstPtr& message) {
    if (currentMode == 0) {
        //Manual mode with Simulated Bot, added gain to compensate for slow Sim
        //mobility.angular.z = message->axes[0] * 7;
        //mobility.linear.x = message->axes[1] * 2;
        //mobilityPublish.publish(mobility);
        // multiply speed for simulation.  remove this factor in abridge for real robots.
        //mobility.angular.z = message->axes[0];
        //mobility.linear.x = message->axes[1];
        mobility.angular.z = message->axes[0] * 8;
        mobility.linear.x = message->axes[1] * 1.5;
        mobilityPublish.publish(mobility);
    } else if (currentMode == 1) {
        //Manual mode with Physical Bot
        // multiply speed for simulation.  remove this factor in abridge for real robots.
        //mobility.angular.z = message->axes[0];
        //mobility.linear.x = message->axes[1];
        mobility.angular.z = message->axes[0] * 8;
        mobility.linear.x = message->axes[1] * 1.5;
        mobilityPublish.publish(mobility);
    } else {
        //Auto modes or other
        //Do nothing, i.e. do not publish mobility commands based on joystick messages if in Auto
        //This empty case is left in place for adding other potential future manual modes
    }
}

void batteryHandler(const std_msgs::Float32& message) {
   /* //Excessive amount of debug statements.
    //cout << "Battery Stats: " << endl;
    cout << endl;
    cout << "State Machine: " << stateMachineState << endl;
    cout << "Is going home: " << isGoingHome << endl;
    cout << "is home " << isHome << endl;
    cout << "Is traveling: " << isTraveling << endl;
    cout << "needs charging: " << needsCharging << endl;
    cout << "isFollowingPheromones: " << isFollowingPheromones << endl;
    //cout << "battery Level" << batteryLevel << endl;
    cout << "Charging State" << chargingState.data << endl;
    //cout << "battery return " << chargeReturn << endl;
    //cout << "Battery Dead percent: " << batteryDeadPercent << endl;
    //cout << "is charging: " << isCharging << endl;
    cout << endl;*/


    batteryLevel = message.data; //set the batteryLevel variable to the published battery voltage

    if (batteryLevel >= chargeLeave) {//if the battery is full, go out and explore
        cout << "Done Charging" << endl;
        if (isCharging) {
            totalBatteryReturnsCount++; //should be only called once now. Eliminates double counting battery returns.
        }
        isCharging = false;
        needsCharging = false;
        chargingState.data = false; //set the ROS message type of the chargingState to false so that the battery module knows that we are done charging
        chargingStatePublisher.publish(chargingState);
        stateMachineState = STATE_MACHINE_HOME;
    } else if (batteryLevel > chargeReturn && batteryLevel < chargeLeave && !isCharging) {//exploring mode
        //cout << "Exploring" << endl;
        //When exploring, don't do anything, just listen to the battery topic.
        //Could possibly add something here such as cout the battery voltage...
    } else if (batteryLevel <= chargeReturn && batteryLevel > batteryDeadPercent && !isCharging) {//needs to go home and charge, the battery voltage is less than the return value but the rover is still alive.
        cout << "Need to charge" << endl;
        isTraveling = false;
        isFollowingPheromones = false;
        isHeadingToTarget = false;
        isInvestigatingArea = false;
        //isFollowingPheromones = false;
        isProcessingTargets = false;
        //cout << "State machine State" << stateMachineState << endl;
        if (!isHome) {//if the rover is not home yet, do this:
            cout << "Going home to charge" << endl;
            isGoingHome = true; //we want the rover to move in the direction towards home
            stateMachineState = STATE_MACHINE_RETURN; //drop into this state. We want the rover to return home to charge. Stop everything else because he will die soon if he does not get a charge
        } else {//Rover is already at home.
            isHome = true;
            stateMachineState = STATE_MACHINE_HOME;
        }
        needsCharging = true; //let the state machine know that the rover needs to charge.
    } else if (batteryLevel <= batteryDeadPercent) {//Rover battery level is less than the dead battery percentage. Halt all movement. Rover is out of the picture
        //cout << "Dead" << endl;
        if (!isDead) {//only called once.
            char *rover = new char[publishedName.size() + 1]; //convert the publishedName string into a char array so we can use ROS info
            rover[publishedName.size()] = 0;
            memcpy(rover, publishedName.c_str(), publishedName.size());


            ROS_INFO("%s has perished", rover); //Publish that the rover has died as a ROS message so we have a timestamp
            totalDeadRovers++; //TODO: Have this publish to the dead rovers topic on the MCP. right now it is just a tally...
        }
        isDead = true; //Rover is dead
        chargingState.data = false;
        chargingStatePublisher.publish(chargingState);
        depositPheromones(false); //make sure the rover stops depositing pheromones while it is dead.
        stateMachineState = STATE_MACHINE_DEAD;

    }

}


