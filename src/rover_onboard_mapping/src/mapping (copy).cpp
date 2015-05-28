/*
 * Author: Karl A. Stolleis
 * Maintainer: Karl A. Stolleis
 * Email: karl.a.stolleis@nasa.gov; kurt.leucht@nasa.gov
 * NASA Center: Kennedy Space Center
 * Mail Stop: NE-C1
 * 
 * Project Name: Swarmie Robotics NASA Center Innovation Fund
 * Principal Investigator: Cheryle Mako
 * Email: cheryle.l.mako@nasa.gov
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
 * -Boundary Checking COMPLETE 9/17/2014 MWN
 * -pheromone service COMPLETE 9/17/2014 MWN
 * -add tag list service request/publish to target COMPLETE 9/18/2014 MWN
 * -Gaussian distribution on cost cloud COMPLETE 9/18/2014 MWN *      
 */

/* TODO: 
 * -move mapping to global level
 * -add pheromone following behavior to mobility
 */


#include <cstdlib>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/Pose2D.h>
#include <sensor_msgs/Range.h>

#include "rover_onboard_target_detection/ATag.h"
#include "rover_onboard_target_detection/harvest.h"
#include "rover_driver_world_state/updateMap.h"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace std;

void obstacleHandler(const std_msgs::UInt8::ConstPtr& message);
void targetHandler(const rover_onboard_target_detection::ATag::ConstPtr& message);
void locationHandler(const geometry_msgs::Pose2D::ConstPtr& message);
void pheromoneHandler(const std_msgs::Bool::ConstPtr& message);
void publishMap(const ros::TimerEvent& e);
void updateMap(int deltaX, int deltaY, int targetSize, int cost);
void decayMap(const ros::TimerEvent& e);
void markObstacles();
//void markPrizes();
void markHarvest();
void markTravelPath();
void markPheremonePath();
float constrain(float value, float minVal, float maxVal);

void leftHandler(const sensor_msgs::Range::ConstPtr& message);
void centerHandler(const sensor_msgs::Range::ConstPtr& message);
void rightHandler(const sensor_msgs::Range::ConstPtr& message);

bool mapHarvestHandler(rover_onboard_target_detection::harvest::Request &req, rover_onboard_target_detection::harvest::Response &res);

char host[128];

const int MAP_SIZE = 300; //in decimeters
const int MAP_ORIGIN_X = MAP_SIZE / 2;
const int MAP_ORIGIN_Y = MAP_SIZE / 2;

cv::Mat robotMap = cv::Mat(MAP_SIZE, MAP_SIZE, CV_8UC1, cv::Scalar(128));
cv_bridge::CvImagePtr rosMap;

ros::Subscriber targetSubscriber;
ros::Subscriber obstacleSubscriber;
ros::Subscriber locationSubscriber;
ros::Subscriber pheromoneSubscriber;

ros::Subscriber leftSubscriber;
ros::Subscriber centerSubscriber;
ros::Subscriber rightSubscriber;

ros::Timer publishTimer;

ros::Timer decayTimer;

ros::ServiceClient mapUpdateClient;
rover_driver_world_state::updateMap mapUpdateMessage;

bool depositingPheromones = false;
float currentHeading;
int currentX = MAP_ORIGIN_X;
int currentY = MAP_ORIGIN_Y;
int obstacleMode = 0;
const double SIDE_COL = (25.0 * M_PI) / 180.0;
const double CTR_SIDE_COL = 12.5 * M_PI / 180.0;

double leftVal = 500.0;
double centerVal = 500.0;
double rightVal = 500.0;

const int CONFIDENCE_LOW = 1; // unused
const int CONFIDENCE_MED = 5; 
const int CONFIDENCE_HIGH = 14;
const int MAP_UPPER_THRESH = 254;
const int SONAR_OFFSET = 5; // unused

rover_onboard_target_detection::ATag foundTagList;

/*
 * Hard Coded Parameters, consider moving to param server
 */
int obstacleCost = 25; //amount to adjust map for found obstacles 
int obstacleSize = 2; //how many map cells to mark for each obstacle detection
//int prizeCost = -15; //amount to adjust map for found prizes
//int prizeSize = 1; //how many map cells to mark for each prize
int harvestCost = -100; //amount to adjust map for harvested prize
int harvestSize = 3; //how many map cells to adjust for harvested prize
int pathCost = 1; //amount to adjust map for the traversed path
int pathSize = 1; //how many map cells to mark for the traversed path
int pheromoneCost = -25; //amount to adjust map for the deposited pheromone
int pheromoneSize = 1; //how many map cells to mark for deposited pheromone
int mapDecayCost = 0; //how much to regress cell cost towards mean each decay cycle
int mapDecayTime = 60; //seconds between decay cycle


int tagsFound = 0;

/*////////////////////////
  Battery Vars
*//////////////////////
void batteryHandler(const std_msgs::Float32& message);
void chargingStateHandler(const std_msgs::Bool message);

ros::Subscriber batterySubscriber;//Subscribe to a battery topic
ros::Subscriber chargingStateSubscriber;

bool isDead = false;
bool isCharging = false;

float batteryLevel;

int main(int argc, char** argv) {

    gethostname(host, sizeof (host));
    string hostname(host);
    string publishedName;
    
    if (argc >= 2) {
        publishedName = argv[1];
        cout << "Welcome to the world of tomorrow " << publishedName << "!  Mapping module started." << endl;
    } else {
        publishedName = hostname;
        cout << "No Name Selected. Default is: " << publishedName << endl;
    }

    ros::init(argc, argv, (publishedName + "_MAPPING"));
    
    ros::NodeHandle mNH;

    obstacleSubscriber = mNH.subscribe((publishedName + "/obstacle"), 10, obstacleHandler);
    targetSubscriber = mNH.subscribe((publishedName + "/targets"), 10, targetHandler);
    locationSubscriber = mNH.subscribe((publishedName + "/location"), 10, locationHandler);
    pheromoneSubscriber = mNH.subscribe((publishedName + "/pheromone"), 10, pheromoneHandler);
    
    leftSubscriber = mNH.subscribe((publishedName + "/USLeft"), 10, leftHandler);
    centerSubscriber = mNH.subscribe((publishedName + "/USCenter"), 10, centerHandler);
    rightSubscriber = mNH.subscribe((publishedName + "/USRight"), 10, rightHandler);
    batterySubscriber = mNH.subscribe((publishedName + "/battery"), 10, batteryHandler);
chargingStateSubscriber = mNH.subscribe((publishedName + "/chargingState"), 10, chargingStateHandler);

    publishTimer = mNH.createTimer(ros::Duration(1.0), publishMap);
    decayTimer = mNH.createTimer(ros::Duration(mapDecayTime), decayMap);

    ros::ServiceServer mapHarvestService = mNH.advertiseService("mapHarvestSrv", mapHarvestHandler);
    mapUpdateClient = mNH.serviceClient<rover_driver_world_state::updateMap>("updateMapSrv");
    
    while (ros::ok()) {
        ros::spin();
    }

    return EXIT_SUCCESS;
}

void obstacleHandler(const std_msgs::UInt8::ConstPtr& message) {
    obstacleMode = (int) message->data;
}

void targetHandler(const rover_onboard_target_detection::ATag::ConstPtr& message) {
    tagsFound = (int) message->tagsFound;
}

void locationHandler(const geometry_msgs::Pose2D::ConstPtr& message) {
    currentHeading = (float) message->theta;
    currentX = message->x * 10; //convert meters to decimeters
    currentY = message->y * 10; //convert meters to decimeters
}

void pheromoneHandler(const std_msgs::Bool::ConstPtr& message) {
    depositingPheromones = message->data;
}

void publishMap(const ros::TimerEvent& e) {    
    if(obstacleMode > 1) { markObstacles(); }
    //if(tagsFound > 0) {markPrizes(); }
    //markTravelPath();
    markPheremonePath();
}

void markObstacles() {
    int deltaX = 0;
    int deltaY = 0;
    int thisSize = obstacleSize;
    double radHeading = currentHeading * M_PI / 180;
    
    //calculate coordinates of detected target
    if (obstacleMode == 2) { //center collision
        deltaX = round(centerVal * 10 * cos(radHeading - M_PI / 2));
        deltaY = round(centerVal * 10 * sin(radHeading + M_PI / 2));
    } else if (obstacleMode == 3) { // left collision
        deltaX = round(leftVal * 10 * cos(radHeading - M_PI / 2 - SIDE_COL));
        deltaY = round(leftVal * 10 * sin(radHeading + M_PI / 2 - SIDE_COL));
    } else if (obstacleMode == 4) { // right collision
        deltaX = round(rightVal * 10 * cos(radHeading - M_PI / 2 + SIDE_COL));
        deltaY = round(rightVal * 10 * sin(radHeading + M_PI / 2 + SIDE_COL));
    } else if (obstacleMode == 5) { // left center collision
        deltaX = round(leftVal * 10 * cos(radHeading - M_PI / 2 - CTR_SIDE_COL));
        deltaY = round(leftVal * 10 * sin(radHeading + M_PI / 2 - CTR_SIDE_COL));
    } else if (obstacleMode == 6) { // right center collision
        deltaX = round(rightVal * 10 * cos(radHeading - M_PI / 2 + CTR_SIDE_COL));
        deltaY = round(rightVal * 10 * sin(radHeading + M_PI / 2 + CTR_SIDE_COL));
    } else if (obstacleMode == 7) { // left right collision
        deltaX = round(rightVal * 10 * cos(radHeading - M_PI / 2 + SIDE_COL));
        deltaY = round(rightVal * 10 * sin(radHeading + M_PI / 2 + SIDE_COL));
    } else if (obstacleMode == 8) { // all 3 collision
        deltaX = round(centerVal * 10 * cos(radHeading - M_PI / 2));
        deltaY = round(centerVal * 10 * sin(radHeading + M_PI / 2));
        thisSize = obstacleSize * 3;
    }

    if(!isDead && !isCharging){
    updateMap(deltaX, deltaY, thisSize, obstacleCost);
    }
}

//void markPrizes() {
//    int deltaX = 0;
//    int deltaY = 0;
//    double radHeading = currentHeading * M_PI / 180;
//    
//    for(int i = 0; i < tagsFound; i++) {
//        deltaX = round(centerVal * 10 * cos(radHeading - M_PI / 2));// TODO: don't use ultrasonic center value here!
//        deltaY = round(centerVal * 10 * sin(radHeading + M_PI / 2));// TODO: don't use ultrasonic center value here!
//        if (!isDead){
//			updateMap(deltaX, deltaY, prizeSize, prizeCost);
//			}
//    }        
//}

void markHarvest() {
    int deltaX = 0;
    int deltaY = 0;
    double radHeading = currentHeading * M_PI / 180;
    
    deltaX = round(centerVal * 10 * cos(radHeading - M_PI / 2));// TODO: don't use ultrasonic center value here!
    deltaY = round(centerVal * 10 * sin(radHeading + M_PI / 2));// TODO: don't use ultrasonic center value here!
    if (!isDead){
        updateMap(deltaX, deltaY, harvestSize, harvestCost);
    }

}

void markTravelPath() {
    if (!isDead){
       updateMap(0, 0, pathSize, pathCost);
    }

}

void markPheremonePath() {
    if(depositingPheromones && !isDead) {
        updateMap(0, 0, pheromoneSize, pheromoneCost );
    }
}

void updateMap(int deltaX, int deltaY, int targetSize, int cost) {
    int targetX = deltaX + currentX;
    int targetY = deltaY + currentY;
    
    //request world state module to update master map
    mapUpdateMessage.request.X = targetX;
    mapUpdateMessage.request.Y = targetY;
    mapUpdateMessage.request.targetSize = targetSize;
    mapUpdateMessage.request.cost = cost;    
    mapUpdateClient.call(mapUpdateMessage);
    
    if ( cost != 1 ) {
    cout << "Called map update service" << targetX << ", " << targetY << ", " << targetSize << ", " << cost << endl;
    }
}            

void decayMap(const ros::TimerEvent& e) {
    for (int i = 0; i < MAP_SIZE; i++) {
        for (int j = 0; j < MAP_SIZE; j++) {
            if (robotMap.at<uchar>(i, j) == 128) {

            } else if (robotMap.at<uchar>(i, j) > 128) {
                robotMap.at<uchar>(i, j) = robotMap.at<uchar>(i, j) - mapDecayCost;
            } else {
                robotMap.at<uchar>(i, j) = robotMap.at<uchar>(i, j) + mapDecayCost;
            }
        }
    }
}

bool mapHarvestHandler(rover_onboard_target_detection::harvest::Request &req, 
                        rover_onboard_target_detection::harvest::Response &res) 
{
    markHarvest();
    return true;
}

void leftHandler(const sensor_msgs::Range::ConstPtr& message) {
    leftVal = message->range;
}

void centerHandler(const sensor_msgs::Range::ConstPtr& message) {
    centerVal = message->range;
}

void rightHandler(const sensor_msgs::Range::ConstPtr& message) {
    rightVal = message->range;
}

float constrain(float value, float minVal, float maxVal) {
    return max(minVal,(float) min(maxVal,value));
}

void batteryHandler(const std_msgs::Float32& message){
    batteryLevel = message.data;

    if (batteryLevel <= 0.65){
        isDead =true;
    }
}

void chargingStateHandler(const std_msgs::Bool message){
  isCharging = message.data;
}


 