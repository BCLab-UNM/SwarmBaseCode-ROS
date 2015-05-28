#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <sstream>
#include <iostream>
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
#include <geometry_msgs/Pose2D.h>

using namespace std;

// no mixing whole number percentages and decimal percentages!
// since parameters come in as decimals, then everything else will use percentages!

const float DISCHARGE_SCALE = 0.028; //Constant from Karl's research to create the battery discharge curve
const float DISCHARGE_VSHIFT = 0.8; //Constant from Karl's research to create the battery discharge curve

const float batteryFullPercent = 1.0; //The battery percentage where the battery is full for the battery discharge curve
const float batteryDeadPercent = 0.65; //The battery percentage where the battery is condisdered dead for the battery discharge curve
const float simTimeMinutes = 60; // minutes of sim time
const float percentOfSimTime = 0.40; // percentage of sim time we want robots to be discharded
const int secondsPerMinute = 60; // seconds in a minute
const int totalDischargeTime = simTimeMinutes * secondsPerMinute * percentOfSimTime; //Time to discharge from full to dead in seconds

const int totalStepsInTable = 100;
double LUT_logit [totalStepsInTable]; //Look up table for the battery discharge curve

float batteryReturnParam = 0; //The battery percentage where the rover will return home from a trial, set by user parameter
float chargeLeaveParam = 0; //The battery percentage where the rover will leave the charger and stop charging, set by user parameter

float mainLoopSleepTimeSeconds = 1.0; // spin rate of main loop in seconds.  Amount to sleep between main loop executions.

int batteryFullTimeStep = 0; // time step in curve where battery is completely full
int roverLeaveTimeStep = 0; // time step in curve where battery is full enough to leave charger
int roverReturnTimeStep = 0; // time step in curve where battery is low enough to return to charger
int batteryDeadTimeStep = 0; // time step in curve where battery is completely dead
int batteryActualTimeStep = 0; //current time step of the battery right now

int currentMode = 0; //The rover's current operating mode. This software waits for auto mode.

bool isDead = false; //boolean to check if the rover has reached less than 65% battery life.
bool isCharging = false; //Boolean to check if the rover is at home and is charging.

std_msgs::Float32 batteryLevel; //ROS message for the current battery voltage. This is published on the topic /roverName/battery

string publishedName = "";
char host[128];

void dischargeBattery(void); //Function to call when battery is being used
void chargeBattery(void); //Function to call when battery is being charged
void doBatteryCalculations(void); // convenience routine. gets called once each main loop iteration to do a bunch of stuff
void createLUT(void); // Actually creates the battery discharge table.  call this first!
float logitFunction(float x, float scale, float vshift); //Math function to create the battery discharge curve.
int findTimeStep(float batteryPercentage); //Function to search for closest time step within the battery discharge function

void batteryStateHandler(const std_msgs::Bool message); //Updates the isCharging boolean. Listens to /roverName/charging topic
void modeHandler(const std_msgs::UInt8::ConstPtr& message); //Updates the currentMode variable. Listens to /roverName/mode topic
void parameterHandler(const std_msgs::Float32MultiArray message); //Loads the battery charge leave and battery charge return parameters from the MCP

ros::Publisher batteryPublisher; //instance of the battery publishing node. Publishes the std_msgs::Float32 batteryLevel topic

ros::Subscriber modeSubscriber; //Instance of the ros subscriber node. Listens to the /roverName/mode topic
ros::Subscriber parameterSubscriber; //Instance of a ros subscriber node to the parameter server;
ros::Subscriber chargingStateSubscriber;// listens to robot to know if it is charging or not

int main(int argc, char **argv) {

    gethostname(host, sizeof (host));
    string hostname(host);

    srand(time(NULL));

    if (argc >= 2) {
        publishedName = argv[1];
        cout << "Welcome to the world of tomorrow " << publishedName << "!  Battery module started." << endl;
    } else {
        publishedName = hostname;
        cout << "No Name Selected. Default is: " << publishedName << endl;
    }

    ros::init(argc, argv, (publishedName + "_BATTERY"));
    ros::NodeHandle bNH;
    batteryPublisher = bNH.advertise<std_msgs::Float32>((publishedName + "/battery"), 1, true);

    modeSubscriber = bNH.subscribe((publishedName + "/mode"), 10, modeHandler);
    chargingStateSubscriber = bNH.subscribe((publishedName + "/chargingState"), 10, batteryStateHandler);
    parameterSubscriber = bNH.subscribe((publishedName + "/parameters"), 1, parameterHandler);

    // create the look up table
    createLUT();
    batteryFullTimeStep = findTimeStep(batteryFullPercent);
    cout << "Battery FULL time step: " << batteryFullTimeStep << endl;
    batteryDeadTimeStep = findTimeStep(batteryDeadPercent);
    cout << "Battery DEAD time step: " << batteryDeadTimeStep << endl;

    // battery needs a starting point
    float startingBatteryLevel = 0.90;
    batteryLevel.data = startingBatteryLevel;
    batteryPublisher.publish(batteryLevel); 
    ros::spinOnce();
    cout << "Starting battery level: " << startingBatteryLevel << endl;
    batteryActualTimeStep = findTimeStep(startingBatteryLevel);
    cout << "Battery CURRENT time step: " << batteryActualTimeStep << endl;

    // forever loop
    while (ros::ok()) {
        ros::Rate r(1 / mainLoopSleepTimeSeconds);

        if (currentMode == 2 || currentMode == 3) { // auto mode
            mainLoopSleepTimeSeconds = (totalDischargeTime / totalStepsInTable); 
            doBatteryCalculations(); 
        } else { /// wait for auto mode
            mainLoopSleepTimeSeconds = 1.0;
       }

        ros::spinOnce();
        r.sleep();

    } // forever loop

    return EXIT_SUCCESS;
}

void doBatteryCalculations(/*const ros::TimerEvent&*/) {

        if (!isDead) {
            /*Debug messages useful for seeing what state the rover is in. Not necessary, but nice to have*/
            cout << "Battery Information for " << publishedName << ": " << endl;
            cout << "It takes " << (float) totalDischargeTime / 60.0 << " minutes to discharge from 100% to 65%." << endl; 
            cout << "Return to charger level is: " << 
		batteryReturnParam * 100 << "%. " <<
            	"Leave the charger level is: " << chargeLeaveParam * 100 << "%." << endl;
            cout << "Current Battery voltage level is: " << setprecision(4) << batteryLevel.data * 100 << "%" << endl;

            cout << publishedName << "'s status: ";
        } // not dead

        if (!isCharging) {//Robot is out and about and is not at home charging, discharge the battery
            dischargeBattery();
        } // not charging

        if (isCharging) {//Robot has recieved the message from mobility that it is at home and is ready to start charging or it is already charging.
            cout << "I am charging" << endl;
            cout << endl;
            chargeBattery();
        } // charging

        /*Debug messages useful for seeing what state the rover is in. Not necessary, but nice to have*/

        if ((batteryLevel.data <= batteryReturnParam) && (batteryLevel.data > 0) && !isDead && !isCharging) {
	    //Rover should be returning home to charge
            cout << publishedName << " should be returning home to charge..." << endl;
            cout << endl;
        }

        if ((batteryLevel.data <= batteryFullPercent) && (batteryLevel.data > batteryReturnParam) && !isCharging && !isDead) {
	    //he is out and about
            cout << "My batteries are fine. " << publishedName << " should be out looking for tags." << endl;
            cout << endl;
        }

        if (batteryLevel.data <= batteryDeadPercent) {
	    //rover is dead
            if (!isDead) { // latch to make it only print once
                cout << publishedName << " just died. Go on without me. " << endl;
                cout << endl;
            }

        }

        batteryPublisher.publish(batteryLevel); //publish the battery voltage topic every cycle

}

void batteryStateHandler(const std_msgs::Bool message) {
    isCharging = message.data;
}

void modeHandler(const std_msgs::UInt8::ConstPtr & message) {
    currentMode = message->data;
}

void parameterHandler(const std_msgs::Float32MultiArray message) {
    chargeLeaveParam = message.data.at(6);
    batteryReturnParam = message.data.at(7);
    roverLeaveTimeStep = findTimeStep(chargeLeaveParam - 0.01);
    cout << "Rover LEAVE time step: " << roverLeaveTimeStep << endl;
    roverReturnTimeStep = findTimeStep(batteryReturnParam);
    cout << "Rover RETURN time step: " << roverReturnTimeStep << endl;

    cout << "Rover should come home when batteries are below: " << batteryReturnParam * 100 << "%." << endl;
    cout << "Rover should leave the charging station when batteries are above: " << chargeLeaveParam * 100 << "%." << endl;
    cout << endl;
}

float logitFunction(float x, float scale, float vshift) {
    float result = -((log(x) * scale) - (log(1 - x) * scale)) + vshift;

    if (result > 1.0) {
        result = 1.0;
    }
    if (result < 0.0) {
        result = 0.0;
    }

    return result;
}

void chargeBattery() {
    batteryActualTimeStep--;
    if (batteryActualTimeStep < 0) {
	batteryActualTimeStep = 0;
    }
    batteryLevel.data = LUT_logit[batteryActualTimeStep];

}

void dischargeBattery() {
    if (batteryLevel.data < batteryDeadPercent) { // If battery < 65% then robot is now dead due to battery damage
        isDead = true;
        cout << "Robot is dead at this battery percentage: " << batteryLevel.data << endl;
    }

    batteryActualTimeStep++;
    if (batteryActualTimeStep > 99) {
	batteryActualTimeStep = 99;
    }
    batteryLevel.data = LUT_logit[batteryActualTimeStep];

}

int findTimeStep(float batteryPercentage) {
    double best = LUT_logit[0];
    int bestCounter = 0;
    for (int i = 0; i < 100; i++) {
        //These two steps check the look up table for the time step that is closest to the desired battery return voltage.
        bestCounter = abs(LUT_logit[i] - batteryPercentage) < abs(best - batteryPercentage) ? i : bestCounter;
        best = abs(LUT_logit[i] - batteryPercentage) < abs(best - batteryPercentage) ? LUT_logit[i] : best;


    }

    return bestCounter;
}

void createLUT(void) {
    for (int i = 0; i < totalStepsInTable; i++) {
        LUT_logit[i] = logitFunction((i/100.0), DISCHARGE_SCALE, DISCHARGE_VSHIFT);
    }

    LUT_logit[0] = batteryFullPercent;
    LUT_logit[totalStepsInTable - 1] = batteryDeadPercent;

    return;
}






