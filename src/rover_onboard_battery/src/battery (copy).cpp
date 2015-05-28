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
//NOTE: I changed the ticks to increments of 1/1000 instead of 1/100 so that we can have a greater resolution on the battery level

const float DISCHARGE_SCALE = 0.028; //Constant from Karl's research to create the battery discharge curve
const float DISCHARGE_VSHIFT = 0.8; //Constant from Karl's research to create the battery discharge curve

float batteryFull = 1000.0; //The battery percentage where the battery is full. This is for the battery discharge curve and does not change
float batteryReturn; //The battery percentage where the rover will return home from a trial. This is the default value but will change based on Karl's parameters
float chargeLeave; //The battery percentage where the rover will leave the charger and stop charging
float batteryDeadPercent = 0.65; //The battery percentage where the rover is condisdered dead. This is a constant.

double LUT_logit [1000]; //Look up table for the battery discharge curve

float returnTimeTick; //The amount of time ticks it takes for the counter to count down to signal a recharge. This is based on the batteryReturn float
float rechargeRate = 720.0; //Time, in seconds, when to signal the rover to go home and recharge.
float stepTime; //Conversion factor to determine how long to set the ROS timer for
float batteryFullTimeStep;
float batteryReturnTimeStep;
float batteryDeadTimeStep;
int currentMode; //The rover's current mode. The battery module will not "turn on" until the rover is set in auto mode.
float batteryPercentage;

float batteryTime = 0.0; //The time ticks used in the battery discharge curve. At batteryTime = 0 the rover should be fully charged. At batteryTime = 100, the rover is dead

bool isDead = false; //boolean to check if the rover has reached less than 65% battery life.
bool isCharging = false; //Boolean to check if the rover is at home and is charging.
bool startBattery = false;

std_msgs::Float32 batteryLevel; //ROS message for the battery life. This is published on the topic /roverName/battery


string publishedName;
char host[128];


static inline float logitFunction(float x, float scale, float vshift); //Function for the battery discharge curve.
void dischargeBattery(); //Function to discharge the battery
void chargeBattery(); //Function to charge the battery
void batteryCallback(/*const ros::TimerEvent& event*/); //Timer based loop that discharges/charges the battery based on a timer set by the rechargeRate.

void batteryStateHandler(const std_msgs::Bool message); //Updates the isCharging boolean. Listens to /roverName/charging topic
void modeHandler(const std_msgs::UInt8::ConstPtr& message); //Updates the currentMode variable. Listens to /roverName/mode topic
void parameterHandler(const std_msgs::Float32MultiArray message); //Loads the battery charge leave and battery charge return parameters from the MCP

int findTimeStep(float batteryPercentage); //Function to determine the correct time step for the battery discharge function

ros::Publisher batteryPublisher; //instance of the battery publishing node. Publishes the std_msgs::Float32 batteryLevel topic
ros::Timer batteryTimer; //An instance of a ROS timer

ros::Subscriber batteryStateSubscriber; //Instance of the ros subscriber node. Listens to the /roverName/charging topic
ros::Subscriber modeSubscriber; //Instance of the ros subscriber node. Listens to the /roverName/mode topic
ros::Subscriber parameterSubscriber; //Instance of a ros subscriber node to the parameter server;
ros::Subscriber chargingStateSubscriber;

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

    batteryPublisher = bNH.advertise<std_msgs::Float32>((publishedName + "/battery"), 1);

    modeSubscriber = bNH.subscribe((publishedName + "/mode"), 10, modeHandler);
    chargingStateSubscriber = bNH.subscribe((publishedName + "/chargingState"), 10, batteryStateHandler);
    parameterSubscriber = bNH.subscribe((publishedName + "/parameters"), 1, parameterHandler);

    batteryLevel.data = 0.9;
    cout << "Starting battery level: " << batteryLevel.data << endl;

    batteryTime = 0;
    stepTime = 1;
    batteryDeadTimeStep = findTimeStep(batteryDeadPercent);
    while (ros::ok()) {
        ros::Rate r(1 / stepTime);

        if (batteryFullTimeStep == 0 || batteryReturnTimeStep == 0) {//These values cannot be zero. Wait for the parameter server to send these values.
            startBattery = false;
            stepTime = 1;
        } else {
            stepTime = rechargeRate / (batteryReturnTimeStep - batteryFullTimeStep); //Use this function to find the necessary time step based on how soon you want the rover to return. 
            startBattery = true;
            batteryCallback();
        }

        ros::spinOnce();
        r.sleep();

    }

    return EXIT_SUCCESS;
}

void batteryCallback(/*const ros::TimerEvent&*/) {

    if (currentMode == 2 || currentMode == 3) {//If robot is automode and the parameters have been sent

        if (!isDead) {
            /*Debug messages useful for seeing what state the rover is in. Not necessary, but nice to have*/
            cout << "Battery Information for " << publishedName << ": " << endl;
            cout << "I will return home to charge when my batteries reach: " << batteryReturn * 100 << "%." << " Which is every " << rechargeRate / 60.0 << " minutes." << endl; //, or " << rechargeRate << " seconds." << endl;
            cout << "I will leave the charger when my batteries reach: " << chargeLeave * 100 << "%." << endl;
            cout << "My current Battery voltage level is: " << setprecision(4) << batteryLevel.data * 100 << "%" << endl;
            if (batteryLevel.data >= batteryReturn) {
                cout << publishedName << " has about: " << setprecision(3) << ((batteryReturnTimeStep - batteryTime) * stepTime) / 60 << " minutes, or " << (batteryReturnTimeStep - batteryTime) * stepTime << " seconds before returning to recharge." << endl;
            } else {
                if (((batteryDeadTimeStep - batteryTime) * stepTime) > 0) {
                    cout << publishedName << " has about: " << setprecision(3) << ((batteryDeadTimeStep - batteryTime) * stepTime) / 60 << " minutes, or " << (batteryDeadTimeStep - batteryTime) * stepTime << " seconds before dying. Hurry " << publishedName << "!" << endl;
                } else {
                    cout << publishedName << " has ran out of time." << endl;
                }
            }
            cout << publishedName << "'s status: ";
        }

        if (!isCharging) {//Robot is out and about and is not at home charging, discharge the battery
            dischargeBattery();
        }

        if (isCharging) {//Robot has recieved the message from mobility that it is at home and is ready to start charging or it is already charging.
            cout << "I am charging" << endl;
            cout << endl;
            chargeBattery();
        }


        /*Debug messages useful for seeing what state the rover is in. Not necessary, but nice to have*/

        if (batteryLevel.data <= batteryReturn && batteryLevel.data > 0 && !isDead && !isCharging) {//Rover should be returning home to charge
            cout << publishedName << " should be returning home to charge..." << endl;
            cout << endl;
        }
        if (batteryLevel.data <= (batteryFull / 1000) && batteryLevel.data > batteryReturn && !isCharging && !isDead) {//he is out and about
            cout << "My batteries are fine. " << publishedName << " should be finding tags! " << endl;
            cout << endl;
        }
        if (batteryLevel.data <= batteryDeadPercent) {//rover is dead
            if (!isDead) {
                cout << publishedName << " is dead. Go on without me. " << endl;
                cout << endl;
            }

        }
        if (isCharging) {
            cout << publishedName << " is charging. Do not bother me. " << endl;
            cout << endl;
            //cout << endl;
        }

        batteryPublisher.publish(batteryLevel); //If the robot is in automode, publish the battery voltage topic.

    } else {//The rover is not set to automode. There is no need for battery voltage readings at this time
        //cout << "I am in manual mode." << endl;
    }


}

void batteryStateHandler(const std_msgs::Bool message) {
    isCharging = message.data;
}

void modeHandler(const std_msgs::UInt8::ConstPtr & message) {
    currentMode = message->data;
}

void parameterHandler(const std_msgs::Float32MultiArray message) {
    chargeLeave = message.data.at(6);
    batteryReturn = message.data.at(7);
    batteryFullTimeStep = findTimeStep(chargeLeave - 0.01);
    batteryReturnTimeStep = findTimeStep(batteryReturn);
    //cout << batteryFullTimeStep << endl;
    //cout << batteryReturnTimeStep << endl;
    cout << "I will come home when my batteries are at: " << batteryReturn * 100 << "%." << endl;
    cout << "I will leave the charging station when my batteries are at: " << chargeLeave * 100 << "%." << endl;
    cout << endl;
}

static inline float logitFunction(float x, float scale, float vshift) {
    float temp = -((log(x) * scale) - (log(1 - x) * scale)) + vshift;


    if (!isfinite(temp) && temp > 0) {
        temp = 1.0;
    }
    if (!isfinite(temp) && temp < 0) {
        temp = 0.0;
    }
    if (temp > 1.0) {
        temp = 1.0;
    }

    return temp;
}

void chargeBattery() {

    //Change this value to zero if you want to use the discharge curve to charge
    int quickCharge = 0;
    cout << "QuickCharge: " << quickCharge << endl;
    if (quickCharge == 1) {
        cout << "Quick Charge mode engaged." << endl;
        batteryLevel.data = batteryLevel.data + 0.05;
        if (batteryLevel.data >= chargeLeave) {
            batteryLevel.data = chargeLeave;
        }
        batteryTime = 0;
    } else if (quickCharge == 0) {
        if (batteryLevel.data >= chargeLeave) {
            cout << "Battery is full! " << endl;
            return;
        }

        if (!isDead && batteryTime >= 0) {
            batteryLevel.data = logitFunction((float) (batteryTime / batteryFull), DISCHARGE_SCALE, DISCHARGE_VSHIFT);
            //batteryLevel.data = batteryLevel.data + 0.05;
            //cout << "battery var: " << (batteryTime/batteryFull) << endl;
            cout << "Battery is charging. Current batteryLevel is : " << batteryLevel.data << endl;
            //cout << "BatteryTime " << batteryTime <<  endl;
            batteryTime--;
            //cout << batteryTime << endl;

        }
    }
}

void dischargeBattery() {
    float temp;
    if (batteryLevel.data >= chargeLeave && chargeLeave > 0) {
        temp = chargeLeave - 0.01;
        batteryLevel.data = temp;
        batteryTime = batteryFullTimeStep++;
    } else if (batteryLevel.data < chargeLeave) {
        batteryTime++;
        temp = logitFunction((float) ((batteryTime) / batteryFull), DISCHARGE_SCALE, DISCHARGE_VSHIFT);
    } else {
        return;
    }

    if (batteryLevel.data < batteryDeadPercent) { // If battery > 65% then robot is now dead due to battery damage
        isDead = true;
        //cout << "Robot is dead at this battery percentage: " << batteryLevel.data << endl;
        return;
    }
    batteryLevel.data = temp;

}

int findTimeStep(float batteryPercentage) {
    batteryTime = 0;

    for (int i = 0; i < 1000; i++) {
        LUT_logit[i] = logitFunction((float) (batteryTime / batteryFull), DISCHARGE_SCALE, DISCHARGE_VSHIFT);
        batteryTime++;
    }

    double best = LUT_logit[0];
    int bestCounter = 0;
    for (int i = 0; i < 1000; i++) {
        //These two steps check the look up table for the time step that is closest to the desired battery return voltage.
        bestCounter = abs(LUT_logit[i] - batteryPercentage) < abs(best - batteryPercentage) ? i : bestCounter;
        best = abs(LUT_logit[i] - batteryPercentage) < abs(best - batteryPercentage) ? LUT_logit[i] : best;


    }
    //returnTimeTick = bestCounter;
    //stepTime = (rechargeRate/returnTimeTick);

    return bestCounter;
}

/*void findTimeStep(float batteryReturn){
    //Create lookup table for the logitFunction
    batteryTime = 0;

    for (int i = 0; i < 1000; i++){
        LUT_logit[i] = logitFunction((float) (batteryTime/batteryFull), DISCHARGE_SCALE, DISCHARGE_VSHIFT);
        batteryTime++;
    }

    double best = LUT_logit[0];
    int bestCounter = 0;
    for (int i = 0; i < 1000; i++){
        //These two steps check the look up table for the time step that is closest to the desired battery return voltage.
        bestCounter = abs(LUT_logit[i] - batteryReturn) < abs(best - batteryReturn) ? i : bestCounter;
        best = abs(LUT_logit[i] - batteryReturn) < abs(best - batteryReturn) ? LUT_logit[i] : best;

    }
    returnTimeTick = bestCounter;
    stepTime = (rechargeRate/returnTimeTick);

}*/








