#include <string.h>
#include <unistd.h>  
#include <sstream>
#include <vector>
#include <usbSerial.h>
#include <math.h>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt64.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Quaternion.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <tf/transform_datatypes.h>
#include <tf/tf.h>

using namespace std;

void cmdHandler(const geometry_msgs::Twist::ConstPtr& message);
void serialActivityTimer(const ros::TimerEvent& e);
void publishRosTopics();
void parseData(string data);

char host[128];
char delimiter = ',';
vector<string> dataSet;

USBSerial usb;
char port[] = "/dev/ttyACM0";
const int baud = 115200;
char dataCmd[] = "d\n";
char moveCmd[16];

std_msgs::String rtime;
std_msgs::String status;
sensor_msgs::Range usLeft;
sensor_msgs::Range usCenter;
sensor_msgs::Range usRight;
sensor_msgs::Imu imu;
sensor_msgs::NavSatFix gps;

ros::Subscriber moveSubscriber;

ros::Publisher imuPublish;
ros::Publisher gpsPublish;
ros::Publisher usLeftPublish;
ros::Publisher usCenterPublish;
ros::Publisher usRightPublish;
ros::Publisher timePublish;
ros::Publisher statusPublish;
ros::Timer publishTimer;

float linearSpeed = 0.;
float turnSpeed = 0.;

const float deltaTime = 0.25; //delta t, in seconds

int main(int argc, char **argv) {

    gethostname(host, sizeof (host));
    string hostname(host);
    string publishedName;

    ros::init(argc, argv, (hostname + "_ABRIDGE"));
    usb.openUSBPort(port, baud);

    sleep(5);

    ros::NodeHandle aNH;
    
    if (argc >= 2) {
        publishedName = argv[1];
        cout << "Welcome to the world of tomorrow " << publishedName << "!  ABridge module started." << endl;
    } else {
        publishedName = hostname;
        cout << "No Name Selected. Default is: " << publishedName << endl;
    }

    moveSubscriber = aNH.subscribe((publishedName + "/mobility"), 10, cmdHandler);

    imuPublish = aNH.advertise<sensor_msgs::Imu>((publishedName + "/imu"), 10);
    gpsPublish = aNH.advertise<sensor_msgs::NavSatFix>((publishedName + "/gps"), 10);
    timePublish = aNH.advertise<std_msgs::String>((publishedName + "/time"), 1);
    statusPublish = aNH.advertise<std_msgs::String>((publishedName + "/status"), 1);
    publishTimer = aNH.createTimer(ros::Duration(deltaTime), serialActivityTimer);
    
    usLeftPublish = aNH.advertise<sensor_msgs::Range>((publishedName + "/USLeft"), 10);
    usCenterPublish = aNH.advertise<sensor_msgs::Range>((publishedName + "/USCenter"), 10);
    usRightPublish = aNH.advertise<sensor_msgs::Range>((publishedName + "/USRight"), 10);
    
    usLeft.radiation_type = 0;
    usLeft.field_of_view = 55;
    usCenter.radiation_type = 0;
    usCenter.field_of_view = 55;
    usRight.radiation_type = 0;
    usRight.field_of_view = 55;
    
    status.data = "INITIALIZED";

    ros::spin();

    return EXIT_SUCCESS;
}

void cmdHandler(const geometry_msgs::Twist::ConstPtr& message) {
    // remove artificial factor that was multiplied for simulation. this scales it back down to -1.0 to +1.0
    linearSpeed = (message->linear.x) / 1.5;
    turnSpeed = (message->angular.z) / 8;
    
    if (linearSpeed != 0.) {
        sprintf(moveCmd, "m,%d\n", (int) (linearSpeed * 255));
        usb.sendData(moveCmd);
    } else if (turnSpeed != 0.) {
        sprintf(moveCmd, "t,%d\n", (int) (turnSpeed * 255));
        usb.sendData(moveCmd);
    } else {
        sprintf(moveCmd, "s\n");
        usb.sendData(moveCmd);
    }

    memset(&moveCmd, '\0', sizeof (moveCmd));
}

void serialActivityTimer(const ros::TimerEvent& e) {
    usb.sendData(dataCmd);
    parseData(usb.readData());
    publishRosTopics();
}

void publishRosTopics() {
    timePublish.publish(rtime);
    usLeftPublish.publish(usLeft);
    usCenterPublish.publish(usCenter);
    usRightPublish.publish(usRight);
    imuPublish.publish(imu);
    gpsPublish.publish(gps);
    statusPublish.publish(status);
}

void parseData(string str) {
    istringstream oss(str);
    string word;
    while (getline(oss, word, delimiter)) {
        dataSet.push_back(word);
    }
    if (dataSet.size() >= 14) {
        rtime.data = dataSet.at(0);
        gps.latitude = atof(dataSet.at(1).c_str())*100;
        gps.longitude = atof(dataSet.at(2).c_str())*100;
        imu.orientation = tf::createQuaternionMsgFromYaw(atof(dataSet.at(3).c_str()) * M_PI / 180.0);
        imu.orientation.z = fmod(-1 * imu.orientation.z, 360.0);
        imu.angular_velocity.x = atof(dataSet.at(4).c_str());
        imu.angular_velocity.y = atof(dataSet.at(5).c_str());
        imu.angular_velocity.z = atof(dataSet.at(6).c_str());
        imu.linear_acceleration.x = atof(dataSet.at(7).c_str());
        imu.linear_acceleration.y = atof(dataSet.at(8).c_str());
        imu.linear_acceleration.z = atof(dataSet.at(9).c_str());
        usCenter.range = atof(dataSet.at(10).c_str()) / 100.0;
        usLeft.range = atof(dataSet.at(11).c_str()) / 100.0;
        usRight.range = atof(dataSet.at(12).c_str()) / 100.0;
        int gpsStatus = (int) atof(dataSet.at(13).c_str());
        if(gpsStatus <= 0){
            gps.status.STATUS_NO_FIX; // is this a function call?  
        } else {
            gps.status.STATUS_FIX; // is this a function call? 
        }
    	status.data = "ONLINE";
    }
    
    dataSet.clear();
}
