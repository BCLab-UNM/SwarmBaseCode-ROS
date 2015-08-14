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
#include <sensor_msgs/NavSatFix.h>
#include <tf/transform_datatypes.h>
#include <tf/tf.h>

using namespace std;

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
sensor_msgs::NavSatFix gps;

ros::Publisher gpsPublish;
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

    ros::init(argc, argv, (hostname + "_GPS"));
    usb.openUSBPort(port, baud);

    sleep(5);

    ros::NodeHandle aNH;
    
    if (argc >= 2) {
        publishedName = argv[1];
        cout << "Welcome to the world of tomorrow " << publishedName << "!  GPS module started." << endl;
    } else {
        publishedName = hostname;
        cout << "No Name Selected. Default is: " << publishedName << endl;
    }

    gpsPublish = aNH.advertise<sensor_msgs::NavSatFix>((publishedName + "/gps"), 10);
    timePublish = aNH.advertise<std_msgs::String>((publishedName + "/time"), 1);
    statusPublish = aNH.advertise<std_msgs::String>((publishedName + "/status"), 1);
    publishTimer = aNH.createTimer(ros::Duration(deltaTime), serialActivityTimer);
    
    status.data = "INITIALIZED";

    ros::spin();

    return EXIT_SUCCESS;
}

void serialActivityTimer(const ros::TimerEvent& e) {
    usb.sendData(dataCmd);
    parseData(usb.readData());
    publishRosTopics();
}

void publishRosTopics() {
    timePublish.publish(rtime);
    gpsPublish.publish(gps);
    statusPublish.publish(status);
}

void parseData(string str) {
    cout << str;
    istringstream oss(str);
    string word;
    while (getline(oss, word, delimiter)) {
        dataSet.push_back(word);
    }
    if (dataSet.size() == 10) {
    	status.data = "ONLINE";
    }
    
    dataSet.clear();
}
