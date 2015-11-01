#include <ros/ros.h>

//ROS messages
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/UInt8.h>

using namespace std;

//Globals
double collisionDistance = 0.4; //meters the ultrasonic detectors will flag obstacles
string publishedName;
char host[128];

//Publishers
ros::Publisher obstaclePublish;

//Subscribers
ros::Subscriber sonarSubscriber;

//Callback handlers
void sonarHandler(const std_msgs::Float64MultiArray sonar);

int main(int argc, char** argv) {

    gethostname(host, sizeof (host));
    string hostname(host);

    if (argc >= 2) {
        publishedName = argv[1];
        cout << "Welcome to the world of tomorrow " << publishedName << "! Obstacle module started." << endl;
    } else {
        publishedName = hostname;
        cout << "No name selected. Default is: " << publishedName << endl;
    }

    ros::init(argc, argv, (publishedName + "_OBSTACLE"));
    ros::NodeHandle oNH;
    
    sonarSubscriber = oNH.subscribe((publishedName + "/sonar"), 10, sonarHandler);

    obstaclePublish = oNH.advertise<std_msgs::UInt8>((publishedName + "/obstacle"), 10);

    ros::spin();

    return EXIT_SUCCESS;
}

void sonarHandler(const std_msgs::Float64MultiArray sonar) {
	std_msgs::UInt8 obstacleMode;
	
	if ((sonar.data[0] > collisionDistance) && (sonar.data[0] > collisionDistance) && (sonar.data[0] > collisionDistance)) {
		obstacleMode.data = 0; //no collision
	}
	else if ((sonar.data[0] > collisionDistance) && (sonar.data[2] < collisionDistance)) {
		obstacleMode.data = 1; //collision on right side
	}
	else {
		obstacleMode.data = 2; //collision in front or on left side
	}
	
	obstaclePublish.publish(obstacleMode);
}

