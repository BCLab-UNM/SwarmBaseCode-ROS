/*
 *      This is only for testing the path planning code.  This needs to be moved out of the onboard workspace!!!
 */

// TODO: remove this test code from the onboard workspace!

//#include <string.h>
#include <unistd.h>  
#include <vector>
#include <iostream>
#include <string>
#include <sstream>
#include <signal.h>

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Image.h>
#include <nav_msgs/GetPlan.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace std;
using namespace cv;

char host[128];

const int MAP_SIZE = 300;
const int MAP_ORIGIN_X = MAP_SIZE / 2;
const int MAP_ORIGIN_Y = MAP_SIZE / 2;

Mat robotMap;
cv_bridge::CvImagePtr rosMap;

ros::Publisher mapPublish;

geometry_msgs::Point startPoint;
geometry_msgs::Point stopPoint;
sensor_msgs::Image mapImage;
nav_msgs::Path returnedPath;

void killHandler(int s) {
	exit(0);
}


int main(int argc, char** argv) {

    gethostname(host, sizeof (host));
    string hostname(host);
    string publishedName;

    ros::init(argc, argv, (hostname + "_PATH_TESTER"));

    ros::NodeHandle pNH;

    if (argc >= 2) {
        publishedName = argv[1];
        cout << "Welcome to the world of tomorrow " << publishedName << "! Path planning module started." << endl;
    } else {
        publishedName = hostname;
        cout << "No name selected. Default is: " << publishedName << endl;
    }

    mapPublish = pNH.advertise<sensor_msgs::Image>((publishedName + "/map"), 10, 1);

    ros::ServiceClient svcClient = pNH.serviceClient<nav_msgs::GetPlan>("getPlan");

    ros::NodeHandle mNH;
    image_transport::ImageTransport imgTransport(mNH);
    rosMap = boost::make_shared<cv_bridge::CvImage>();
    rosMap->encoding = sensor_msgs::image_encodings::MONO8;

    while (ros::ok()) {

	cout << "Which map file to publish? " <<  endl;
	char mapFile[255];
	cin.getline(mapFile, 255);
	signal(SIGINT, killHandler);

	// sanity check
	robotMap = imread(mapFile, CV_LOAD_IMAGE_GRAYSCALE);
	if(! robotMap.data )                       
	{
		cout <<  "ERROR: Could not open or find the image specified!" << std::endl ;
		break;
	}

	// publish map image
	rosMap->image = robotMap;
	mapPublish.publish(rosMap->toImageMsg());
        ros::spinOnce();

	string input = "";
	int startXPoint = 0;
	while (true) {
		cout << "Starting X coordinate? ";
		getline(cin, input);
		// This code converts from string to number safely.
		stringstream myStream(input);
		if (myStream >> startXPoint)
			break;
		cout << "Invalid number, please try again" << endl;
	}
	signal(SIGINT, killHandler);

	nav_msgs::GetPlan getPlanService;

	int startYPoint = 0;
	while (true) {
		cout << "Starting Y coordinate? ";
		getline(cin, input);
		// This code converts from string to number safely.
		stringstream myStream(input);
		if (myStream >> startYPoint)
			break;
		cout << "Invalid number, please try again" << endl;
	}
	signal(SIGINT, killHandler);

	getPlanService.request.start.pose.position.x = startXPoint;
	getPlanService.request.start.pose.position.y = startYPoint;

	int stopXPoint = 0;
	while (true) {
		cout << "Ending X coordinate? ";
		getline(cin, input);
		// This code converts from string to number safely.
		stringstream myStream(input);
		if (myStream >> stopXPoint)
			break;
		cout << "Invalid number, please try again" << endl;
	}
	signal(SIGINT, killHandler);

	int stopYPoint = 0;
	while (true) {
		cout << "Ending Y coordinate? ";
		getline(cin, input);
		// This code converts from string to number safely.
		stringstream myStream(input);
		if (myStream >> stopYPoint)
			break;
		cout << "Invalid number, please try again" << endl;
	}
	signal(SIGINT, killHandler);

	getPlanService.request.goal.pose.position.x = stopXPoint;
	getPlanService.request.goal.pose.position.y = stopYPoint;

	// make the service call
	if (svcClient.call(getPlanService))
	{
		cout << "Successful service call. " <<  endl;

		for (int i = 0; i < 2; i++) {
			// convert for image coordinate system
			float pointX = (float) getPlanService.response.plan.poses[i].pose.position.x + (MAP_SIZE/2);
			float pointY = (float) getPlanService.response.plan.poses[i].pose.position.y + (MAP_SIZE/2);
			cout << "Path point (shifted) [" << i << "]: " << pointX << " " << pointY << endl;
		}

	}
	else
	{
		cout << "ERROR: Service call returned an error.  Sorry. " <<  endl;
		return 1;
	}

	cout << "Enter a map file in order to try again. " <<  endl;

	signal(SIGINT, killHandler);

    }

    return 0;
}






