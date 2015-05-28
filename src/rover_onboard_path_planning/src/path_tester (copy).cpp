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

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace std;
using namespace cv;

void pathHandler(const nav_msgs::Path::ConstPtr& message);

char host[128];

const int MAP_SIZE = 300;
const int MAP_ORIGIN_X = MAP_SIZE / 2;
const int MAP_ORIGIN_Y = MAP_SIZE / 2;

Mat robotMap;
//Mat pathImage;
cv_bridge::CvImagePtr rosMap;

ros::Publisher startPublish;
ros::Publisher stopPublish;
ros::Publisher mapPublish;

ros::Subscriber pathSubscriber;

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

    pathSubscriber = pNH.subscribe((publishedName + "/path"), 10, pathHandler);

    startPublish = pNH.advertise<geometry_msgs::Point>((publishedName + "/start"), 10, 1);
    stopPublish = pNH.advertise<geometry_msgs::Point>((publishedName + "/stop"), 10, 1);
    mapPublish = pNH.advertise<sensor_msgs::Image>((publishedName + "/map"), 10, 1);

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
	//pathImage = robotMap; // make copy for drawing on top of

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

	//string input = "";
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

	// publish starting point
	startPoint.x = startXPoint;
	startPoint.y = startYPoint;
	startPublish.publish(startPoint);
        ros::spinOnce();

	//string input = "";
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

	//string input = "";
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

	// publish stopping point
	stopPoint.x = stopXPoint;
	stopPoint.y = stopYPoint;
	stopPublish.publish(stopPoint);
        ros::spinOnce();

	// hopefully the path message will come in and be handled by the handler
	ros::spinOnce();

	// display map image to user
	//namedWindow("Map", WINDOW_AUTOSIZE);
	//imshow("Map", pathImage); 
	//waitKey(100); 

	cout << "You should see the results displayed in the image window. " <<  endl;
	cout << "Enter a map file in order to try again. " <<  endl;

	ros::spinOnce();

	signal(SIGINT, killHandler);
    }

    return 0;
}

void pathHandler(const nav_msgs::Path::ConstPtr& message) {
	cout << "Path Handler was called " << endl;
	//float oldPointX, oldPointY;
	for (int i = 0; i < 8; i++) {
		// convert for image coordinate system
		float pointX = (float) message->poses[i].pose.position.x + (MAP_SIZE/2);
		float pointY = (float) message->poses[i].pose.position.y + (MAP_SIZE/2);
		cout << "Path point (shifted) [" << i << "]: " << pointX << " " << pointY << endl;

		/* if(i > 0) {
 			line(   pathImage,
        			Point( oldPointX, oldPointY ),
        			Point( pointX, pointY ),
        			Scalar( 0, 0, 0 ),
        			2,
        			8 );

			imshow("Map", pathImage); 
			waitKey(100); 

		} */

		//oldPointX = pointX;
		//oldPointY = pointY;

	} 

}


class Node{
    
    int x;
    int y;
    int occupy;
    int tempVal;
    int finalVal;
    vector<Node> neighbors;
    
public:
    
    Node(int x, int y);
    Node(int x, int y, int c);

};

Node::Node(int x , int y){
    x = x;
    y = y;
}

Node::Node(int x, int y, int c){
    Node(x,y);
    occupy = c;
}



