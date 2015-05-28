/*
 * Author: Kurt Leucht
 * Maintainer: Kurt Leucht
 * Email: kurt.leucht@nasa.gov
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
 *      
 */

#include <string.h>
#include <unistd.h>  
#include <sstream>
#include <vector>
#include <math.h>

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/GetPlan.h>
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

void startHandler(const geometry_msgs::Point::ConstPtr& message);
void stopHandler(const geometry_msgs::Point::ConstPtr& message);
void mapHandler(const sensor_msgs::Image::ConstPtr& message);

char host[128];

const int MAP_SIZE = 300;
const int MAP_ORIGIN_X = MAP_SIZE / 2;
const int MAP_ORIGIN_Y = MAP_SIZE / 2;

#define PI 3.14159265

// TODO: Make this part of the parameter server and read it every service call
int obstacleIntensityThresholdValue = 200; // 0 to 255, below which is not considered an obstacle to avoid in the path
int obstacleIntensityBalloonValue = 10; // 0 to 100, lower number is bigger ballooning
int angleToIncrement = 5;
int chordToLengthen = 4;

Mat thresholdImage(300, 300, CV_8UC1);
cv_bridge::CvImagePtr pathCreationImage;
cv_bridge::CvImagePtr finalPathImage;
cv_bridge::CvImagePtr originalMapImage;

ros::Publisher pathCreationImagePublisher;
ros::Publisher finalPathImagePublisher;

ros::Subscriber mapSubscriber;

geometry_msgs::Point startUser;
geometry_msgs::Point goalUser;

bool getPlan(	nav_msgs::GetPlan::Request  &req,
		nav_msgs::GetPlan::Response &res	)
{
	cout << "getPlan service was called." << endl;

	startUser.x = req.start.pose.position.x;
	startUser.y = req.start.pose.position.y;
	cout << "Start point: " << startUser.x << " " << startUser.y << endl;

	goalUser.x = req.goal.pose.position.x;
	goalUser.y = req.goal.pose.position.y;
	cout << "Stop point: " << goalUser.x << " " << goalUser.y << endl;

	// TODO: Decide whether tolerance would be useful to use someday
	//float32 req.tolerance

	// initialize map images to publish at the end of this service call
	cvtColor(thresholdImage, pathCreationImage->image, CV_GRAY2RGB);
	cvtColor(originalMapImage->image, finalPathImage->image, CV_GRAY2RGB);

	// set up global start and goal variables, adjust for image coordinate system center point
	Point globalStartPointImageCoord;
	globalStartPointImageCoord.x = startUser.y + (MAP_SIZE/2);
	globalStartPointImageCoord.y = startUser.x + (MAP_SIZE/2);
	Point globalGoalPointImageCoord;
	globalGoalPointImageCoord.x = goalUser.y + (MAP_SIZE/2);
	globalGoalPointImageCoord.y = goalUser.x + (MAP_SIZE/2);

	// set up local start and goal variables
	Point localStartPointImageCoord = globalStartPointImageCoord;
	Point localGoalPointImageCoord = globalGoalPointImageCoord;
	int totalPoints = 1;
	int angleAwayFromGoal = 0;

	// add this goal point to Path array, adjust for image coordinate system center point
	res.plan.poses.resize(totalPoints);
	res.plan.poses[totalPoints-1].pose.position.x = localStartPointImageCoord.x - (MAP_SIZE/2); 
	res.plan.poses[totalPoints-1].pose.position.y = localStartPointImageCoord.y - (MAP_SIZE/2); 

	bool globalGoalReached = false;
	int iterations = 0;
	while(!globalGoalReached) {

		bool foundBoundary = false;
		bool metLocalGoal = false;
		iterations++;

		// calculate bearing for current leg, rotate 90 degrees
		float currentBearing = 90 + (atan2(
			(localGoalPointImageCoord.y-localStartPointImageCoord.y), 
			(localGoalPointImageCoord.x-localStartPointImageCoord.x)	
			) * 180 / PI);
		if (currentBearing >= 360) {
			currentBearing = currentBearing - 360;
		}
		if (currentBearing < 0) {
			currentBearing = currentBearing + 360;
		}
		cout << "Bearing to local goal: " << currentBearing << endl;

		// iterate a line between local start and local goal
		LineIterator myIterator(thresholdImage, 
				localStartPointImageCoord, 
				localGoalPointImageCoord, 
				8);
		vector<Point> points(myIterator.count);
		int lineIterCount;
		cout << "Intensity: " << endl;
		for(lineIterCount = 0; lineIterCount < myIterator.count; lineIterCount++, ++myIterator)
		{
			points[lineIterCount] = myIterator.pos();
			// get pixel intensity at this point
			int intensity = thresholdImage.at<uchar>(points[lineIterCount]);
			cout << intensity << " (" << myIterator.pos().x << ", " << myIterator.pos().y << ") " ;

			// look for boundary
			if(intensity > 127) {
				cout << endl;
				cout << "Boundary was found at: " << myIterator.pos().x << " " << 
							myIterator.pos().y << endl;
				foundBoundary = true;

				// draw this failed try in a lighter color, previous iteration's point, though
				line(pathCreationImage->image, 
					localStartPointImageCoord, 
					points[lineIterCount-1], // TODO: this is not crash proof.  Assumes line is longer than one pixel!
					Scalar(20, 20, 230), 1, 4);

				break;
			}

			// look for local goal met
			if(myIterator.count == lineIterCount + 1) {
				cout << endl;
				cout << "Local goal was met. " << endl;
				metLocalGoal = true;
				break;
			}

		} // end line iterator

		// handle boundary by changing local goal and let it try again (don't paint line)
		if(foundBoundary) {
			// grab bearing for use later
			float calcBearing = currentBearing - 90;
			if (calcBearing >= 360) {
				calcBearing = calcBearing - 360;
			}
			if (calcBearing < 0) {
				calcBearing = calcBearing + 360;
			}

			// calculate distance of short chord to boundary
			int xDiff = myIterator.pos().x - localStartPointImageCoord.x;
			int yDiff = myIterator.pos().y - localStartPointImageCoord.y;
			float shortCordLength = sqrt( (xDiff * xDiff) + (yDiff * yDiff) );
			cout << "short chord length: " << shortCordLength << endl;
			// lock in a minimum chord length
			if (shortCordLength < 20) {
				shortCordLength = 20;
			}

			// TODO: try turning to the right and to the left and compare
			// the final resulting cleared chord.  test each's distance to global goal.

			// change the bearing to search for a clearing in a slightly new direction
			float newBearing = calcBearing + angleToIncrement;
			if (newBearing >= 360) {
				newBearing = newBearing - 360;
			}
			if (newBearing < 0) {
				newBearing = newBearing + 360;
			}
			// this bearing is only for display, the above one is used for calculations
			float testBearing = newBearing + 90;
			if (testBearing >= 360) {
				testBearing = testBearing - 360;
			}
			if (testBearing < 0) {
				testBearing = testBearing + 360;
			}
			cout << "test bearing: " << testBearing << endl;

			// calculate new temporary goal coordinates, add a little length to short chord
			localGoalPointImageCoord.x = localStartPointImageCoord.x + 
							( ( (int)shortCordLength + chordToLengthen) * 
								cos(newBearing * PI / 180) );
			localGoalPointImageCoord.y = localStartPointImageCoord.y + 
							( ( (int)shortCordLength + chordToLengthen) * 
								sin(newBearing * PI / 180) );
			cout << "new temporary goal: " << localGoalPointImageCoord.x << " " << 
							localGoalPointImageCoord.y << endl;
		}

		// handle local goal found
		if(metLocalGoal) {
			angleAwayFromGoal = 0;

			// paint the line for this leg on the path creation image, previous iteration's point, though
			line(pathCreationImage->image, 
				localStartPointImageCoord, 
				//localGoalPointImageCoord, 
				points[lineIterCount-1], // TODO: this is not crash proof.  Assumes line is longer than one pixel!
				Scalar(230, 20, 20), 1, 4);

			// paint the line for this leg on the final path image
			line(finalPathImage->image, 
				localStartPointImageCoord, 
				localGoalPointImageCoord, 
				Scalar(230, 20, 20), 1, 4);

			// add this goal point to Path array, adjust for image coordinate system center point
			totalPoints++;
			res.plan.poses.resize(totalPoints);
			res.plan.poses[totalPoints-1].pose.position.x = localGoalPointImageCoord.x - (MAP_SIZE/2); 
			res.plan.poses[totalPoints-1].pose.position.y = localGoalPointImageCoord.y - (MAP_SIZE/2); 

			// if also is global goal, then we're done
			if(localGoalPointImageCoord == globalGoalPointImageCoord) {
				cout << "Global goal was met. " << endl;
				globalGoalReached = true;

			}
			else {
				// we met local goal but not global goal yet
				// we need to set the local goal to the global goal and try again
				// we also need to set a new start point
				localStartPointImageCoord = myIterator.pos();
				localGoalPointImageCoord = globalGoalPointImageCoord;
			}

		}

		// bomb out if things go badly
		if (iterations > 500) {
			cout << "Oops.  The algorithm is likely stuck.  Quitting. " << endl;
			// TODO: Should probably return an error to caller, or at least zero out the Path message
			break;
		}

	} // end while goal not reached

	// publish the path creation image
	pathCreationImagePublisher.publish(pathCreationImage->toImageMsg());
	ros::spinOnce();

	// publish the final path image
	finalPathImagePublisher.publish(finalPathImage->toImageMsg());
	ros::spinOnce();

	res.plan.header.frame_id = "TBD";

 	return true;
}

int main(int argc, char** argv) {

	gethostname(host, sizeof (host));
	string hostname(host);
	string publishedName;

	ros::init(argc, argv, (hostname + "_PATH"));
	ros::NodeHandle pNH;

	if (argc >= 2) {
		publishedName = argv[1];
		cout << "Welcome to the world of tomorrow " << publishedName << "! Path planning module started." << endl;
	} else {
		publishedName = hostname;
		cout << "No name selected. Default is: " << publishedName << endl;
	}

	mapSubscriber = pNH.subscribe((publishedName + "/map"), 10, mapHandler);

	pathCreationImagePublisher = pNH.advertise<sensor_msgs::Image>((publishedName + "/pathalgorithm"), 10);
	finalPathImagePublisher = pNH.advertise<sensor_msgs::Image>((publishedName + "/mappluspath"), 10);

	ros::ServiceServer service = pNH.advertiseService("getPlan", getPlan);

	image_transport::ImageTransport imgTransport(pNH);
	pathCreationImage = boost::make_shared<cv_bridge::CvImage>();
	pathCreationImage->encoding = sensor_msgs::image_encodings::BGR8;
	finalPathImage = boost::make_shared<cv_bridge::CvImage>();
	finalPathImage->encoding = sensor_msgs::image_encodings::BGR8;

	// spin pumping callbacks
	ros::spin();

	return 0;
}


void mapHandler(const sensor_msgs::Image::ConstPtr& message) {
	cv_bridge::CvImagePtr threshImage;
	cv_bridge::CvImagePtr blurImage;
	try
	{
		originalMapImage = cv_bridge::toCvCopy(message, sensor_msgs::image_encodings::MONO8);
		threshImage = cv_bridge::toCvCopy(message, sensor_msgs::image_encodings::MONO8); // TODO: initialize as empty
		blurImage = cv_bridge::toCvCopy(message, sensor_msgs::image_encodings::MONO8); // TODO: initialize as empty
	}
	catch (cv_bridge::Exception& e)
	{
		cout << "cv_bridge exception: " << e.what() << endl;
		return;
	}

	// perform threshold
	threshold(originalMapImage->image, threshImage->image, obstacleIntensityThresholdValue, 255, THRESH_BINARY);

	// perform gaussian blur to expand edges and add some buffer
	int MAX_KERNEL_LENGTH = 13;
	for ( int i = 1; i < MAX_KERNEL_LENGTH; i = i + 2 )
        { 
		GaussianBlur(threshImage->image, blurImage->image, Size( i, i ), 0, 0);          
	}

	// perform threshold again with balloon buffer
	threshold(blurImage->image, thresholdImage, obstacleIntensityBalloonValue, 255, THRESH_BINARY);

}



