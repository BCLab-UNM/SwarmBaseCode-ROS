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

void mapHandler(const sensor_msgs::Image::ConstPtr& message);
bool getPlan(nav_msgs::GetPlan::Request &req, nav_msgs::GetPlan::Response &res);

char host[128];

// TODO: Map size should not be hard coded.  Read it from the actual published map message.
const int MAP_SIZE = 300;
const int MAP_ORIGIN_X = MAP_SIZE / 2;
const int MAP_ORIGIN_Y = MAP_SIZE / 2;

#define PI 3.14159265

// TODO: Make this part of the parameter server and read it every service call
int obstacleIntensityThresholdValue = 200; // 0 to 255, below which is not considered an obstacle to avoid in the path
int obstacleIntensityBalloonValue = 10; // 0 to 100, lower number is bigger ballooning
int angleToIncrement = 5;
int chordToLengthen = 4;

// TODO: if we're stuck, we can try to reduce the obstacle threshold and try again before giving up

// TODO: consider whether these should be global or not
Mat thresholdImage(300, 300, CV_8UC1);
cv_bridge::CvImagePtr originalMapImage;

// TODO: I think these can be local
cv_bridge::CvImagePtr pathCreationImage;
cv_bridge::CvImagePtr finalPathImage;

ros::Publisher pathCreationImagePublisher;
ros::Publisher finalPathImagePublisher;

/*
 * This routine just initializes stuff, mostly.
 */
int main(int argc, char** argv) {

	gethostname(host, sizeof (host));
	string hostname(host);
	string publishedName;

	// initialize ROS
	ros::init(argc, argv, (hostname + "_PATH"));
	ros::NodeHandle pNH;

	// handle arguments
	if (argc >= 2) {
		publishedName = argv[1];
		cout << "Welcome to the world of tomorrow " << publishedName << "! Path planning module started." << endl;
	} else {
		publishedName = hostname;
		cout << "No name selected. Default is: " << publishedName << endl;
	}

	// set up subscriber
	ros::Subscriber mapSubscriber;
	mapSubscriber = pNH.subscribe((publishedName + "/map"), 1, mapHandler);

	// set up publishers
	pathCreationImagePublisher = pNH.advertise<sensor_msgs::Image>((publishedName + "/pathalgorithm"), 1, true);
	finalPathImagePublisher = pNH.advertise<sensor_msgs::Image>((publishedName + "/mappluspath"), 1, true);

	// set up service
	ros::ServiceServer service = pNH.advertiseService("getPlan", getPlan);

	// set up opencv bridge
	image_transport::ImageTransport imgTransport(pNH);
	pathCreationImage = boost::make_shared<cv_bridge::CvImage>();
	pathCreationImage->encoding = sensor_msgs::image_encodings::BGR8;
	finalPathImage = boost::make_shared<cv_bridge::CvImage>();
	finalPathImage->encoding = sensor_msgs::image_encodings::BGR8;

	// spin pumping callbacks
	ros::spin();

	return 0;
}


/*
 * This routine processes the map, which can be published at any time.
 * When the service is called, it uses a copy of the map that is in place at that time.
 */
void mapHandler(const sensor_msgs::Image::ConstPtr& message) {
	cv_bridge::CvImagePtr threshImage;
	cv_bridge::CvImagePtr blurImage;

	// make copy of image in message into local variables
	try
	{
		originalMapImage = cv_bridge::toCvCopy(message, sensor_msgs::image_encodings::MONO8);
		// TODO: these two should just be created as empty images
		threshImage = cv_bridge::toCvCopy(message, sensor_msgs::image_encodings::MONO8); 
		blurImage = cv_bridge::toCvCopy(message, sensor_msgs::image_encodings::MONO8); 
	}
	catch (cv_bridge::Exception& e)
	{
		cout << "cv_bridge exception: " << e.what() << endl;
		return;
	}

	// perform threshold on original map image
	threshold(originalMapImage->image, threshImage->image, obstacleIntensityThresholdValue, 255, THRESH_BINARY);

	// perform gaussian blur to expand edges and add some buffer
	int MAX_KERNEL_LENGTH = 13;
	for ( int i = 1; i < MAX_KERNEL_LENGTH; i = i + 2 )
        { 
		GaussianBlur(threshImage->image, blurImage->image, Size( i, i ), 0, 0);          
	}

	// perform threshold again with balloon buffer
	threshold(blurImage->image, thresholdImage, obstacleIntensityBalloonValue, 255, THRESH_BINARY);

	// TODO: Remove global map that this routine is creating and that the getPlan routine is using.  
	// Create a copy just for the service handler.  Maybe use a getter function.

}


/*
 * This routine is called when a service arrives.  This routine does most of the heavy lifting.
 */
bool getPlan(	nav_msgs::GetPlan::Request  &req,
		nav_msgs::GetPlan::Response &res	)
{
	geometry_msgs::Point startUser;
	geometry_msgs::Point goalUser;

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
	Point globalStartPointImageCoord, globalGoalPointImageCoord;
	globalStartPointImageCoord.x = startUser.y + (MAP_SIZE/2);
	globalStartPointImageCoord.y = startUser.x + (MAP_SIZE/2);
	globalGoalPointImageCoord.x = goalUser.y + (MAP_SIZE/2);
	globalGoalPointImageCoord.y = goalUser.x + (MAP_SIZE/2);

	// set up some other variables
	Point localStartPointImageCoord = globalStartPointImageCoord;
	Point localGoalPointImageCoord = globalGoalPointImageCoord;
	int totalPoints = 1;
	bool avertingObstacle = false;
	bool directionClockwise = true;
	#define UNSET -99999
	Point clearPointCW, clearPointCCW;
	clearPointCW.x = UNSET;
	clearPointCW.y = UNSET;
	clearPointCCW.x = UNSET;
	clearPointCCW.y = UNSET;
	Point boundaryEncounteredPoint = globalGoalPointImageCoord;

	// add this goal point to Path array, adjust for image coordinate system center point
	res.plan.poses.resize(totalPoints);
	res.plan.poses[totalPoints-1].pose.position.x = localStartPointImageCoord.x - (MAP_SIZE/2); 
	res.plan.poses[totalPoints-1].pose.position.y = localStartPointImageCoord.y - (MAP_SIZE/2); 

	bool globalGoalReached = false;
	int iterations = 0;
	while(!globalGoalReached) {
	//for (int j = 0; j < 20; j++) {

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
		//cout << "Bearing to local goal: " << currentBearing << endl;

		// iterate a line between local start and local goal
		LineIterator myIterator(thresholdImage, 
				localStartPointImageCoord, 
				localGoalPointImageCoord, 
				8);
		vector<Point> points(myIterator.count);
		int lineIterCount;
		//cout << "Intensity: " << endl;
		for(lineIterCount = 0; lineIterCount < myIterator.count; lineIterCount++, ++myIterator)
		{
			points[lineIterCount] = myIterator.pos();
			// get pixel intensity at this point
			int intensity = thresholdImage.at<uchar>(points[lineIterCount]);
			//cout << intensity << " (" << myIterator.pos().x << ", " << myIterator.pos().y << ") " ;

			// look for boundary
			if(intensity > 127) {
				//cout << endl;
				//cout << "Boundary was found at: " << myIterator.pos().x << " " << 
				//			myIterator.pos().y << endl;
				foundBoundary = true;

				// save only the initial obstacle encounter point for use later in CCW testing
				if (!avertingObstacle) {
					boundaryEncounteredPoint = myIterator.pos();
					//cout << "Saved encounter point for use later in CCW testing. " << endl;
				}


				// draw this failed try in a lighter color, previous iteration's point, though
				line(pathCreationImage->image, 
					localStartPointImageCoord, 
					points[lineIterCount-1], // TODO: this is not crash proof.  Assumes line is longer than one pixel!
					Scalar(20, 20, 230), 1, 4);

				break;
			}

			// look for local goal met
			if(myIterator.count == lineIterCount + 1) {
				//cout << endl;
				//cout << "Local goal was met. " << endl;
				metLocalGoal = true;
				break;
			}

		} // end line iterator

		// handle boundary by changing local goal and let it try again (don't paint line)
		if(foundBoundary) {
			avertingObstacle = true;

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
			//cout << "short chord length: " << shortCordLength << endl;
			// lock in a minimum chord length
			// TODO: This may need to start small, and get big as we iterate a bunch to get out of trouble?
			if (shortCordLength < 20) {
				shortCordLength = 20;
			}

			// check for direction to test
			int invertDirection = 1;
			if (!directionClockwise) {
				invertDirection = -1;
			}

			// TODO: If we get stuck in a trap, we might need to stop swapping directions and just follow the wall out

			// change the bearing to search for a clearing in a slightly new direction
			float newBearing = calcBearing + (angleToIncrement * invertDirection);
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
			//cout << "test bearing: " << testBearing << endl;

			// calculate new temporary goal coordinates, add a little length to short chord
			localGoalPointImageCoord.x = localStartPointImageCoord.x + 
							( ( (int)shortCordLength + chordToLengthen) * 
								cos(newBearing * PI / 180) );
			localGoalPointImageCoord.y = localStartPointImageCoord.y + 
							( ( (int)shortCordLength + chordToLengthen) * 
								sin(newBearing * PI / 180) );
			//cout << "new temporary goal: " << localGoalPointImageCoord.x << " " << 
			//				localGoalPointImageCoord.y << endl;
		}

		// handle local goal found
		if(metLocalGoal) {

			// paint the line for this leg on the path creation image, previous iteration's point, though
			line(pathCreationImage->image, 
				localStartPointImageCoord, 
				//localGoalPointImageCoord, 
				points[lineIterCount-1], // TODO: this is not crash proof.  Assumes line is longer than one pixel!
				Scalar(230, 20, 20), 1, 4);

			// set the appropriate clear point variable
			if (avertingObstacle) {
				if (directionClockwise) {
					// set clockwise clear point
					clearPointCW.x = localGoalPointImageCoord.x;
					clearPointCW.y = localGoalPointImageCoord.y;
					//cout << "Clockwise testing is complete." << endl;

					// reset goal to one where boundary was originally encountered, and start CCW testing
					localGoalPointImageCoord = boundaryEncounteredPoint;
					directionClockwise = false;
					continue;
				}
				else {
					// set counterclockwise clear point
					clearPointCCW.x = localGoalPointImageCoord.x;
					clearPointCCW.y = localGoalPointImageCoord.y;
					//cout << "Counterclockwise testing is complete." << endl;

					// both directions have been tried.  calculate both chord lengths.
					int xDiff = clearPointCW.x - globalGoalPointImageCoord.x;
					int yDiff = clearPointCW.y - globalGoalPointImageCoord.y;
					float cwCordLength = sqrt( (xDiff * xDiff) + (yDiff * yDiff) );
					//cout << "CW chord length: " << cwCordLength << endl;
					xDiff = clearPointCCW.x - globalGoalPointImageCoord.x;
					yDiff = clearPointCCW.y - globalGoalPointImageCoord.y;
					float ccwCordLength = sqrt( (xDiff * xDiff) + (yDiff * yDiff) );
					//cout << "CCW chord length: " << ccwCordLength << endl;

					// shortest straight line distance to goal wins
					// although this is not necessarily the most efficient route
					if (cwCordLength <= ccwCordLength) {
						// CW wins
						localGoalPointImageCoord = clearPointCW;
						//cout << "CW wins!" << endl;
					}
					else {
						// CCW wins
						localGoalPointImageCoord = clearPointCCW;
						//cout << "CCW wins!" << endl;
					}

					// reset and return to normal path planning
					directionClockwise = true;
					avertingObstacle = false;
				}
			}

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

			avertingObstacle = false;

			// if also is global goal, then we're done
			if(localGoalPointImageCoord == globalGoalPointImageCoord) {
				cout << "Goal was met using " << totalPoints << " total points. " << endl;
				globalGoalReached = true;
			}
			else {
				// we met local goal but not global goal yet
				// we need to set the local goal to the global goal and try again
				// we also need to set a new start point
				localStartPointImageCoord = localGoalPointImageCoord;
				localGoalPointImageCoord = globalGoalPointImageCoord;
			}

		}

		// bomb out if things go badly
		// TODO: iterations before giving up should probably be in the parameter server to change at runtime
		if (iterations > 1000) {
			cout << "Oops.  The algorithm is likely stuck.  Quitting now. Sorry." << endl;
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



