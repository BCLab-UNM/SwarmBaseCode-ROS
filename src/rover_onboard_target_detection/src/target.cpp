#include <cstdlib>
#include <iostream>
#include <vector>

//C headers
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

//Apriltag C headers
#include "apriltag.h"
#include "image_u8.h"
#include "tag36h11.h"
#include "tag36h10.h"
#include "tag36artoolkit.h"
#include "tag25h9.h"
#include "tag25h7.h"
#include "image_u8.h"
#include "pnm.h"
#include "zarray.h"
#include "getopt.h"

//ROS Headers
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <vector>

//ROS msg types
#include "rover_onboard_target_detection/ATag.h"
#include "rover_onboard_target_detection/harvest.h"

using namespace std;

rover_onboard_target_detection::ATag tagInfo;
rover_onboard_target_detection::ATag foundTagList;

ros::Publisher tagPublish;
ros::Subscriber foundTagListSubscriber;

//Declaration of tag family and detector for C
apriltag_family_t *tf = NULL;
apriltag_detector_t *td = NULL;
image_u8_t *u8_image = NULL;

void targetDetect(const sensor_msgs::ImageConstPtr& rawImage);
void foundTagListHandler(const rover_onboard_target_detection::ATag message);
image_u8_t *copy_image_data_into_u8_container(int width, int height, uint8_t *rgb, int stride);

int main(int argc, char* argv[]) {
    //Get hostname
    char host[128];
    gethostname(host, sizeof (host));
    string hostname(host);
    string publishedName;
    string worldName = "world";

    //Create tag family and detector, these are calls to april tag library
    tf = tag36h11_create();
    td = apriltag_detector_create();
    apriltag_detector_add_family(td, tf);

    // allocate memory up front so it doesn't need to be done for every image frame
    u8_image = image_u8_create(320, 240);

    if (argc >= 2) {
        publishedName = argv[1];
        cout << "Welcome to the world of tomorrow " << publishedName << "!  Target detect module started." << endl;
    } else {
        publishedName = hostname;
        cout << "No Name Selected. Default is: " << publishedName << endl;
    }
    ros::init(argc, argv, (publishedName + "_TARGET"));
    ros::NodeHandle tNH;

    // subscribe to camera feed (slow, maybe 2 Hz)
    image_transport::ImageTransport it(tNH);
    image_transport::Subscriber imgSubscribe = it.subscribe((publishedName + "/camera/image"), 2, targetDetect);

    // subscribe to tag list feed (only published on change)
    foundTagListSubscriber = tNH.subscribe((worldName + "/foundTagList"), 1, foundTagListHandler);

    // set up topic to publish detected targets on
    tagPublish = tNH.advertise<rover_onboard_target_detection::ATag>((publishedName + "/targets"), 2, true);

    // spin forever, pumping callbacks    
    ros::spin();

    //TODO: These destructor calls may need to be moved into a signal handler, assuming rosspin never exits
    apriltag_detector_destroy(td);
    tag36h11_destroy(tf);

    return EXIT_SUCCESS;
}

void targetDetect(const sensor_msgs::ImageConstPtr& rawImage) {

    cv_bridge::CvImagePtr cvImage;

    // clear the target data type for publishing a new value for this image
    int tagsFoundInImage = 0;
    tagInfo.tagID.clear();

    try {
	// convert from MONO8 to BGR8
        // TODO: consider whether we should let the camera publish as BGR8 and skip this conversion
        cvImage = cv_bridge::toCvCopy(rawImage); //, sensor_msgs::image_encodings::MONO8);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", rawImage->encoding.c_str());
        return;
    }

    // create Mat image for processing
    cv::Mat matImage = cvImage->image;

    // force greyscale and force image size.  This is only for Gazebo.
    // TODO: fix model so Gazebo publishes the correct format
    // TODO: if Mat is only used here, why not use the cvImage format here and skip the Mat image completely?
    if (matImage.cols != 320 && matImage.rows != 240) {
        cv::cvtColor(matImage, matImage, cv::COLOR_BGR2GRAY);
        cv::resize(matImage, matImage, cv::Size(320, 240), cv::INTER_LINEAR);
    }

    // copy all image data into an array that april tag library expects
    image_u8_t *im = copy_image_data_into_u8_container(	matImage.cols, 
							matImage.rows, 
							(uint8_t *) matImage.data, 
							matImage.step);

    //Detect AprilTags using C detector
    zarray_t *detections = apriltag_detector_detect(td, im);

    // march through results and add to list if no other robot has found it
    for (int i = 0; i < zarray_size(detections); i++) {

        apriltag_detection_t *det;
        zarray_get(detections, i, &det);

        int thisTag = det->id;

        // TODO: consider letting target node report on tags already found, and let mobility or world node filter out the previously found ones.  that will alleviate target node from even subscribing to found tag list and doing that processing.  I think mobility already does that, so this is possibly redundant logic.
        if (find(foundTagList.tagID.begin(), foundTagList.tagID.end(), thisTag) == foundTagList.tagID.end()) {
            //detected tag is not on the foundTagList, publish it as detected
            tagsFoundInImage++;
            tagInfo.tagID.push_back(thisTag); //add to published data
        }
    }

    // publish how many tags were found in current image and their IDs
    tagInfo.tagsFound = tagsFoundInImage;
    tagPublish.publish(tagInfo);
    //tagInfo.tagID.clear();

    //image_u8_destroy(im);

}

void foundTagListHandler(const rover_onboard_target_detection::ATag message) {
    foundTagList = message;
}

image_u8_t *copy_image_data_into_u8_container(int width, int height, uint8_t *rgb, int stride) {

    //image_u8_t *im = image_u8_create(width, height);
    for (int y = 0; y < u8_image->height; y++) {
        for (int x = 0; x < u8_image->width; x++) {
            u8_image->buf[y * u8_image->stride + x] = rgb[y * stride + x + 0];
        }
    }

    return u8_image;
}




