#include <cstdlib>
#include <iostream>
#include <vector>
/*
 * Author: Karl A. Stolleis
 * Maintainer: Karl A. Stolleis
 * Email: karl.a.stolleis@nasa.gov; kurt.leucht@nasa.gov
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
 * 08/25/2014 - MWN: Implemented master tag list, so only new tags are registered
 *      
 */
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

/*
#include "AprilTags/TagDetector.h"
#include "AprilTags/Tag16h5.h"
#include "AprilTags/Tag25h7.h"
#include "AprilTags/Tag25h9.h"
#include "AprilTags/Tag36h9.h"
#include "AprilTags/Tag36h11.h"*/

using namespace std;

rover_onboard_target_detection::ATag tagInfo;
rover_onboard_target_detection::ATag foundTagList;


//AprilTags::TagDetector* tagDetector;
ros::Publisher tagPublish;
ros::Subscriber foundTagListSubscriber;

//Declaration of tag family and detector for C
apriltag_family_t *tf = NULL;
apriltag_detector_t *td = NULL;




void targetDetect(const sensor_msgs::ImageConstPtr& image);
void foundTagListHandler(const rover_onboard_target_detection::ATag message);
image_u8_t *image_u8_create_from_rgb3(int width, int height, uint8_t *rgb, int stride);

int main(int argc, char* argv[]) {
    //Get hostname
    char host[128];
    gethostname(host, sizeof (host));
    string hostname(host);
    string publishedName;
    string worldName = "world";

    //Create tag family and detector
    tf = tag36h11_create();
    td = apriltag_detector_create();
    apriltag_detector_add_family(td, tf);

    if (argc >= 2) {
        publishedName = argv[1];
        cout << "Welcome to the world of tomorrow " << publishedName << "!  Target detect module started." << endl;
    } else {
        publishedName = hostname;
        cout << "No Name Selected. Default is: " << publishedName << endl;
    }
    ros::init(argc, argv, (publishedName + "_TARGET"));
    ros::NodeHandle tNH;

    image_transport::ImageTransport it(tNH);
    image_transport::Subscriber imgSubscribe = it.subscribe((publishedName + "/camera/image"), 1, targetDetect);

    foundTagListSubscriber = tNH.subscribe((worldName + "/foundTagList"), 10, foundTagListHandler);

    tagPublish = tNH.advertise<rover_onboard_target_detection::ATag>((publishedName + "/targets"), 10);

    //tagDetector = new AprilTags::TagDetector(AprilTags::tagCodes36h11);
    while (ros::ok()) {
        ros::spin();
    }
    apriltag_detector_destroy(td);
    tag36h11_destroy(tf);

    return EXIT_SUCCESS;
}

void targetDetect(const sensor_msgs::ImageConstPtr& image) {

    cv_bridge::CvImagePtr imagePtr;
    int newTagCount = 0;

    try {
        imagePtr = cv_bridge::toCvCopy(image); //, sensor_msgs::image_encodings::MONO8);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", image->encoding.c_str());
    }

    //Create apriltag image data structure
    cv::Mat apriltagImage = imagePtr->image;

    //Gazebo does not use the resized image. This checks to make sure the image going into the tag detector is the correct size and also grayscale
    if (apriltagImage.cols != 320 && apriltagImage.rows != 240) {
        cv::cvtColor(apriltagImage, apriltagImage, cv::COLOR_BGR2GRAY);
        cv::resize(apriltagImage, apriltagImage, cv::Size(320, 240), cv::INTER_LINEAR);
    }


    //cv::imwrite("/home/gmontague/DEBUG/apriltag.png",apriltagImage);
    //cout << apriltagImage.cols << "," << apriltagImage.rows << "," << apriltagImage.step << endl;
    image_u8_t *im = image_u8_create_from_rgb3(apriltagImage.cols, apriltagImage.rows, (uint8_t *) apriltagImage.data, apriltagImage.step);
    //vector<AprilTags::TagDetection> detections = tagDetector->extractTags(imagePtr->image);
    //string pathString = "/home/gmontague/DEBUG/debug.pnm";
    //const char *path = pathString.c_str();
    //image_u8_write_pnm(im, path); //write u8 image to disk to inspect data type 
    //Detect AprilTags using C detector
    zarray_t *detections = apriltag_detector_detect(td, im);

    //cout << zarray_size(detections) << endl;
    for (int i = 0; i < zarray_size(detections); i++) {

        apriltag_detection_t *det;
        zarray_get(detections, i, &det);

        int thisTag = det->id;
        //int thisTag = detections.data()[i].id;

        if (find(foundTagList.tagID.begin(), foundTagList.tagID.end(), thisTag) == foundTagList.tagID.end()) {
            //detected tag is not on the foundTagList, publish it as detected
            newTagCount++;
            tagInfo.tagID.push_back(thisTag); //add to published data
        }
    }

    tagInfo.tagsFound = newTagCount;
    tagPublish.publish(tagInfo);
    tagInfo.tagID.clear();
    image_u8_destroy(im);

}

void foundTagListHandler(const rover_onboard_target_detection::ATag message) {
    foundTagList = message;
}

image_u8_t *image_u8_create_from_rgb3(int width, int height, uint8_t *rgb, int stride) {

    image_u8_t *im = image_u8_create(width, height);
    for (int y = 0; y < im->height; y++) {
        for (int x = 0; x < im->width; x++) {
            // int r = rgb[y * stride + 3 * x + 0];
            // int g = rgb[y * stride + 3 * x + 0];
            // int b = rgb[y * stride + 3 * x + 0];
            //int gray = (int) (0.6 * g + 0.3 * r + 0.1 * b);
            //int gray = rgb[y * stride + x + 0];
            //if (gray > 255)
            //    gray = 255;
            im->buf[y * im->stride + x] = rgb[y * stride + x + 0];
        }
    }

    //for (int i = 0; )

    // const initializer
    //image_u8_t *tmp = { tmp->width = width, tmp->height = height, tmp->stride = stride, tmp->buf = rgb };
    //image_u8_t *tmp;
    //tmp->width = width;

    //image_u8_t tmp = { tmp->width = width, tmp->height = height, tmp->stride = stride, tmp->buf = buf };
    //size_t size = sizeof(image_u8_t);
    //image_u8_t *im = calloc(1, size);
    //memcpy(im, &tmp, size);
    return im;
}
/*
image_u8_t *image_u8_create_alignment(unsigned int width, unsigned int height, unsigned int alignment)
{
    int stride = width;

    if ((stride % alignment) != 0)
        stride += alignment - (stride % alignment);

    uint8_t *buf = calloc(height*stride, sizeof(uint8_t));

    // const initializer
    image_u8_t tmp = { tmp->width = width, tmp->height = height, tmpstride = stride, .buf = buf };

    image_u8_t *im = calloc(1, sizeof(image_u8_t));
    memcpy(im, &tmp, sizeof(image_u8_t));
    return im;
}*/
