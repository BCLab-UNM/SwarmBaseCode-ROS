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
 */

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <vector>

#include "rover_onboard_target_detection/ATag.h"

#include "AprilTags/TagDetector.h"
#include "AprilTags/Tag16h5.h"
#include "AprilTags/Tag25h7.h"
#include "AprilTags/Tag25h9.h"
#include "AprilTags/Tag36h9.h"
#include "AprilTags/Tag36h11.h"

using namespace std;

rover_onboard_target_detection::ATag tagInfo;
AprilTags::TagDetector* tagDetector;
ros::Publisher tagPublish;

std::vector <int> foundTagList;

void targetDetect(const sensor_msgs::ImageConstPtr& image);

int main(int argc, char* argv[]) {

    char host[128];
    gethostname(host, sizeof (host));
    string hostname(host);
    string publishedName;

    ros::init(argc, argv, (hostname + "_TARGET"));

    ros::NodeHandle tNH;

    if (argc >= 2) {
        publishedName = argv[1];
        cout << "Welcome to the world of tomorrow " << publishedName << "!  Target detect module started." << endl;
    } else {
        publishedName = hostname;
        cout << "No Name Selected. Default is: " << publishedName << endl;
    }

    image_transport::ImageTransport it(tNH);
    image_transport::Subscriber imgSubscribe = it.subscribe((publishedName + "/camera/image"), 1, targetDetect);

    tagPublish = tNH.advertise<rover_onboard_target_detection::ATag>((publishedName + "/targets"), 10);

    tagDetector = new AprilTags::TagDetector(AprilTags::tagCodes36h11);

    while (ros::ok()) {
        ros::spin();
    }

    return EXIT_SUCCESS;
}

void targetDetect(const sensor_msgs::ImageConstPtr& image) {

    cv_bridge::CvImagePtr imagePtr;
    int newTagCount = 0;

    try {
        imagePtr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::MONO8);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", image->encoding.c_str());
    }

    vector<AprilTags::TagDetection> detections = tagDetector->extractTags(imagePtr->image);

    for (int i = 0; i < detections.size(); i++) {
        int thisTag = detections.data()[i].id;

        if(find(foundTagList.begin(), foundTagList.end(), thisTag) == foundTagList.end()) {
            //this tag is new
            newTagCount++;
            tagInfo.tagID.push_back(thisTag); //add to published data
            foundTagList.push_back(thisTag); //add to master list of found tags
        }
    }

    tagInfo.tagsFound = newTagCount;
    tagPublish.publish(tagInfo);
    tagInfo.tagID.clear();
}

