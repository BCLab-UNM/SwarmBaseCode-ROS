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
 */

#ifndef USBCAMERA_H
#define	USBCAMERA_H

#include <string.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace std;

class USBCamera {
public:
    
    USBCamera(int frameRate, int cameraIndex, string hostname);
    virtual ~USBCamera();
    
    void capture(const ros::TimerEvent& te);
    
private:
    
    ros::NodeHandle nh;
    image_transport::ImageTransport it;
    image_transport::Publisher rawImgPublish;

    cv::VideoCapture videoStream;
    cv::Mat cvImageColor;
    cv::Mat cvImageBW;
    cv::Mat cvImageLR;
    cv_bridge::CvImagePtr rosImage;

    ros::Timer timer;

    int camera_index;
    int fps;
    
};

#endif	/* USBCAMERA_H */

