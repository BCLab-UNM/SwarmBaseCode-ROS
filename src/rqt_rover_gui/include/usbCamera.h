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
    cv::Mat cvImageLR;
    cv_bridge::CvImagePtr rosImage;

    ros::Timer timer;

    int camera_index;
    int fps;
    
};

#endif	/* USBCAMERA_H */

