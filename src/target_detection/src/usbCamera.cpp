#include "usbCamera.h"

enum Default {
        DEFAULT_CAMERA_INDEX = 0,
        DEFAULT_FPS = 1
    };

USBCamera::USBCamera(int frameRate, int cameraIndex, string hostname):
    it(nh),
    videoStream(cameraIndex) {
    
        nh.param<int>("cameraIndex", cameraIndex, DEFAULT_CAMERA_INDEX);
        nh.param<int>("fps", fps, frameRate);
        
        if (not videoStream.isOpened()) {
            ROS_ERROR_STREAM("Failed to open camera device!");
            ros::shutdown();
        }
        
        ros::Duration period = ros::Duration(1. / fps);

        rawImgPublish = it.advertise((hostname + "/camera/image"), 2);

        rosImage = boost::make_shared<cv_bridge::CvImage>();
        rosImage->encoding = sensor_msgs::image_encodings::BGR8;

        timer = nh.createTimer(period, &USBCamera::capture, this);
}

void USBCamera::capture(const ros::TimerEvent& te) {
        videoStream >> cvImageColor;
        
        cv::resize(cvImageColor, cvImageLR, cv::Size(320,240), cv::INTER_LINEAR);
        
        rosImage->image = cvImageLR;
        if (not rosImage->image.empty()) {
            rosImage->header.stamp = ros::Time::now();
            rawImgPublish.publish(rosImage->toImageMsg());
        }
    }

USBCamera::~USBCamera(){
    videoStream.release();
}

