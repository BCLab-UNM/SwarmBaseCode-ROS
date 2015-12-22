#include <ros/ros.h>

//ROS libraries
#include <image_transport/image_transport.h>

//ROS messages
#include <std_msgs/Int16.h>
#include <sensor_msgs/image_encodings.h>

//OpenCV headers
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

//AprilTag headers
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

using namespace std;

//AprilTag objects
apriltag_family_t *tf = NULL; //tag family
apriltag_detector_t *td = NULL; //tag detector

//Image container
image_u8_t *u8_image = NULL;

//Image converter
image_u8_t *copy_image_data_into_u8_container(int width, int height, uint8_t *rgb, int stride);

//Publishers
ros::Publisher tagPublish;

//Callback handlers
void targetDetect(const sensor_msgs::ImageConstPtr& rawImage);

int main(int argc, char* argv[]) {
    //Get hostname
    char host[128];
    gethostname(host, sizeof (host));
    string hostname(host);
    string publishedName;

    tf = tag36h11_create();
    td = apriltag_detector_create();
    apriltag_detector_add_family(td, tf);

    //Allocate memory up front so it doesn't need to be done for every image frame
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

    image_transport::ImageTransport it(tNH);
    image_transport::Subscriber imgSubscribe = it.subscribe((publishedName + "/camera/image"), 2, targetDetect);

    tagPublish = tNH.advertise<std_msgs::Int16>((publishedName + "/targets"), 2, true);

    ros::spin();

    apriltag_detector_destroy(td);
    tag36h11_destroy(tf);

    return EXIT_SUCCESS;
}

void targetDetect(const sensor_msgs::ImageConstPtr& rawImage) {

    cv_bridge::CvImagePtr cvImage;
    std_msgs::Int16 tagDetected;

	//Convert from MONO8 to BGR8
	//TODO: consider whether we should let the camera publish as BGR8 and skip this conversion
    try {
        cvImage = cv_bridge::toCvCopy(rawImage); //, sensor_msgs::image_encodings::MONO8);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", rawImage->encoding.c_str());
        return;
    }

    //Create Mat image for processing
    cv::Mat matImage = cvImage->image;
    cv::cvtColor(matImage, matImage, cv::COLOR_BGR2GRAY);

    //Force greyscale and force image size.  This is only for Gazebo.
    //TODO: fix model so Gazebo publishes the correct format
    //TODO: if Mat is only used here, why not use the cvImage format here and skip the Mat image completely?
    if (matImage.cols != 320 && matImage.rows != 240) {
        cv::resize(matImage, matImage, cv::Size(320, 240), cv::INTER_LINEAR);
    }

    //Copy all image data into an array that AprilTag library expects
    image_u8_t *im = copy_image_data_into_u8_container(	matImage.cols, 
							matImage.rows, 
							(uint8_t *) matImage.data, 
							matImage.step);

    //Detect AprilTags
    zarray_t *detections = apriltag_detector_detect(td, im);
    
    //Check result for valid tag
    if (zarray_size(detections) > 0) {
	    apriltag_detection_t *det;
	    zarray_get(detections, 0, &det); //use the first tag detected in the image
	    tagDetected.data = det->id;
	
	    //Publish detected tag
	    tagPublish.publish(tagDetected);
	}
}

image_u8_t *copy_image_data_into_u8_container(int width, int height, uint8_t *rgb, int stride) {
    for (int y = 0; y < u8_image->height; y++) {
        for (int x = 0; x < u8_image->width; x++) {
            u8_image->buf[y * u8_image->stride + x] = rgb[y * stride + x + 0];
        }
    }
    return u8_image;
}




