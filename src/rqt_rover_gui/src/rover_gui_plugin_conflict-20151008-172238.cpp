// Author: Matthew Fricke
// E-mail: matthew@fricke.co.uk
// Date: 9-16-205
// Purpose: implementation of a simple graphical front end for the UNM-NASA Swarmathon rovers.
// License: GPL3

#include <rover_gui_plugin.h>
#include <pluginlib/class_list_macros.h>

#include <QStringList>
#include <QLCDNumber>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt8.h>
#include <nav_msgs/Odometry.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv/cv.h>

using namespace std;

namespace rqt_rover_gui 
{
  RoverGUIPlugin::RoverGUIPlugin() : rqt_gui_cpp::Plugin(), widget(0)
  {
    setObjectName("RoverGUI");
  }

  void RoverGUIPlugin::initPlugin(qt_gui_cpp::PluginContext& context)
  {
    string rover_name = "moe";

    QStringList argv = context.argv();

    widget = new QWidget();

    ui.setupUi(widget);
    
    context.addWidget(widget);

    widget->setWindowTitle("Rover Interface");

    // Create a subscriber to listen for joystick events
    joystick_subscriber = nh.subscribe("/joy", 1000, &RoverGUIPlugin::joyEventHandler, this);

    // Create a subscriber to listen for camera events
    image_transport::ImageTransport it(nh);
    int frame_rate = 1;
    // Theroa codex results in the least information being transmitted
    camera_subscriber = it.subscribe("/"+rover_name+"/camera/image", frame_rate, &RoverGUIPlugin::cameraEventHandler, this, image_transport::TransportHints("theora"));

    // Odometry subscriber. Used to get the robots' estimated positions (as opposed to its true position).
    odometry_subscriber = nh.subscribe("/"+rover_name+"/odometry/fusion", 10,  &RoverGUIPlugin::odometryEventHandler, this);

    //ros::spin();

    // Set the robot to accept manual control. Latch so even if the robot connects later it will get the message.
    string manual_mode_topic = "/"+rover_name+"/mode";
    manual_mode_publisher = nh.advertise<std_msgs::UInt8>(manual_mode_topic, 10, true); // last argument sets latch to true

     std_msgs::UInt8 manual_mode_msg;
     manual_mode_msg.data = 1; // should be 1 or 0 here?
     manual_mode_publisher.publish(manual_mode_msg);

    string joystick_topic = "/"+rover_name+"/joystick";
    joystick_publisher = nh.advertise<sensor_msgs::Joy>(joystick_topic, 10, this);


    ROS_INFO_STREAM("Rover GUI started.");

    ROS_INFO_STREAM("Node name is " + ros::this_node::getName() + " and rover name is " + rover_name + ".");

    string rover_name_msg = "<font color='white'>Rover: " + rover_name + "</font>";
    QString rover_name_msg_qstr = QString::fromStdString(rover_name_msg);
    ui.rover_name->setText(rover_name_msg_qstr);



    // Rover map test

    //ui.map_frame->addToRoverPath(0,0);
    //ui.map_frame->addToRoverPath(1.0,1.0);

    //ui.map_frame->addCollectionPoint(0,0);

    for (int i = 0; i < 25; i++) ui.map_frame->addTargetLocation(rand()*1.0/RAND_MAX, rand()*1.0/RAND_MAX);
    for (int i = 0; i < 5; i++) ui.map_frame->addCollectionPoint(rand()*1.0/RAND_MAX, rand()*1.0/RAND_MAX);
    for (int i = 0; i < 15; i++) ui.map_frame->addToRoverPath(rand()*1.0/RAND_MAX, rand()*1.0/RAND_MAX);


    //ui.map_frame->addTargetLocation(0.75, 0.75);
    //ui.map_frame->addTargetLocation(1.0, 1.0);

  }

  void RoverGUIPlugin::shutdownPlugin()
  {
    //ros::shutdown();
  }

void RoverGUIPlugin::saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const
{
}

void RoverGUIPlugin::restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings)
{
}


void RoverGUIPlugin::joyEventHandler(const sensor_msgs::Joy::ConstPtr& joy_msg)
{
    // Set the gui values
    if (joy_msg->axes[4] > 0)
       ui.joy_lcd_forward->display(joy_msg->axes[4]);
    if (joy_msg->axes[4] < 0)
        ui.joy_lcd_back->display(-joy_msg->axes[4]);

    if (joy_msg->axes[3] > 0)
      {
       ui.joy_lcd_left->display(joy_msg->axes[3]);
      }
    if (joy_msg->axes[3] < 0)
      {
        ui.joy_lcd_right->display(-joy_msg->axes[3]);
      }




// Magic axis values in the code below were taken the rover_driver_rqt_motor code /joystick output for default linear and angular velocities.
// Magic indicies are taken from rover_motor.cpp.
// This way the code is consistent with the existing GUI joystick.
// A better way would be to standardize a manual movement control interface and requre all input mechanisms to take input from the user
// and repackage te information according to the interface spec.
    sensor_msgs::Joy standardized_joy_msg;
    standardized_joy_msg.axes.resize(6);

    int x_axis = 0;
    int y_axis = 1;

    if (abs(joy_msg->axes[4]) > 0.05)
    {
      standardized_joy_msg.axes[y_axis] = joy_msg->axes[4];
    }

  if (abs(joy_msg->axes[3]) > 0.05)
    {
      standardized_joy_msg.axes[x_axis] = joy_msg->axes[3];
    }

  joystick_publisher.publish(standardized_joy_msg);

}

 void RoverGUIPlugin::cameraEventHandler(const sensor_msgs::ImageConstPtr& image)
 {
     cv_bridge::CvImagePtr cv_image_ptr;

     try
     {
        cv_image_ptr = cv_bridge::toCvCopy(image);
     }
     catch (cv_bridge::Exception &e)
     {
         ROS_ERROR("In rover_gui_plugin.cpp: cv_bridge exception: %s", e.what());
     }

     int image_cols = cv_image_ptr->image.cols;
     int image_rows = cv_image_ptr->image.rows;
     int image_step = cv_image_ptr->image.step;

     ostringstream cols_stream, rows_stream;
     cols_stream << image_cols;
     rows_stream << image_rows;

    // ROS_INFO_STREAM("Image received Size:" + rows_stream.str() + "x" + cols_stream.str());

     // Create QImage to hold the image
     //const uchar* image_buffer = (const uchar*)cv_image_ptr->image.data; // extract the raw data
     QImage qimg(&(image->data[0]), image_cols, image_rows, image_step, QImage::Format_RGB888);
     ui.camera_frame->setImage(qimg);
 }

void RoverGUIPlugin::odometryEventHandler( const nav_msgs::Odometry::ConstPtr& msg )
{
    float x = msg->pose.pose.position.x;
    float y = msg->pose.pose.position.y;

    ui.map_frame->addToRoverPath(x,y);
}

} // End namespace

PLUGINLIB_EXPORT_CLASS(rqt_rover_gui::RoverGUIPlugin, rqt_gui_cpp::Plugin)
