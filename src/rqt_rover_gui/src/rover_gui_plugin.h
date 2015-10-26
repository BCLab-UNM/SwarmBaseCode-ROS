#ifndef rqt_rover_gui__rover_gui_plugin_H
#define rqt_rover_gui__rover_gui_plugin_H

#include <rqt_gui_cpp/plugin.h>
#include <ui_rover_gui_plugin.h>
//#include <rqt_rover_gui/ui_rover_gui_plugin.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/Image.h>
#include <ros/macros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/Imu.h>
#include <pluginlib/class_list_macros.h>

//ROS msg types
#include "rover_onboard_target_detection/ATag.h"
#include "rover_onboard_target_detection/harvest.h"

#include <QWidget>
#include <QTimer>
#include <QLabel>

namespace rqt_rover_gui {

  class RoverGUIPlugin 
    : public rqt_gui_cpp::Plugin
  {
    Q_OBJECT
      
  public:
    RoverGUIPlugin();
    virtual void initPlugin(qt_gui_cpp::PluginContext& context);
    virtual void shutdownPlugin();
    virtual void saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const;
    virtual void restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings);
    
    // Handles output from the joystick node
    void joyEventHandler(const sensor_msgs::Joy::ConstPtr& joy_msg);
    void cameraEventHandler(const sensor_msgs::ImageConstPtr& image);
    void EKFEventHandler(const nav_msgs::Odometry::ConstPtr& msg);
    void GPSEventHandler(const nav_msgs::Odometry::ConstPtr& msg);
    void encoderEventHandler(const nav_msgs::Odometry::ConstPtr& msg);

    void centerUSEventHandler(const sensor_msgs::Range::ConstPtr& msg);
    void leftUSEventHandler(const sensor_msgs::Range::ConstPtr& msg);
    void rightUSEventHandler(const sensor_msgs::Range::ConstPtr& msg);
    void IMUEventHandler(const sensor_msgs::Imu::ConstPtr& msg);

    void targetDetectedEventHandler( rover_onboard_target_detection::ATag tagInfo ); //rover_onboard_target_detection::ATag msg );

    void setupSubscribers();
    void setupPublishers();

    // Detect rovers that are broadcasting information
    set<string> findConnectedRovers();

    // Display log message to the text frame in the GUI
    void displayLogMessage(QString msg);

  private slots:
    void currentRoverChangedEventHandler(QListWidgetItem *current, QListWidgetItem *previous);
    void pollRoversTimerEventHandler();

    void GPSCheckboxToggledEventHandler(bool checked);

    void EKFCheckboxToggledEventHandler(bool checked);

    void encoderCheckboxToggledEventHandler(bool checked);

  private:

    ros::Publisher manual_mode_publisher;
    ros::Publisher joystick_publisher;

    ros::Subscriber joystick_subscriber;
    ros::Subscriber encoder_subscriber;
    ros::Subscriber gps_subscriber;
    ros::Subscriber ekf_subscriber;
    ros::Subscriber us_center_subscriber;
    ros::Subscriber us_left_subscriber;
    ros::Subscriber us_right_subscriber;
    ros::Subscriber imu_subscriber;
    ros::Subscriber target_detection_subscriber;

    image_transport::Subscriber camera_subscriber;

    string selected_rover_name;
    set<string> rover_names;
    ros::NodeHandle nh;
    QWidget* widget;
    Ui::RoverGUI ui;

    QString log_messages;

  };
} // end namespace

#endif // end ifndef
