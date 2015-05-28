#ifndef rover_driver_rqt_ui__RoverMotor_H
#define rover_driver_rqt_ui__RoverMotor_H

#include <rqt_gui_cpp/plugin.h>
#include <ui_rover_motor.h>

#include <iostream>

#include <ros/ros.h>
#include <ros/master.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt64.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int16.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/Joy.h>
#include <pluginlib/class_list_macros.h>

#include <QString>
#include <QWidget>
#include <QTime>
#include <QTimer>

namespace rover_driver_rqt_motor {

class RoverMotor
        : public rqt_gui_cpp::Plugin
{

    Q_OBJECT

public:

    RoverMotor();

    virtual void initPlugin(qt_gui_cpp::PluginContext& context);
    virtual void shutdownPlugin();
    virtual void saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const;
    virtual void restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings);

private slots:

    void moveForward();
    void moveBackward();
    void turnRight();
    void turnLeft();
    void allStop();
    void hbUpdate();
    void publishName();

private:

    ros::NodeHandle teleopNode;
    //ros::Publisher cmdPublish;
    ros::Publisher hbPublish;
    ros::Publisher stickPublish;
    ros::Publisher whichPublish;
    ros::Publisher steerPublish;

    ros::Subscriber statusSubscribe;
    ros::Subscriber haltSubscribe;

    void statusUpdate(const std_msgs::String::ConstPtr& msg);
    void haltUpdate(const std_msgs::Bool::ConstPtr& msg);

    QTimer *timer;

    Ui::RoverMotor ui;
    QWidget* widget;

};

}
#endif // rover_driver_rqt_motor__HarveTeleop_H
