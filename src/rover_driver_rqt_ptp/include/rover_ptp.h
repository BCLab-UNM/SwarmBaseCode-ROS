#ifndef rover_driver_rqt_ui__RoverPtp_H
#define rover_driver_rqt_ui__RoverPtp_H

#include <rqt_gui_cpp/plugin.h>
#include <ui_rover_ptp.h>

#include <iostream>

#include <ros/ros.h>
#include <ros/master.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt64.h>
#include <std_msgs/UInt8.h>
//#include <std_msgs/Float32MultiArray.h>
//#include <std_msgs/Bool.h>
//#include <std_msgs/Int16.h>
#include <geometry_msgs/Point.h>
//#include <geometry_msgs/Vector3.h>
//#include <sensor_msgs/Joy.h>
#include <pluginlib/class_list_macros.h>

#include <QString>
#include <QWidget>
#include <QTime>
#include <QTimer>

namespace rover_driver_rqt_ptp {

class RoverPtp
        : public rqt_gui_cpp::Plugin
{

    Q_OBJECT

public:

    RoverPtp();

    virtual void initPlugin(qt_gui_cpp::PluginContext& context);
    virtual void shutdownPlugin();
    virtual void saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const;
    virtual void restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings);

private slots:

//    void manualMode();
//    void autoMode();
//    void manualSimMode();
//    void autoSimMode();
    void publishPoint();
    void publishName();

private:

    ros::NodeHandle ptpNode;
    ros::Publisher ptpPublish;
    ros::Publisher pointPublish;
    ros::Publisher modePublish;
//    ros::Subscriber statusSubscribe;
//    void haltUpdate(const std_msgs::Bool::ConstPtr& msg);

//    float pheromone;
//    float uninformed;
//    float informed;
//    float travel;
//    float search;
//    float laying;
//    float leave;
//    float charge;
//    float site;

    geometry_msgs::Point navPoint;
    std_msgs::UInt8 operationMode;
//    std_msgs::Float32MultiArray parameterArray;

    Ui::RoverPTP ui;
    QWidget* widget;

};

}
#endif
