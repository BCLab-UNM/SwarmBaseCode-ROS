#ifndef rover_driver_rqt_ui__RoverMcp_H
#define rover_driver_rqt_ui__RoverMcp_H

#include <rqt_gui_cpp/plugin.h>
#include <ui_rover_mcp.h>

#include <iostream>
#include <sstream>

#include <ros/ros.h>
#include <ros/master.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt64.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
//#include <std_msgs/Bool.h>
//#include <std_msgs/Int16.h>
//#include <geometry_msgs/Pose2D.h>
//#include <geometry_msgs/Vector3.h>
//#include <sensor_msgs/Joy.h>
#include <pluginlib/class_list_macros.h>

#include <QString>
#include <QWidget>
#include <QTime>
#include <QTimer>



namespace rover_driver_rqt_mcp {

class RoverMcp
        : public rqt_gui_cpp::Plugin
{

    Q_OBJECT

public:

    RoverMcp();

    virtual void initPlugin(qt_gui_cpp::PluginContext& context);
    virtual void shutdownPlugin();
    virtual void saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const;
    virtual void restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings);

private slots:

    void manualMode();
    void autoMode();
    void manualSimMode();
    void autoSimMode();
    void publishParameters();
    void publishName();
    void publishVel();
    void harvestDataHandler(const std_msgs::UInt64 message);  // for targets harvested per rover
    void stateMachineDataHandler(const std_msgs::String message);  // for rover's state machine string


private:
    ros::NodeHandle mcpNode;
    ros::Publisher modePublish;
    ros::Publisher parameterPublish;
    ros::Publisher linearPublish;
    ros::Publisher angularPublish;
//    ros::Subscriber statusSubscribe;
//    void haltUpdate(const std_msgs::Bool::ConstPtr& msg);
    ros::Subscriber harvestedSubscriber;  // for targets harvested per rover
    ros::Subscriber stateMachineSubscriber;  // for rover's state machine string


    float pheromone;
    float uninformed;
    float informed;
    float travel;
    float search;
    float laying;
    float leave;
    float charge;
    float site;
    float following;
    std_msgs::Float32 linearSpd;
    std_msgs::Float32 angularSpd;

    std_msgs::UInt8 operationMode;
    std_msgs::Float32MultiArray parameterArray;

    Ui::RoverMCP ui;
    QWidget* widget;

};

}
#endif
