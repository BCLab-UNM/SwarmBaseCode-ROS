/**
 * The main function for the behavior node
 */
#include <memory>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt8.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h> //arming cmd
#include <mavros_msgs/CommandTOL.h> //takeoff cmd
#include <mavros_msgs/OverrideRCIn.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>

#include <boost/algorithm/string.hpp>

#include "ROSTimer.hpp"

using namespace std;

int mode = 0;


mavros_msgs::SetMode JUAV_target_mode;
mavros_msgs::State current_state;
mavros_msgs::SetMode target_mode;

bool landed = true;
bool takeoff_attempted = false;

geometry_msgs::PoseWithCovariance juav_current_location;

ros::Publisher odom_publisher;

//***********************************************************************************************************************************************************************

void heartbeatHandler(ros::Publisher& heartbeatPublisher, const ros::TimerEvent& event)
{
   std_msgs::String msg;
   msg.data = "";
   heartbeatPublisher.publish(msg);
}

void statusHandler(ros::Publisher& statusPublisher, const ros::TimerEvent& event)
{
  std_msgs::String msg;
  if(current_state.connected == false)
  {
    msg.data = "JUAV_subsumption2";
  }
  else
  {
    msg.data = "" + current_state.mode;
    if (current_state.armed == true)
    {
      msg.data += " Armed"; 
    }
    else
    {
      msg.data += " Disarmed";
    }
  }
  
  statusPublisher.publish(msg);
}

void FCU_state_handler(const mavros_msgs::State& state)
{
  current_state = state;
}

void modeHandler(const std_msgs::UInt8::ConstPtr& message) {
  mode = message->data;
}

void juavHandler(const nav_msgs::Odometry::ConstPtr& message) 
{
  juav_current_location.pose = message->pose.pose;
  
  odom_publisher.publish(message);
}


//***********************************************************************************************************************************************************************

int main(int argc, char **argv)
{
   char host[256];
   host[0] = '\0';
   gethostname(host, sizeof host);
   std::string hostname(host);
   std::string name;

   if(argc >= 2) {
      name = argv[1];
   }
   else {
      name = hostname;
   }
   
   ros::init(argc, argv, name + "_BEHAVIOUR");
   ros::NodeHandle nh;
   
   //Subscribers
   ros::Subscriber modeSubscriber = nh.subscribe((name + "/mode"), 1, modeHandler);						//receives mode from GUI
   ros::Subscriber FCU_state_subscriber = nh.subscribe((name + "/mavros/state"), 1, FCU_state_handler);
   ros::Subscriber JUAV_Position_Subscriber = nh.subscribe((name + "/mavros/global_position/local"), 10, juavHandler);		//receives JAUV postition via mavros
   

   //Publishers
   ros::Publisher heartbeatPublisher = nh.advertise<std_msgs::String>(name + "/behaviour/heartbeat", 1, true);
   ros::Timer heartbeatTimer = nh.createTimer(ros::Duration(2),boost::bind(&heartbeatHandler, heartbeatPublisher, _1));
   ros::Publisher statusPublisher = nh.advertise<std_msgs::String>(name + "/status", 1, true);
   ros::Timer statusTimer = nh.createTimer(ros::Duration(2),boost::bind(&statusHandler, statusPublisher, _1));
   
   ros::Publisher rc_override = nh.advertise<mavros_msgs::OverrideRCIn>(name + "/mavros/rc/override", 1);
   odom_publisher = nh.advertise<nav_msgs::Odometry>((name + "/odom/filtered"), 10);
   ros::Publisher guided_publisher = nh.advertise<geometry_msgs::PoseStamped> ((name + "/mavros/setpoint_position/local"), 1);

   //Services
   ros::ServiceClient set_mode = nh.serviceClient<mavros_msgs::SetMode>(name + "/mavros/set_mode");
   ros::ServiceClient arm = nh.serviceClient<mavros_msgs::CommandBool>(name + "/mavros/cmd/arming");
   ros::ServiceClient takeoff = nh.serviceClient<mavros_msgs::CommandTOL>(name + "/mavros/cmd/takeoff");
   
   ros::Rate r(30); // 30 Hz
   while(ros::ok())
   {
      ros::spinOnce();
      if (mode == 1)
      {
        
      }
      else if (mode == 2 || mode == 3) //auto
      {
        target_mode.request.custom_mode = "GUIDED";
        
        if((current_state.armed != true) && landed == true)
        {
          mavros_msgs::CommandBool arming;
          arming.request.value = true;
          bool armed = arm.call(arming);

        }
        
        if((current_state.mode != target_mode.request.custom_mode) && landed == true && current_state.armed == true)
        {
          if(set_mode.call(target_mode))
          {
            mavros_msgs::CommandTOL takeoff_cmd;
            takeoff_cmd.request.altitude = 2.0; //meters
            takeoff_cmd.request.min_pitch = 0;
            takeoff_cmd.request.latitude = 0;
            takeoff_cmd.request.longitude = 0;
            takeoff_cmd.request.yaw = 0;
            
            if (takeoff.call(takeoff_cmd))
            {
              takeoff_attempted = true;
            }
           
          }
          
        }
        
        if(juav_current_location.pose.position.z >= 1 && takeoff_attempted && landed)
        {
          landed = false;
        }
        
        if(!landed)
        {
          geometry_msgs::PoseStamped start_pose;
          start_pose.pose.position.x = 5;
          start_pose.pose.position.y = 5;
          start_pose.pose.position.z = 2;
          guided_publisher.publish(start_pose); 
        }
      }

      r.sleep();
   }
}


//mavros_msgs::OverrideRCIn throttle_up;
//throttle_up.channels[2] = 1200;
//rc_override.publish(throttle_up);
