/**
 * The main function for the behavior node
 */
#include <memory>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt8.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/SetMode.h>


#include "ROSTimer.hpp"


#define CAMERA_OFFSET (-0.02)
#define CAMERA_HEIGHT 0.195

int mode = 0;
mavros_msgs::SetMode JUAV_target_mode;

void heartbeatHandler(ros::Publisher& heartbeatPublisher, const ros::TimerEvent& event)
{
   std_msgs::String msg;
   msg.data = "";
   heartbeatPublisher.publish(msg);
}

void statusHandler(ros::Publisher& statusPublisher, const ros::TimerEvent& event)
{
   std_msgs::String msg;
   msg.data = "JUAV_subsumption2";
   statusPublisher.publish(msg);
}

void modeHandler(const std_msgs::UInt8::ConstPtr& message) {
  mode = message->data;
}

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

   //Publishers
   ros::Publisher heartbeatPublisher = nh.advertise<std_msgs::String>(name + "/behaviour/heartbeat", 1, true);
   ros::Timer heartbeatTimer = nh.createTimer(ros::Duration(2),
                                              boost::bind(&heartbeatHandler, heartbeatPublisher, _1));
   ros::Publisher statusPublisher = nh.advertise<std_msgs::String>(name + "/status", 1, true);
   ros::Timer statusTimer = nh.createTimer(ros::Duration(2),
                                           boost::bind(&statusHandler, statusPublisher, _1));

   //Services
   ros::ServiceClient set_mode = nh.serviceClient<mavros_msgs::SetMode>("mavros/State");
   
   ros::Rate r(30); // 30 Hz
   while(ros::ok())
   {
      ros::spinOnce();
      if (mode == 1)
      {
        
      }
      else if (mode == 2 || mode == 3)
      {
        
      }
      

      r.sleep();
   }
}
