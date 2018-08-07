/**
 * The main function for the behavior node
 */
#include <memory>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt8.h>

#include "BehaviorManager.hpp"
#include "ObstacleBehavior.hpp"
#include "StraightLineBehavior.hpp"
#include "AvoidNest.hpp"
#include "AlignToCube.hpp"
#include "ApproachCube.hpp"
#include "PickUpCube.hpp"
#include "DropOffCube.hpp"

#include "ROSTimer.hpp"

#include "SwarmieInterface.hpp"

#define CAMERA_OFFSET (-0.02)
#define CAMERA_HEIGHT 0.195
#define AUTO 2
#define MANUAL 1

int mode = 0;

void heartbeatHandler(ros::Publisher& heartbeatPublisher, const ros::TimerEvent& event)
{
   std_msgs::String msg;
   msg.data = "";
   heartbeatPublisher.publish(msg);
}

void statusHandler(ros::Publisher& statusPublisher, const ros::TimerEvent& event)
{
   std_msgs::String msg;
   msg.data = "subsumption2";
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

   ros::Subscriber modeSubscriber = nh.subscribe((name + "/mode"), 1, modeHandler);						//receives mode from GUI
   
   ros::Publisher heartbeatPublisher = nh.advertise<std_msgs::String>(name + "/behaviour/heartbeat", 1, true);
   ros::Timer heartbeatTimer = nh.createTimer(ros::Duration(2),
                                              boost::bind(&heartbeatHandler, heartbeatPublisher, _1));
   ros::Publisher statusPublisher = nh.advertise<std_msgs::String>(name + "/status", 1, true);
   ros::Timer statusTimer = nh.createTimer(ros::Duration(2),
                                           boost::bind(&statusHandler, statusPublisher, _1));

   SwarmieInterface robot(name);
   BehaviorManager  manager;

   ObstacleBehavior     obstacle(new ROSTimer());
   StraightLineBehavior driveStraight;
   AvoidNest            avoidNest(new ROSTimer());
   AlignToCube          align;
   ApproachCube         approach;
   PickUpCube           pickup(new ROSTimer());
   DropOffCube          dropOff(new ROSTimer());

   pickup.Subsumes(&driveStraight);
   pickup.SetRecheckInterval(60);

   dropOff.Subsumes(&pickup);

   manager.RegisterBehavior(&obstacle);
   manager.RegisterBehavior(&driveStraight);
   manager.RegisterBehavior(&avoidNest);
   manager.RegisterBehavior(&align);
   manager.RegisterBehavior(&approach);
   manager.RegisterBehavior(&pickup);
   manager.RegisterBehavior(&dropOff);
 
   ros::Rate r(30); // 30 Hz
   while(ros::ok())
   {
     ros::spinOnce();
     
     if (mode == MANUAL)
     {
       //not implemented
     }
     else if (mode == AUTO)
     {
       const SwarmieSensors& sensors = robot.GetSensors();
       SwarmieAction action = manager.NextAction(sensors);
       robot.DoAction(action);
     }
     
     r.sleep();
   }
}
