/**
 * The main function for the behavior node
 */
#include <memory>

#include <ros/ros.h>
#include <std_msgs/String.h>

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

   ros::Publisher heartbeatPublisher = nh.advertise<std_msgs::String>(name + "/behaviour/heartbeat", 1, true);
   ros::Timer heartbeatTimer = nh.createTimer(ros::Duration(2),
                                              boost::bind(&heartbeatHandler, heartbeatPublisher, _1));
   ros::Publisher statusPublisher = nh.advertise<std_msgs::String>(name + "/status", 1, true);
   ros::Timer statusTimer = nh.createTimer(ros::Duration(2),
                                           boost::bind(&statusHandler, statusPublisher, _1));

   SwarmieInterface robot(name);
   BehaviorManager  manager;
   const SwarmieSensors* sensors = robot.GetSensors();

   ObstacleBehavior<ROSTimer> obstacle(sensors);
   StraightLineBehavior       driveStraight;
   AvoidNest<ROSTimer>        avoidNest(sensors);
   AlignToCube                align(sensors);
   ApproachCube               approach(sensors);
   PickUpCube<ROSTimer>       pickup(sensors);
   DropOffCube<ROSTimer>      dropOff(sensors);

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
      Action action = manager.NextAction();
      robot.DoAction(action);
      r.sleep();
   }
}