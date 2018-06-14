/**
 * The main function for the behavior node
 */

#include <ros/ros.h>

#include "BehaviorManager.hpp"
#include "ObstacleBehavior.hpp"
#include "StraightLineBehavior.hpp"
#include "AvoidNest.hpp"

#define DEFAULT_RATE 0.05

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

   RobotInterface robot(name);
   BehaviorManager manager;
   ObstacleBehavior obstacle(name);
   manager.RegisterBehavior(&obstacle);
   StraightLineBehavior driveStraight(name);
   manager.RegisterBehavior(&driveStraight);
   AvoidNest avoidNest(name, 0.02); // 0.02 is the offset of the camera from the center of the robot
   manager.RegisterBehavior(&avoidNest);

   ros::Rate r(30); // 30 Hz
   while(ros::ok())
   {
      ros::spinOnce();
      Action action = manager.NextAction();
      robot.DoAction(action);
      r.sleep();
   }
}