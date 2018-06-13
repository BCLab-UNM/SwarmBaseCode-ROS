/**
 * The main function for the behavior node
 */

#include <ros/ros.h>

#include "BehaviorManager.hpp"
#include "ObstacleBehavior.hpp"
#include "StraightLineBehavior.hpp"

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

   BehaviorManager manager(DEFAULT_RATE, name);
   ObstacleBehavior obstacle(name);
   manager.DeclareBehavior(&obstacle);
   StraightLineBehavior driveStraight(name);
   manager.DeclareBehavior(&driveStraight);

   ros::spin();
}