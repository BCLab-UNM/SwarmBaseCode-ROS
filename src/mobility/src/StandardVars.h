#ifndef STANDARDVARS_H
#define STANDARDVARS_H

#include <geometry_msgs/Pose2D.h>

//this file contains variable declarations that are used throught multiple classes such as the Results struct.
using namespace std;

#define FAST_PID 0 //quickest turn reasponse time
#define SLOW_PID 1 //slower turn reasponse time
#define CONST_PID 2 //constant angular turn rate


struct Result {
  float cmdVel;
  float cmdError;
  float cmdAngular;
  float fingerAngle;
  float wristAngle;
  bool waypointDriving;
  int waypointCount;
  geometry_msgs::Pose2D waypoint[50];
  string changeBehaviour;
  int PID;
};



#endif // STANDARDVARS_H
