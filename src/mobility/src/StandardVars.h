#ifndef STANDARDVARS_H
#define STANDARDVARS_H

#include <geometry_msgs/Pose2D.h>

//this file contains variable declarations that are used throught multiple classes such as the Results struct.
using namespace std;

struct Result {
  float cmdVel;
  float cmdError;
  float cmdAngular;
  float fingerAngle;
  float wristAngle;
  bool waypointDriving;
  geometry_msgs::Pose2D waypoint[50];
  string changeBehaviour;
};



#endif // STANDARDVARS_H
