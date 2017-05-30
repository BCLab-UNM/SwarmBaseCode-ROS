#ifndef STANDARDVARS_H
#define STANDARDVARS_H

//this file contains variable declarations that are used throught multiple classes such as the Results struct.

struct Result {
  float cmdVel;
  float cmdError;
  float cmdAngular;
  float fingerAngle;
  float wristAngle;
  bool pickedUp;
  bool giveUp;
  bool waypointDriving;
  geometry_msgs::Pose2D waypoint[50];
  string changeBehaviour;
};



#endif // STANDARDVARS_H
