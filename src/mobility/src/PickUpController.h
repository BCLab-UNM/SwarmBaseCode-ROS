#ifndef PICKUPCONTROLLER_H
#define PICKUPCONTROLLER_H
#define HEADERFILE_H
#include <apriltags_ros/AprilTagDetectionArray.h>
#include <ros/ros.h>

struct PickUpResult {
  float cmdVel;
  float angleError;
  float fingerAngle;
  float wristAngle;
  bool pickedUp;
  bool giveUp;
};

class PickUpController
{
 public:
  PickUpController();
  ~PickUpController();

  PickUpResult selectTarget(const apriltags_ros::AprilTagDetectionArray::ConstPtr& message);
  PickUpResult pickUpSelectedTarget(bool blockBlock);

  float getDist() {return blockDist;}
  bool getLockTarget() {return lockTarget;}
  float getTD() {return td;}

  void reset();

private:
  //set true when the target block is less than targetDist so we continue attempting to pick it up rather than
  //switching to another block that is in view
  bool lockTarget; 

  // Failsafe state. No legitimate behavior state. If in this state for too long return to searching as default behavior.
  bool timeOut;
  int nTargetsSeen;
  ros::Time millTimer;

  //yaw error to target block 
  double blockYawError;
  
  //distance to target block from front of robot
  double blockDist;

  //struct for returning data to mobility
  PickUpResult result;

  float td;
};
#endif // end header define
