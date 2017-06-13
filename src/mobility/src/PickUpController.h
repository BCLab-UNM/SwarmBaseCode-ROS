#ifndef PICKUPCONTROLLER_H
#define PICKUPCONTROLLER_H
#define HEADERFILE_H
#include <apriltags_ros/AprilTagDetectionArray.h>
#include <ros/ros.h>
#include "StandardVars.h"

class PickUpController
{
 public:
  PickUpController();
  ~PickUpController();
  
  Result CalculateResult();
  
  void UpdateData(const apriltags_ros::AprilTagDetectionArray::ConstPtr& message);
  bool NewUpdateData(float rangeCenter);
  bool ShouldInterrupt();

  float getDistance() {return blockDistance;}
  bool GetLockTarget() {return lockTarget;}
  float GetTD() {return timeDifference;}
  void SetUltraSoundData(bool blockBlock);
  void reset();

private:
  //set true when the target block is less than targetDist so we continue attempting to pick it up rather than
  //switching to another block that is in view
  bool lockTarget; 

  bool targetFound;

  // Failsafe state. No legitimate behavior state. If in this state for too long return to searching as default behavior.
  bool timeOut;
  int nTargetsSeen;
  ros::Time millTimer;

  //yaw error to target block 
  double blockYawError;
  
  //distance to target block from front of robot
  double blockDistance;

  //struct for returning data to mobility
  Result result;

  bool blockBlock;

  float timeDifference;


};
#endif // end header define
