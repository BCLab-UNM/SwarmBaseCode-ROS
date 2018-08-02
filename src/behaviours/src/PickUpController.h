#ifndef PICKUPCONTROLLER_H
#define PICKUPCONTROLLER_H

#include "Controller.h"
#include "Tag.h"

class PickUpController : virtual Controller
{
public:
  PickUpController();
  ~PickUpController();

  void Reset() override;
  Result DoWork() override;

  // Give the controller a list of visible april tags.
  void SetTagData(vector<Tag> tags);
  bool ShouldInterrupt() override;
  bool HasWork() override;

  bool SetSonarData(float rangeCenter);

  float getDistance() {return blockDistance;}
  bool GetLockTarget() {return lockTarget;}

  // Give the controller the current ultrasound readings. This is
  // needed because ultrasound data is used to determine whether a
  // block has been successfully picked up.
  void SetUltraSoundData(bool blockBlock);

  bool GetIgnoreCenter() {return ignoreCenterSonar;}
  bool GetTargetHeld() {return targetHeld;}

  void SetCurrentTimeInMilliSecs( long int time );

protected:

  void ProcessData();

private:
  //Set true when the target block is less than targetDist so we continue attempting to pick it up rather than
  //switching to another block that is in view. In other words, the robot focuses on one particular target so
  //it doesn't get confused by having a whole bunch of targets in its view.
  bool lockTarget;

  bool targetFound;
  bool targetHeld;

  // Failsafe state. No legitimate behavior state. If in this state for too long return to searching as default behavior.
  bool timeOut;
  int nTargetsSeen;
  long int millTimer;
  long int target_timer;

  //yaw error to target block
  double blockYawError;

  //distance to target block from front of robot
  double blockDistance;

  //distance to target block from camera
  double blockDistanceFromCamera;

  //struct for returning data to the ROS adapter
  Result result;

  //is the block obstructing the ultrasound
  bool blockBlock;

  //set true when we have locked on to a target to pre-empt obstacle detection being triggered when we pick the target up
  bool ignoreCenterSonar = false;

  //current ROS time from the RosAdapter
  long int current_time;

  //has a controller interupt occurred; this is a guard to prevent this controller from generating multiple interrupts
  //before doing its work
  bool interupted = false;

  //interupt in order to release control; i.e., this controller has finished interupting
  bool release_control = false;

  //this controller has control~
  bool has_control = false;
};

#endif // end header define
