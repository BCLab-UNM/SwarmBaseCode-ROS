#ifndef _PICK_UP_CUBE_HPP
#define _PICK_UP_CUBE_HPP

#include <ros/ros.h>
#include <apriltags_ros/AprilTagDetectionArray.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/Range.h>

#include "BehaviorManager.hpp"
#include "SwarmieSensors.hpp"

class PickUpCube : public Behavior
{
private:
   enum State { LastInch, Grip, Raise, Holding, Rechecking, Checking, NotHolding } _state;
   
   const double PICKUP_DISTANCE = 0.3;
   const double ALIGNMENT_THRESHOLD = 0.005;
   const double OUT_OF_RANGE    = 100;

   ros::Timer      _pickupTimer;
   ros::Timer      _checkTimer;
   ros::Timer      _recheckTimer;

   bool   _allowReset;
   double _cameraOffset;
   double _cameraHeight;
   double _distanceToTarget;
   double _centerRange;

   void ProcessTags();
   void PickupTimeout(const ros::TimerEvent& event);
   void CheckTimeout(const ros::TimerEvent& event);
   void RecheckHandler(const ros::TimerEvent& event);
   void ResetTimer(double time);

public:
   PickUpCube(const SwarmieSensors* sensors);
   ~PickUpCube() {}
   
   void Update() override;
   void SetRecheckInterval(double t);
};

#endif // _PICK_UP_CUBE_HPP
