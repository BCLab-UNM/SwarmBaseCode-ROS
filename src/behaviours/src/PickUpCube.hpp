#ifndef _PICK_UP_CUBE_HPP
#define _PICK_UP_CUBE_HPP

#include <ros/ros.h>
#include <apriltags_ros/AprilTagDetectionArray.h>
#include <geometry_msgs/Point.h>

#include "BehaviorManager.hpp"

class PickUpCube : public Behavior
{
private:
   enum State { LastInch, Grip, Raise, Wait } _state;
   
   const double PICKUP_DISTANCE = 0.1;

   ros::Subscriber _tagSubscriber;
   ros::Timer      _pickupTimer;

   bool   _allowReset;
   double _cameraOffset;
   double _cameraHeight;

   void TagHandler(const apriltags_ros::AprilTagDetectionArray::ConstPtr& tags);
   void PickupTimeout(const ros::TimerEvent& event);
   
   void   ResetTimer(double time);
   bool   Aligned(const geometry_msgs::Point p);
   double Distance(const geometry_msgs::Point p);
   
public:
   PickUpCube(std::string name, double cameraOffset, double cameraHeight);
   ~PickUpCube() {}
   
   Action GetAction() override;
};

#endif // _PICK_UP_CUBE_HPP