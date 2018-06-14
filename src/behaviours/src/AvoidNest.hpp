#ifndef _AVOID_NEST_HPP
#define _AVOID_NEST_HPP

#include "BehaviorManager.hpp"

#include <ros/ros.h>
#include <apriltags_ros/AprilTagDetectionArray.h>

class AvoidNest : public Behavior
{
private:
   int    _tagsLeft;
   int    _tagsRight;
   double _cameraOffset;
   double _cameraHeight;
   bool   _persist;
   bool   _tooClose;
   Action _savedAction;

   ros::Subscriber _tagSubscriber;
   ros::Timer      _persistenceTimer;

   void   PersistenceCallback(const ros::TimerEvent& event);
   void   TagHandler(const apriltags_ros::AprilTagDetectionArray::ConstPtr& message);
   double HorizontalDistance(double y, double x, double z);
public:
   AvoidNest(std::string name, double cameraOffset, double cameraHeight);
   ~AvoidNest() {}
   
   Action GetAction() override;
   
};

#endif // _AVOID_NEST_HPP