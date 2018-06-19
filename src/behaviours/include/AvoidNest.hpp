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
   bool   _persist;
   bool   _tooClose;
   Action _savedAction;

   ros::Timer      _persistenceTimer;

   void PersistenceCallback(const ros::TimerEvent& event);
   void TagHandler();
public:
   AvoidNest(const SwarmieSensors* sensors);
   ~AvoidNest() {}

   void Update() override;
};

#endif // _AVOID_NEST_HPP