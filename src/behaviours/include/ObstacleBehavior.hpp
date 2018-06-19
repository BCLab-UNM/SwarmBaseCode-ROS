#ifndef _OBSTACLE_BEHAVIOR_HPP
#define _OBSTACLE_BEHAVIOR_HPP

#include <ros/ros.h>
#include <sensor_msgs/Range.h>

#include "BehaviorManager.hpp"

class ObstacleBehavior : public Behavior
{
public:
   ObstacleBehavior(const SwarmieSensors* sensors);
   ~ObstacleBehavior() {}

   void Update() override;
};

#endif // _OBSTACLE_BEHAVIOR_HPP