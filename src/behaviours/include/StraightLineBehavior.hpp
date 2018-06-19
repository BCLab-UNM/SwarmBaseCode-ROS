#ifndef _STRAIGHT_LINE_BEHAVIOR_HPP
#define _STRAIGHT_LINE_BEHAVIOR_HPP

#include "BehaviorManager.hpp"

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

/**
 * Drive at a constant velocity while maintaining the current heading
 */
class StraightLineBehavior : public Behavior
{
public:
   StraightLineBehavior();
   ~StraightLineBehavior() {}

   void Update() override;
};

#endif // _STRAIGHT_LINE_BEHAVIOR_HPP