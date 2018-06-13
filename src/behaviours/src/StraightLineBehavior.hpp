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
private:
   ros::NodeHandle _nh;
   ros::Subscriber _odomSubscriber;

   bool   _heading_set;
   double _heading;

   void OdometryHandler(const nav_msgs::Odometry::ConstPtr& message);
public:
   StraightLineBehavior(std::string name);
   ~StraightLineBehavior() {}

   Action GetAction() override;
};

#endif // _STRAIGHT_LINE_BEHAVIOR_HPP