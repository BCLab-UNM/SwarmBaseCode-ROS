#ifndef _OBSTACLE_BEHAVIOR_HPP
#define _OBSTACLE_BEHAVIOR_HPP

#include <ros/ros.h>
#include <sensor_msgs/Range.h>

#include "BehaviorManager.hpp"

class ObstacleBehavior : public Behavior
{
private:
   ros::NodeHandle _nh;
   
   double _left;
   double _right;
   double _center;

   ros::Subscriber _leftSubscriber;
   ros::Subscriber _rightSubscriber;
   ros::Subscriber _centerSubscriber;
   
   void LeftSonarCallback(  const sensor_msgs::Range& range) { std::cout << "update Left: " << _left << std::endl; _left   = range.range; }
   void RightSonarCallback( const sensor_msgs::Range& range) { _right  = range.range; }
   void CenterSonarCallback(const sensor_msgs::Range& range) { _center = range.range; }

public:
   ObstacleBehavior(std::string name);
   ~ObstacleBehavior() {}
   
   Action GetAction() override;
};

#endif // _OBSTACLE_BEHAVIOR_HPP