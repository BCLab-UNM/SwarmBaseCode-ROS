#ifndef _OBSTACLE_BEHAVIOR_HPP
#define _OBSTACLE_BEHAVIOR_HPP

#include <ros/ros.h>
#include <sensor_msgs/Range.h>

#include "BehaviorManager.hpp"

class ObstacleBehavior : public Behavior
{
private:
   const double TURNAROUND_THRESHOLD = 0.3;
   enum State { Normal, Turnaround } _state;

   ros::Timer _turnaroundTimer;
   void TurnaroundHandler(const ros::TimerEvent& event);
   void UpdateState();
public:
   ObstacleBehavior(const SwarmieSensors* sensors);
   ~ObstacleBehavior() {}

   void Update() override;
};

#endif // _OBSTACLE_BEHAVIOR_HPP
