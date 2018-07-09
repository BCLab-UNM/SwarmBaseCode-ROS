#ifndef _ROS_TIMER_HPP
#define _ROS_TIMER_HPP

#include "Timer.hpp"
#include <ros/ros.h>

class ROSTimer : public Timer
{
private:
   ros::Timer      _timer;
   ros::NodeHandle _nh;

   bool   _oneshot;
   double _interval;
   bool   _expired;

   void TimerCallback(const ros::TimerEvent& event);
public:
   ROSTimer();
   ~ROSTimer() {}
   
   void SetInterval(double t) override;
   void StartOnce() override;
   void Stop() override;
   bool Expired() const override;
};

#endif // _ROS_TIMER_HPP