#ifndef _ROS_TIMER_HPP
#define _ROS_TIMER_HPP

#include "Timer.hpp"
#include <ros/ros.h>

#include <functional> // std::function

class ROSTimer : public Timer
{
private:
   ros::Timer _timer;
   ros::NodeHandle _nh;
   bool _oneshot;
   double _interval;
   std::function<void()> _callback;

   void TimerCallback(const ros::TimerEvent& event);
public:
   ROSTimer(std::function<void()> callback);
   ~ROSTimer() {}
   
   void SetInterval(double t) override;
   void StartOnce() override;
   void StartRepeat() override;
   void Stop() override;
};

#endif // _ROS_TIMER_HPP