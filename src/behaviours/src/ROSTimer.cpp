#include "ROSTimer.hpp"

ROSTimer::ROSTimer(std::function<void()> callback) :
   _interval(1),
   _nh(),
   _callback(callback)
{
   _timer = _nh.createTimer(ros::Duration(_interval), &ROSTimer::TimerCallback, this, true);
   _timer.stop();
}

void ROSTimer::SetInterval(double t)
{
   _interval = t;
   _timer.setPeriod(ros::Duration(_interval));
}

void ROSTimer::StartOnce()
{
   _oneshot = true;
   _timer.stop();
   _timer.setPeriod(ros::Duration(_interval));
   _timer.start();
}

void ROSTimer::StartRepeat()
{
   _oneshot = false;
   _timer.stop();
   _timer.setPeriod(ros::Duration(_interval));
   _timer.start();
}

void ROSTimer::Stop()
{
   _timer.stop();
}

void ROSTimer::TimerCallback(const ros::TimerEvent& event)
{
   _callback();
   if(!_oneshot)
   {
      _timer.stop();
      _timer.setPeriod(ros::Duration(_interval));
      _timer.start();
   }
}




