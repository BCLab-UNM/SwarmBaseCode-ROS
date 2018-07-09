#include "ROSTimer.hpp"

ROSTimer::ROSTimer() :
   _interval(1),
   _nh(),
   _expired(false)
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
   _expired = false;
   _timer.stop();
   _timer.setPeriod(ros::Duration(_interval));
   _timer.start();
}

void ROSTimer::Stop()
{
   _expired = false;
   _timer.stop();
}

void ROSTimer::TimerCallback(const ros::TimerEvent& event)
{
   _expired = true;
   if(!_oneshot)
   {
      _timer.stop();
      _timer.setPeriod(ros::Duration(_interval));
      _timer.start();
   }
}

bool ROSTimer::Expired() const
{
   return _expired;
}
