#ifndef _TIMER_HPP
#define _TIMER_HPP

class Timer
{
public:
   Timer() {}
   ~Timer() {}
   
   virtual void SetInterval(double t) {}
   virtual void StartOnce() {}
   virtual void Stop() {}
   virtual bool Expired() const {}

   // XXX: Might need additional API functions in the future.
   // int    NumTimesExpired()
   // double TimeSinceExpired()
};

#endif // _TIMER_HPP