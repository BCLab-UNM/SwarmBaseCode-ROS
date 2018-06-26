#ifndef _TIMER_HPP
#define _TIMER_HPP

class Timer
{
public:
   Timer() {}
   ~Timer() {}
   
   virtual void SetInterval(double t) {}
   virtual void StartOnce() {}
   virtual void StartRepeat() {}
   virtual void Stop() {}
};

#endif // _TIMER_HPP