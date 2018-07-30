#ifndef _AVOID_NEST_HPP
#define _AVOID_NEST_HPP

#include "BehaviorManager.hpp"
#include "SwarmieSensors.hpp"
#include "Timer.hpp"

#include <cmath>

class AvoidNest : public Behavior
{
private:
   int    _tagsLeft;
   int    _tagsRight;
   bool   _persist;
   bool   _tooClose;
   SwarmieAction _savedAction;

   Timer* _persistenceTimer;

   void TagHandler();
   
public:
   AvoidNest(const SwarmieSensors* sensors, Timer* timer);
   ~AvoidNest() {}

   void Update() override;
};

#endif // _AVOID_NEST_HPP