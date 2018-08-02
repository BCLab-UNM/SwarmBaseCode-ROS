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

   void TagHandler(const SwarmieSensors& sensors);
   
public:
   AvoidNest(Timer* timer);
   ~AvoidNest() {}

   void Update(const SwarmieSensors& sensors, const SwarmieAction& ll_action) override;
};

#endif // _AVOID_NEST_HPP