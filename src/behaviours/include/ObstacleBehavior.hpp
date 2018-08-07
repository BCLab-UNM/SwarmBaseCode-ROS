#ifndef _OBSTACLE_BEHAVIOR_HPP
#define _OBSTACLE_BEHAVIOR_HPP

#include <math.h>

#include "BehaviorManager.hpp"
#include "Timer.hpp"

class ObstacleBehavior : public core::Behavior
{
private:
   const double DRIVE_MAX = 512;
   const double TURNAROUND_THRESHOLD = 0.3;
   enum State { Normal, Turnaround } _state;

   Timer* _turnaroundTimer;

   void TurnaroundHandler();
   void UpdateState(const SwarmieSensors& sensors);
   bool AllFar(const SwarmieSensors& sensors);
   
public:
    ObstacleBehavior(Timer* timer);
   ~ObstacleBehavior() {}

   void Update(const SwarmieSensors& sensors, const SwarmieAction& action) override;
};

#endif // _OBSTACLE_BEHAVIOR_HPP
