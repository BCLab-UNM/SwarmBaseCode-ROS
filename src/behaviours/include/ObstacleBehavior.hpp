#ifndef _OBSTACLE_BEHAVIOR_HPP
#define _OBSTACLE_BEHAVIOR_HPP

#include <math.h>

#include "BehaviorManager.hpp"
#include "Timer.hpp"

class ObstacleBehavior : public Behavior
{
private:
   const double DRIVE_MAX = 512;
   const double TURNAROUND_THRESHOLD = 0.3;
   enum State { Normal, Turnaround } _state;

   Timer* _turnaroundTimer;

   void TurnaroundHandler();
   void UpdateState();
   bool AllFar();
   
public:
    ObstacleBehavior(const SwarmieSensors* sensors, Timer* timer);
   ~ObstacleBehavior() {}

   void Update() override;
};

#endif // _OBSTACLE_BEHAVIOR_HPP
