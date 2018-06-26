#ifndef _OBSTACLE_BEHAVIOR_HPP
#define _OBSTACLE_BEHAVIOR_HPP

#include <math.h>

#include "BehaviorManager.hpp"

template <typename T>
class ObstacleBehavior : public Behavior
{
private:
   const double DRIVE_MAX = 512;
   const double TURNAROUND_THRESHOLD = 0.3;
   enum State { Normal, Turnaround } _state;

   T _turnaroundTimer;
   void TurnaroundHandler()
   {
      _state = Normal;
   }

   void UpdateState()
   {
      switch(_state)
      {
      case Normal:
         if(_sensors->GetLeftSonar() < TURNAROUND_THRESHOLD
            || _sensors->GetRightSonar() < TURNAROUND_THRESHOLD
            || _sensors->GetCenterSonar() < TURNAROUND_THRESHOLD)
         {
            _state = Turnaround;
            _turnaroundTimer.StartOnce();
         }
         break;
      }      
   }
   
public:
   ObstacleBehavior(const SwarmieSensors* sensors) :
      Behavior(sensors),
      _state(Normal),
      _turnaroundTimer([this]() { this->TurnaroundHandler(); })
   {
      _turnaroundTimer.SetInterval(1.0);
   }

   ~ObstacleBehavior() {}

   void Update() override
   {
      _action = _llAction;

      UpdateState();
   
      switch(_state)
      {
      case Normal:
         if(_sensors->GetLeftSonar() > 0.6)
            _action.drive.left  = 1/pow(0.6 - _sensors->GetLeftSonar(), 4);
         else
            _action.drive.left = DRIVE_MAX;

         if(_sensors->GetRightSonar() > 0.6)
            _action.drive.right = 1/pow(0.6 - _sensors->GetRightSonar(), 4);
         else
            _action.drive.right = DRIVE_MAX;

         if(_sensors->GetCenterSonar() < 0.8) {
            if(_sensors->GetLeftSonar() < _sensors->GetRightSonar())
            {
               _action.drive.right = -DRIVE_MAX;
            }
            else
            {
               _action.drive.left = -DRIVE_MAX;
            }
         }
         break;
      case Turnaround:
         _action.drive.right = DRIVE_MAX;
         _action.drive.left  = -DRIVE_MAX;
         break;
      }
   }
};

#endif // _OBSTACLE_BEHAVIOR_HPP
