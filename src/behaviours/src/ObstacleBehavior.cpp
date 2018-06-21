#include "ObstacleBehavior.hpp"

#include <math.h>

#define DRIVE_MAX 512

ObstacleBehavior::ObstacleBehavior(const SwarmieSensors* sensors) :
   Behavior(sensors),
   _state(Normal)
{
   _turnaroundTimer = _nh.createTimer(ros::Duration(1.0), &ObstacleBehavior::TurnaroundHandler, this);
   _turnaroundTimer.stop();
}

void ObstacleBehavior::Update()
{
   _action = _llAction;

   switch(_state)
   {
   case Normal:
      if(_sensors->GetLeftSonar() < TURNAROUND_THRESHOLD
         && _sensors->GetRightSonar() < TURNAROUND_THRESHOLD
         && _sensors->GetCenterSonar() < TURNAROUND_THRESHOLD)
      {
         _state = Turnaround;
         _turnaroundTimer.stop();
         _turnaroundTimer.setPeriod(ros::Duration(1.0));
         _turnaroundTimer.start();
      }
      
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
            _action.drive.right = -_action.drive.right;
         }
         else
         {
            _action.drive.left = -_action.drive.left;
         }
      }
      break;
   case Turnaround:
      _action.drive.right = DRIVE_MAX;
      _action.drive.left  = -DRIVE_MAX;
      break;
   }
}

void ObstacleBehavior::TurnaroundHandler(const ros::TimerEvent& event)
{
   _state = Normal;
}
