#include "ObstacleBehavior.hpp"

ObstacleBehavior::ObstacleBehavior(const SwarmieSensors* sensors, Timer* timer) :
   Behavior(sensors),
   _state(Normal),
   _turnaroundTimer(timer)
{
   _turnaroundTimer->SetInterval(1.0);
}

void ObstacleBehavior::UpdateState()
{
   if(_state == Turnaround && _turnaroundTimer->Expired())
   {
      _state = Normal;
   }
   
   // must be a separate if statement, rather than an else, so the
   // robot transitions back to 'Turnaround' if there are still
   // obstacles too close to it.
   if(_state == Normal)
   {
      if(_sensors->GetLeftSonar() < TURNAROUND_THRESHOLD
         || _sensors->GetRightSonar() < TURNAROUND_THRESHOLD
         || _sensors->GetCenterSonar() < TURNAROUND_THRESHOLD)
      {
         _state = Turnaround;
         _turnaroundTimer->StartOnce();
      }      
   }
}

void ObstacleBehavior::Update()
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
