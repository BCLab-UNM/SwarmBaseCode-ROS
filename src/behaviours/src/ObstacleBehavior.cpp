#include "ObstacleBehavior.hpp"

#include <algorithm> // std::min

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

bool ObstacleBehavior::AllFar()
{
   return _sensors->GetLeftSonar()   > 2.8
      &&  _sensors->GetCenterSonar() > 2.8
      &&  _sensors->GetRightSonar()  > 2.8;
}

void ObstacleBehavior::Update()
{
   _action = _llAction;

   UpdateState();
   
   switch(_state)
   {
   case Normal:
      if(AllFar()) {
         _action.SetAction(
            core::VelocityAction());
         return;
      }

      if(_sensors->GetLeftSonar() > 0.8)
         _action.SetAction(
            core::VelocityAction(
               AngularVelocity(0.0, 0.0, -std::min(1/pow(0.6 - _sensors->GetLeftSonar(), 4), 1.0))));
      else
         _action.SetAction(
            core::VelocityAction(
               AngularVelocity(0, 0, -1)));

      if(_sensors->GetRightSonar() > 0.8)
         _action.SetAction(
            core::VelocityAction(
               AngularVelocity(0.0, 0.0, std::min(1/pow(0.6 - _sensors->GetLeftSonar(), 4), 1.0))));
      else
         _action.SetAction(
            core::VelocityAction(
               AngularVelocity(0,0,1)));

      break;
   case Turnaround:
      _action.SetAction(core::VelocityAction(AngularVelocity(0,0,1)));
      break;
   }
}
