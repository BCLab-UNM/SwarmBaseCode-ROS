#include "ObstacleBehavior.hpp"

#include <algorithm> // std::min

ObstacleBehavior::ObstacleBehavior(Timer* timer) :
   _state(Normal),
   _turnaroundTimer(timer)
{
   _turnaroundTimer->SetInterval(1.0);
}

void ObstacleBehavior::UpdateState(const SwarmieSensors& sensors)
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
      if(sensors.GetLeftSonar() < TURNAROUND_THRESHOLD
         || sensors.GetRightSonar() < TURNAROUND_THRESHOLD
         || sensors.GetCenterSonar() < TURNAROUND_THRESHOLD)
      {
         _state = Turnaround;
         _turnaroundTimer->StartOnce();
      }      
   }
}

bool ObstacleBehavior::AllFar(const SwarmieSensors& sensors)
{
   return sensors.GetLeftSonar()   > 2.8
      &&  sensors.GetCenterSonar() > 2.8
      &&  sensors.GetRightSonar()  > 2.8;
}

void ObstacleBehavior::Update(const SwarmieSensors& sensors, const SwarmieAction& ll_action)
{
   _action = ll_action;

   UpdateState(sensors);
   
   switch(_state)
   {
   case Normal:
      if(AllFar(sensors)) {
         _action.SetAction(
            core::VelocityAction());
         return;
      }

      if(sensors.GetLeftSonar() > 0.8)
         _action.SetAction(
            core::VelocityAction(
               AngularVelocity(0.0, 0.0, -std::min(1/pow(0.6 - sensors.GetLeftSonar(), 4), 1.0))));
      else
         _action.SetAction(
            core::VelocityAction(
               AngularVelocity(0, 0, -1)));

      if(sensors.GetRightSonar() > 0.8)
         _action.SetAction(
            core::VelocityAction(
               AngularVelocity(0.0, 0.0, std::min(1/pow(0.6 - sensors.GetLeftSonar(), 4), 1.0))));
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
