#include "PickUpCube.hpp"

PickUpCube::PickUpCube(Timer* timer) :
   _state(NotHolding),
   _allowReset(true),
   _distanceToTarget(OUT_OF_RANGE),
   _timer(timer),
   _recheckInterval(60.0),
   _sensors(SwarmieSensors())
{}


void PickUpCube::ProcessTags()
{
   _distanceToTarget = OUT_OF_RANGE;

   for(auto tag : _sensors.GetTags())
   {
      if(!tag.IsCube()) continue;

      switch(_state)
      {
      case NotHolding:
         if(fabs(tag.Alignment()) < ALIGNMENT_THRESHOLD
            && tag.HorizontalDistance() < PICKUP_DISTANCE)
         {
            _state = LastInch;
         }
         break;
      case Checking:
         if(fabs(tag.Alignment()) < 0.02)
         {
            if(_distanceToTarget > tag.Distance())
               _distanceToTarget = tag.Distance();
         }
         break;
      default:
         break;
      }
   }
}

inline void PickUpCube::StartTimer(double i)
{
   _timer->Stop();
   _timer->SetInterval(i);
   _timer->StartOnce();
}

inline void PickUpCube::StartCheckTimer()
{
   StartTimer(4.0);
}

inline void PickUpCube::StartRecheckTimer()
{
   StartTimer(_recheckInterval);
}

inline void PickUpCube::ResetPickupTimer(double time)
{
   if(!_allowReset) return;
   
   _allowReset = false;
   StartTimer(time);
}

void PickUpCube::TimerTriggered()
{
   _allowReset = true;
   switch(_state)
   {
   case LastInch:
      _state = Grip;
      break;
   case Grip:
      _state = Raise;
      break;
   case Raise:
      _state = Checking;
      StartCheckTimer();
      break;
   case Checking:
   case Rechecking:
      if(_sensors.GetCenterSonar() < 0.12 || _distanceToTarget < 0.14)
      {
         _state = Holding;
         StartRecheckTimer();
      }
      else
      {
         _state = NotHolding;
      }         
      break;
   case Holding:
      StartCheckTimer();
      _state = Rechecking;
      break;
   }
}

void PickUpCube::Reset()
{
   _state = NotHolding;
   _allowReset = true;
   _timer->Stop();
   _distanceToTarget = OUT_OF_RANGE;
}

void PickUpCube::Update(const SwarmieSensors& sensors, const SwarmieAction& ll_action)
{
   _sensors = sensors;
   ProcessTags();
   if(_timer->Expired())
   {
      TimerTriggered();
   }
   _action = ll_action;

   switch(_state)
   {
   case LastInch:
      _action.SetAction(core::VelocityAction(LinearVelocity(0.6)));
      ResetPickupTimer(0.7);
      break;
   case Grip:
      _action.SetAction(core::VelocityAction());
      _action.SetGrip(GripperControl::CLOSED);
      _action.SetWrist(WristControl::DOWN);
      ResetPickupTimer(1.5);
      break;
   case Raise:
      _action.SetAction(core::VelocityAction());
      _action.SetGrip(GripperControl::CLOSED);
      _action.SetWrist(WristControl::UP);
      ResetPickupTimer(2);
      break;
   case Holding:
      _action = _subsumedBehavior->GetAction();
      _action.SetGrip(GripperControl::CLOSED);
      _action.SetWrist(WristControl::DOWN_2_3);
      break;
   case Checking:
      _action.SetAction(core::VelocityAction());
   case Rechecking:
      _action.SetGrip(GripperControl::CLOSED);
      _action.SetWrist(WristControl::UP);
      break;
   default:
      break;
   }
}

void PickUpCube::SetRecheckInterval(double t)
{
   _recheckInterval = t;
}
