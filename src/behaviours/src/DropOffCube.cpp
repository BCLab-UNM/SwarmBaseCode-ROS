#include "DropOffCube.hpp"

DropOffCube::DropOffCube(const SwarmieSensors *sensors, Timer* timer) :
   Behavior(sensors),
   _state(NotHolding),
   _timer(timer),
   _centeringPID(1.5, 0.2, 0.5) // XXX: Not tuned
{}


void DropOffCube::TagHandler()
{
   _numNestTags = 0;
   _averageAlignment = 0;

   for(auto tag : _sensors->GetTags())
   {
      if(!tag.IsNest()) continue;
      _averageAlignment += tag.Alignment();
      _numNestTags++;
   }

   if(_numNestTags > 0)
   {
      _averageAlignment /= _numNestTags;
   }
}

void DropOffCube::UpdateState()
{
   switch(_state)
   {
   case NotHolding:
      if(_llAction.GripperCommand() == GripperControl::CLOSED) {
         _state = Holding;
      }
      break;
   case Holding:
      if(_llAction.GripperCommand() == GripperControl::OPEN) {
         _state = NotHolding;
      }
      else if(_numNestTags > 0)
      {
         _state = Centering;
         _centeringPID.Reset();
      }
      break;
   case Centering:
      if(_numNestTags == 0)
      {
         // TODO: Don't consider target lost until it is not seen
         // for X ticks
         _state = Holding;
      }
      else if(fabs(_averageAlignment) < 0.02)
      {
         _state = Entering;
         _timer->SetInterval(2.25);
         _timer->StartOnce();
      }
      break;
   }
}

void DropOffCube::Timeout()
{
   switch(_state)
   {
   case Entering:
      _state = Droping;
      _timer->SetInterval(1.0);
      _timer->StartOnce();
      break;
   case Droping:
      _state = Leaving;
      _timer->SetInterval(3.5);
      _timer->StartOnce();
      break;
   case Leaving:
      _subsumedBehavior->Reset();
      _state = NotHolding;
      break;
   }
}

void DropOffCube::CheckTimeout()
{
   // These are the only states where the timer is running.
   if(_state == Entering || _state == Droping || _state == Leaving)
   {
      if(_timer->Expired())
      {
         Timeout();
      }
   }
}

void DropOffCube::Center()
{
   _centeringPID.Update(_averageAlignment);
   double control = _centeringPID.GetControlOutput();
   core::VelocityAction v;
   v.SetAngular(AngularVelocity(0, 0, -control));
   _action.SetAction(v);
}

void DropOffCube::Enter()
{
   _action.SetAction(core::VelocityAction(LinearVelocity(0.35)));
}

void DropOffCube::Leave()
{
   _action.SetAction(core::VelocityAction(LinearVelocity(-0.35)));
   _action.SetGrip(GripperControl::OPEN);
   _action.SetWrist(WristControl::UP);
}

void DropOffCube::Update()
{
   TagHandler();
   CheckTimeout();
   UpdateState();

   switch(_state)
   {
   case Centering:
      Center();
      break;
   case Entering:
      Enter();
      break;
   case Leaving:
      Leave();
      break;
   case Droping:
      _action.SetAction(core::VelocityAction());
      _action.SetGrip(GripperControl::OPEN);
      _action.SetWrist(WristControl::UP);
   case Holding:
   case NotHolding:
      _action = _llAction;
      break;
   }
}

