#include "DropOffCube.hpp"

DropOffCube::DropOffCube(const SwarmieSensors* sensors) :
   Behavior(sensors),
   _state(NotHolding),
   _exited(false),
   _dropped_cube(false)
{}

void DropOffCube::TagHandler()
{
   // only process tags if the robot is holding a cube.
   if(_state != Holding) return;

   _tagsLeft  = 0;
   _tagsRight = 0;

   for(auto tag : _sensors->GetTags())
   {
      if(!tag.IsNest()) continue;
      _state = Entering;

      if(tag.Alignment() > 0)
      {
         _tagsRight++;
      }
      else
      {
         _tagsLeft++;
      }
   }
}

void DropOffCube::UpdateState()
{
   switch(_state)
   {
   case Leaving:
      if(_exited)
      {
         _state = NotHolding;
         _exited = false;
      }
   case Entering:
      if(_dropped_cube)
      {
         _state = Leaving;
         _dropped_cube = false;
      }
      break;
   case Holding:
      if(_llAction.grip == GripperControl::OPEN)
      {
         _state = NotHolding;
      }
      TagHandler();
      break;
   case NotHolding:
      if(_llAction.grip == GripperControl::CLOSED)
      {
         _state = Holding;
      }
      break;
   }
}

void DropOffCube::Update()
{
   UpdateState();
   TagHandler();

   switch(_state)
   {
   case Leaving:
      if(_tagsLeft + _tagsRight > EXIT_THRESHOLD)
      {
         _exited = true;
      }
      _action.drive.left = -60;
      _action.drive.right = -60;
      _action.wrist = WristControl::UP;
      _action.grip  = GripperControl::OPEN;
      break;
   case Entering:
      if(_tagsLeft + _tagsRight == 0)
      {
         _action.wrist = WristControl::UP;
         _action.grip  = GripperControl::OPEN;
         _dropped_cube = true;
      }

      if(_tagsLeft > _tagsRight)
      {
         _action.drive.right = 45;
         _action.drive.left  = 0;
      }
      else if(_tagsRight > _tagsLeft)
      {
         _action.drive.right = 0;
         _action.drive.left  = 45;
      }
      else
      {
         _action.drive.left  = 45;
         _action.drive.right = 45;
      }
      break;
   case Holding:
   case NotHolding:
      _action = _llAction;
      break;
   }
}

