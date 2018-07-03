#include "DropOffCube.hpp"

DropOffCube::DropOffCube(const SwarmieSensors* sensors) :
   Behavior(sensors),
   _state(NotHolding),
   _exited(false),
   _dropped_cube(false)
{}

void DropOffCube::TagHandler()
{
   _tagsLeft  = 0;
   _tagsRight = 0;
   _averagePitch = 0;
   if(_state == NotHolding) return;

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
      double p = tag.GetPitch();
      _averagePitch += p;
   }
   
   if(_tagsLeft + _tagsRight > 0)
   {
      _averagePitch /= _tagsLeft + _tagsRight;
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
      break;
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

void DropOffCube::ApproachCollectionZone()
{
   if(_averagePitch < -0.5)
   {
      std::cout << "Turning Left" << std::endl;
      _action.drive.right = 40;
      _action.drive.left  = 0;
   }
   else if(_averagePitch > 0.5)
   {
      std::cout << "Turning Right" << std::endl;
      _action.drive.right = 0;
      _action.drive.left  = 40;
   }
   else
   {
      std::cout << "Going Straight" << std::endl;
      _action.drive.left  = 40;
      _action.drive.right = 40;
   }
}

void DropOffCube::Update()
{
   UpdateState();
   TagHandler();

   switch(_state)
   {
   case Leaving:
      if(_tagsLeft + _tagsRight > EXIT_THRESHOLD && _averagePitch > 0)
      {
         _exited = true;
      }
      _action.drive.left = -40;
      _action.drive.right = -40;
      _action.wrist = WristControl::UP;
      _action.grip  = GripperControl::OPEN;
      break;
   case Entering:
      if(_tagsLeft + _tagsRight == 0)
      {
         std::cout << "Dropping Cube" << std::endl;
         _action.wrist = WristControl::UP;
         _action.grip  = GripperControl::OPEN;
         _dropped_cube = true;
      }
      
      if(_tagsLeft + _tagsRight < APPROACH_THRESHOLD)
      {
         _action.drive.left  = 40;
         _action.drive.right = 40;
      }
      else
      {
         ApproachCollectionZone();
      }
      break;
   case Holding:
   case NotHolding:
      _action = _llAction;
      break;
   }
}

