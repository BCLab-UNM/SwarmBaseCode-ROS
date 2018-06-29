#include "ApproachCube.hpp"

ApproachCube::ApproachCube(const SwarmieSensors *sensors) :
   Behavior(sensors),
   _distanceToTag(0),
   _alignment(0),
   _approachPID(0.55, 0.05, 0.25)
{}

void ApproachCube::ProcessTags()
{
   bool   target_detected = false;
   double minimum_distance = 100;
   double distance = 0;
   for(auto tag : _sensors->GetTags())
   {
      if(!tag.IsCube())
      {
         // ignore the cube if the nest is visible
         _distanceToTag = 0;
         _alignment = 100;
         return;
      }

      target_detected = true;
      if(fabs(tag.Alignment()) < minimum_distance)
      {
         minimum_distance = tag.Alignment();
         _alignment = tag.Alignment();
         _distanceToTag = tag.HorizontalDistance();
      }
   }

   if(!target_detected)
   {
      _distanceToTag = 0;
      _alignment = 100;
   }
}

void ApproachCube::Update()
{
   ProcessTags();
   _action = _llAction;
   _approachPID.Update(_distanceToTag);

   if(fabs(_alignment) < 0.05)
   {
      _action.drive.left += 60*_approachPID.GetControlOutput();
      _action.drive.right += 60*_approachPID.GetControlOutput();
   }
   else
   {
      _approachPID.Reset();
   }
}
