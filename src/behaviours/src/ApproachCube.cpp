#include "ApproachCube.hpp"

ApproachCube::ApproachCube() :
   _distanceToTag(0),
   _alignment(0),
   _approachPID(0.4, 0.05, 0.35)
{}

void ApproachCube::ProcessTags(const SwarmieSensors& sensors)
{
   bool   target_detected = false;
   double minimum_alignment = 100;
   double distance = 0;
   for(auto tag : sensors.GetTags())
   {
      if(!tag.IsCube())
      {
         // ignore the cube if the nest is visible
         _distanceToTag = 0;
         _alignment = 100;
         return;
      }

      target_detected = true;
      if(fabs(tag.Alignment()) < minimum_alignment)
      {
         minimum_alignment = fabs(tag.Alignment());
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

void ApproachCube::Update(const SwarmieSensors& sensors, const SwarmieAction& ll_action)
{
   ProcessTags(sensors);
   _action = ll_action;
   _approachPID.Update(_distanceToTag);

   if(fabs(_alignment) < 0.05)
   {
      core::VelocityAction v = ll_action.GetVelocity();
      v.SetLinear(LinearVelocity(_approachPID.GetControlOutput()));
      _action.SetAction(v);
   }
   else
   {
      _approachPID.Reset();
   }
}
