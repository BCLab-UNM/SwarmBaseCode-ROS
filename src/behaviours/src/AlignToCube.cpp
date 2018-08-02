#include "AlignToCube.hpp"

#define MAX_TURN 100

void AlignToCube::ProcessTags(const SwarmieSensors& sensors)
{
   bool target_detected = false;
   double minimum_distance = 100;
   double distance = 0;
   double linearDistance = 0;
   for(auto tag : sensors.GetTags())
   {
      if(tag.GetId() != Tag::CUBE_TAG_ID)
      {
         // If we can detect a nest tag then igore any other detections.
         _distanceToTag = 0;
         _linearDistance = 0;
         return;
      }
      target_detected = true;
      double alignment = tag.Alignment();
      if(fabs(alignment) < minimum_distance)
      {
         minimum_distance = fabs(alignment);
         distance = alignment;
         linearDistance = tag.Distance();
      }
   }

   if(target_detected) {
      _linearDistance = linearDistance;
      _distanceToTag = distance;
   }
   else
   {
      _distanceToTag = 0;
      _linearDistance = 0;
   }
}

AlignToCube::AlignToCube() :
   _distanceToTag(0),
   _linearDistance(0),
   _integral(0),
   _alignPID(-1.5, 0.1, 0.85)
{}

void AlignToCube::Update(const SwarmieSensors& sensors, const SwarmieAction& ll_action)
{
   ProcessTags(sensors);
   _action = ll_action;
   _alignPID.Update(_distanceToTag);

   if(fabs(_distanceToTag) > 0.005)
   {
      core::VelocityAction v;
      v.SetAngular(AngularVelocity(0, 0, _alignPID.GetControlOutput()));
      _action.SetAction(v);
   }
   else if(_linearDistance == 0)
   {
      // no cubes detected, don't take any action.
      _alignPID.Reset();
   }
   else
   {
      // managed to line up, reset the pid, and set the action to idle
      _action.SetAction(core::VelocityAction());
      _alignPID.Reset();
   }
}
