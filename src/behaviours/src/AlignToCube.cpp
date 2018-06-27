#include "AlignToCube.hpp"

#define MAX_TURN 100

void AlignToCube::ProcessTags()
{
   bool target_detected;
   double minimum_distance = 100;
   double distance = 0;
   double linearDistance = 0;
   for(auto tag : _sensors->GetTags())
   {
      if(tag.GetId() != CUBE_TAG_ID)
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

AlignToCube::AlignToCube(const SwarmieSensors* sensors) :
   Behavior(sensors),
   _distanceToTag(0),
   _linearDistance(0),
   _integral(0),
   _alignPID(0.88, 0.6, 0.2)
{}

void AlignToCube::Update()
{
   ProcessTags();
   _action = _llAction;
   _alignPID.Update(_distanceToTag);

   if(fabs(_distanceToTag) > 0.005)
   {
      _action.drive.left = 100*_alignPID.GetControlOutput();
      _action.drive.right = - (100*_alignPID.GetControlOutput());
   }
   else
   {
      // managed to line up, reset the pid.
      _alignPID.Reset();
   }
}
