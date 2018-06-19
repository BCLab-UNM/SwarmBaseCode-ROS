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
      if(tag.GetId() != 0)
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
   _integral(0)
{}

void AlignToCube::Update()
{
   ProcessTags();
   _action = _llAction;

   if(fabs(_distanceToTag) > 0.005)
   {
      // set a turn speed proportionally to the misalignment + an integral term (a PI controller)
      _action.drive.left = (_linearDistance * 80) + _distanceToTag * 100 + 10 * _integral;
      _action.drive.right = (_linearDistance * 80) - (_distanceToTag * 100 + 10 * _integral);
      _integral += _distanceToTag;
   }
   else if(_linearDistance != 0)
   {
      _integral = 0;
      _action.drive.left = (_linearDistance * 80);
      _action.drive.right = (_linearDistance * 80);
   }
   else
   {
      _integral = 0;
   }
}
