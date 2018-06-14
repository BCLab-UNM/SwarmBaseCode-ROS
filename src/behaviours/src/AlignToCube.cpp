#include "AlignToCube.hpp"

#include <cmath>

#define MAX_TURN 100

void AlignToCube::TagHandler(const apriltags_ros::AprilTagDetectionArray::ConstPtr& message)
{
   double minimum_distance = 0;
   double distance = 0;
   for(auto tag : message->detections)
   {
      if(tag.id != 0) continue;

      double d = tag.pose.pose.position.x - _cameraOffset;
      if(abs(d) < minimum_distance)
      {
         minimum_distance = abs(d);
         distance = d;
      }
   }

   _distanceToTag = distance;
}

AlignToCube::AlignToCube(std::string name, double cameraOffset) :
   _cameraOffset(cameraOffset),
   _distanceToTag(0)
{
   _tagSubscriber = _nh.subscribe(name + "/targets", 1, &AlignToCube::TagHandler, this);
}

Action AlignToCube::GetAction()
{
   Action reaction = _llAction;

   if(abs(_distanceToTag) > 0.0001) // arbitrary small value
   {
      // set a turn speed proportionally to the misalignment
      reaction.drive.left = 5 * _distanceToTag * MAX_TURN;
      reaction.drive.right = -5 * _distanceToTag * MAX_TURN;
   }
   
   return reaction;
}

