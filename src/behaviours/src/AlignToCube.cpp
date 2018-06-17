#include "AlignToCube.hpp"

#define MAX_TURN 100

void AlignToCube::TagHandler(const apriltags_ros::AprilTagDetectionArray::ConstPtr& message)
{
   bool target_detected;
   double minimum_distance = 100;
   double distance = 0;
   for(auto tag : message->detections)
   {
      if(tag.id != 0) continue;
      target_detected = true;
      double d = tag.pose.pose.position.x - _cameraOffset;
      if(fabs(d) < minimum_distance)
      {
         minimum_distance = fabs(d);
         distance = d;
         _linearDistance = hypot(hypot(tag.pose.pose.position.x, tag.pose.pose.position.y), tag.pose.pose.position.z);
      }
   }

   if(target_detected)
      _distanceToTag = distance;
   else
      _distanceToTag = 0;
}

AlignToCube::AlignToCube(std::string name, double cameraOffset) :
   _cameraOffset(cameraOffset),
   _distanceToTag(0),
   _linearDistance(0),
   _integral(0)
{
   _tagSubscriber = _nh.subscribe(name + "/targets", 1, &AlignToCube::TagHandler, this);
}

Action AlignToCube::GetAction()
{
   Action reaction = _llAction;

   if(fabs(_distanceToTag) > 0.005)
   {
      // set a turn speed proportionally to the misalignment + an integral term (a PI controller)
      reaction.drive.left = (_linearDistance * 100) + _distanceToTag * 500 + 10 * _integral;
      reaction.drive.right = (_linearDistance * 100) - (_distanceToTag * 500 + 10 * _integral);
      _integral += _distanceToTag;
   }
   else
   {
      _integral = 0;
   }

   return reaction;
}

