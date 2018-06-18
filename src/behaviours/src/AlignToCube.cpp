#include "AlignToCube.hpp"

#define MAX_TURN 100

void AlignToCube::TagHandler(const apriltags_ros::AprilTagDetectionArray::ConstPtr& message)
{
   bool target_detected;
   double minimum_distance = 100;
   double distance = 0;
   double linearDistance = 0;
   for(auto tag : message->detections)
   {
      if(tag.id != 0) continue;
      target_detected = true;
      double d = tag.pose.pose.position.x - _cameraOffset;
      if(fabs(d) < minimum_distance)
      {
         minimum_distance = fabs(d);
         distance = d;
         linearDistance = hypot(hypot(tag.pose.pose.position.x, tag.pose.pose.position.y), tag.pose.pose.position.z);
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

AlignToCube::AlignToCube(std::string name, double cameraOffset) :
   _cameraOffset(cameraOffset),
   _distanceToTag(0),
   _linearDistance(0),
   _integral(0)
{
   _tagSubscriber = _nh.subscribe(name + "/targets", 1, &AlignToCube::TagHandler, this);
}

void AlignToCube::Update()
{
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
