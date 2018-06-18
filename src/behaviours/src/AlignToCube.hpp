#include "BehaviorManager.hpp"

#include <ros/ros.h>
#include <apriltags_ros/AprilTagDetectionArray.h>

class AlignToCube : public Behavior
{
private:
   double _cameraOffset;
   double _distanceToTag;
   double _linearDistance;

   double _integral;

   ros::Subscriber _tagSubscriber;

   void TagHandler(const apriltags_ros::AprilTagDetectionArray::ConstPtr& message);

public:
   AlignToCube(std::string name, double cameraOffset);
   ~AlignToCube() {}

   void Update() override;
};