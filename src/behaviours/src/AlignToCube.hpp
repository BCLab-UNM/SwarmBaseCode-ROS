#include "BehaviorManager.hpp"

#include <ros/ros.h>
#include <apriltags_ros/AprilTagDetectionArray.h>

class AlignToCube : public Behavior
{
private:
   double _cameraOffset;
   double _distanceToTag;

   ros::Subscriber _tagSubscriber;

   void TagHandler(const apriltags_ros::AprilTagDetectionArray::ConstPtr& message);

public:
   AlignToCube(std::string name, double cameraOffset);
   ~AlignToCube() {}

   Action GetAction() override;
};