#include "BehaviorManager.hpp"

#include <ros/ros.h>
#include <apriltags_ros/AprilTagDetectionArray.h>

class AlignToCube : public Behavior
{
private:
   double _distanceToTag;
   double _linearDistance;

   double _integral;

   void ProcessTags();

public:
   AlignToCube(const SwarmieSensors* sensors);
   ~AlignToCube() {}

   void Update() override;
};