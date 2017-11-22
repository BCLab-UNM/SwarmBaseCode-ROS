#ifndef OBSTACLECONTOLLER_H
#define OBSTACLECONTOLLER_H

#include "Controller.h"
#include "Tag.h"

class ObstacleController : virtual Controller
{
public:
  ObstacleController();

  Result result;

  void Reset() override;
  Result DoWork() override;
  void setSonarData(float left, float center, float right);
  void setCurrentLocation(Point currentLocation);
  void setTagData(vector<Tag> tags);
  bool ShouldInterrupt() override;
  bool HasWork() override;
  void setIgnoreCenterSonar();
  void setCurrentTimeInMilliSecs( long int time );
  void setTargetHeld ();

  // Checks if a target is held and if so resets the state of the obestacle controller otherwise does nothing
  void setTargetHeldClear();
  bool getShouldClearWaypoints() {bool tmp = clearWaypoints; clearWaypoints = false; return tmp;}

protected:

  void ProcessData();

private:

  // Try not to run over the collection zone
  void avoidCollectionZone();

  // Try not to run into a physical object
  void avoidObstacle();

  // Are there AprilTags in the camera view that mark the collection zone
  // and are those AprilTags oriented towards or away from the camera.
  bool checkForCollectionZoneTags( vector<Tag> );
  
  const float K_angular = 1.0; //radians a second
  const float reactivate_center_sonar_threshold = 0.8;
  const int targetCountPivot = 6;
  const float obstacleDistancePivot = 0.2526;
  const float triggerDistance = 0.8;

  /*
     * Member variables
     */


  bool obstacleInterrupt;
  bool obstacleDetected;
  bool obstacleAvoided;
  bool clearWaypoints = false;

  float left = 0;
  float center = 0;
  float right = 0;

  unsigned int count_left_collection_zone_tags;
  unsigned int count_right_collection_zone_tags;

  // Ignore the center sonar because we are carrying a target
  bool ignore_center_sonar = false;

  Point currentLocation;

  long int current_time;
  long int timeSinceTags;
  long int delay;

  bool targetHeld = false;
  bool previousTargetState = false;

  bool phys = false; // Physical obstacle
  bool collection_zone_seen = false; // The obstacle is the collection zone
  
  bool set_waypoint = false;
  bool can_set_waypoint = false;

  float camera_offset_correction = 0.020; //meters;
};

#endif // OBSTACLECONTOLLER_H
