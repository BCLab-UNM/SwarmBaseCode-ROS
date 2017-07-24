#ifndef OBSTACLECONTOLLER_H
#define OBSTACLECONTOLLER_H

#include "Controller.h"
#include "TagPoint.h"

class ObstacleController : virtual Controller
{
public:
  ObstacleController();

  Result result;

  void Reset() override;
  Result DoWork() override;
  void SetSonarData(float left, float center, float right);
  void SetCurrentLocation(Point currentLocation);
  void SetTagData(vector<TagPoint> tags);
  bool ShouldInterrupt() override;
  bool HasWork() override;
  void SetIgnoreCenter();
  void SetCurrentTimeInMilliSecs( long int time );
  void SetTargetHeld ();
  void SetTargetHeldClear() {targetHeld = false; previousTargetState = false;}
  bool GetShouldClearWaypoints() {bool tmp = clearWaypoints; clearWaypoints = false; return tmp;}

protected:

  void ProcessData();

private:

  const float K_angular = 0.01500f;
  const float reactivateCenterThreshold = 0.8;
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

  int countLeft;
  int countRight;
  bool centerSeen;

  bool ignoreCenter = false;

  Point currentLocation;

  long int current_time;
  long int timeSinceTags;

  bool targetHeld = false;
  bool previousTargetState = false;

};

#endif // OBSTACLECONTOLLER_H
