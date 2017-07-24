// Defines the Result type which used to standardize the information that
// can be returned by a controller object.

/* EXAMPLE:
 *
 * struct Result res;
 * res.type = ...;
 * res.b = ...;
 * ...
 *
 * if(res.type == behavior) {
 *      if(res.b == targetDropped) {
 *          ...
 *      } else if(res.b == ...) {
 *          ...
 *      } ...
 * } else if(res.type == waypoint) {
 *      Pose2D next = res.wpts.waypoint[0];
 *      ...
 * } else if(res.type == precisionDriving) {
 *      ...
 * }
 *
 *
 *
 */

#include <vector>

#include "Point.h"
#include <iostream>

using namespace std;

enum PIDType {
  FAST_PID, //quickest turn reasponse time
  SLOW_PID, //slower turn reasponse time
  CONST_PID //constant angular turn rate
};

enum ResultType {
  behavior,
  waypoint,
  precisionDriving
};

enum BehaviorTrigger {
  wait,
  prevProcess,
  noChange,
  nextProcess
};

struct PrecisionDriving {
  float cmdVel = 0.0;
  float cmdAngularError = 0.0;
  float cmdAngular = 0.0;
  float setPointVel = 0.0;
  float setPointYaw = 0.0;

  float left = 0.0;
  float right = 0.0;
};

struct Waypoints {
  vector<Point> waypoints;
};

struct Result {
  ResultType type;

  BehaviorTrigger b;
  Waypoints wpts;
  PrecisionDriving pd;

  float fingerAngle = -1;
  float wristAngle = -1;
  PIDType PIDMode;

  bool reset;
};
