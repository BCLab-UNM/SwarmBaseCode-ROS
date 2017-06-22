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
    float cmdVel;
    float cmdAngularError;
    float cmdAngular;
    float setPointVel;
    float setPointYaw;

    float left;
    float right;
};

struct Waypoints {
    vector<Point> waypoints;
};

struct Result {
    ResultType type;

    BehaviorTrigger b;
    Waypoints wpts;
    PrecisionDriving pd;
    
    float fingerAngle;
    float wristAngle;
    PIDType PIDMode;

    bool reset;
};
