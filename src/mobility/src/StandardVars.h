#ifndef STANDARDVARS_H
#define STANDARDVARS_H

#include <geometry_msgs/Pose2D.h>
#include <vector>

//this file contains variable declarations that are used throught multiple classes such as the Results struct.
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
    init,
    targetDropped,
    targetReturned,
    targetLost,
    targetPickedUp,
    obstacleAvoided
};

struct PrecisionDriving {
    float cmdVel;
    float cmdAngularError;
    float cmdAngular;
    float setPointVel;
    float setPointYaw;
};

struct Waypoints {
    vector<geometry_msgs::Pose2D> waypoints;
};


struct Result {
    ResultType type;

    BehaviorTrigger b;
    Waypoints wpts;
    PrecisionDriving pd;
    
    float fingerAngle;
    float wristAngle;
    PIDType PIDMode;
};


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


#endif // STANDARDVARS_H
