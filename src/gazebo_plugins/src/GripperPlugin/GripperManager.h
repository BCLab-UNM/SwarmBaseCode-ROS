#ifndef GripperManager_h
#define GripperManger_h
#include "FingerController.h"
#include "WristController.h"


// This will be the output type of the getForces function
struct GripperForces {
  float wristForce;
  float leftFingerForce;
  float rightFingerForce;
};

// This is the input type for the getForces function
struct GripperState {
  float wristAngle;
  float leftFingerAngle;
  float rightFingerAngle;
};

class GripperManager {

 public:
  GripperManager();
  ~GripperManager();

  GripperForces getForces( GripperState desState, GripperState curState );

 private:

  FingerController leftFingerController;
  FingerController rightFingerController;
  WristController wristController;

};

#endif // GripperManager_h
