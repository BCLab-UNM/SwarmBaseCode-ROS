#ifndef FingerController_h
#define FingerController_h

#include "PIDController.h"

class FingerController : public PIDController {

 public:
  FingerController();
  FingerController( float dt, float max, float min, float Kp, float Kd, float Ki );

 private:
 
};

#endif // FingerController
