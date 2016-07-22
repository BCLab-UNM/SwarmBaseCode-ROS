#ifndef WristController_h
#define WristController_h

#include "PIDController.h"

class WristController : public PIDController {

 public:
  WristController();
  WristController( float dt, float max, float min, float Kp, float Kd, float Ki );

 private:
  
};

#endif // WristController_h
