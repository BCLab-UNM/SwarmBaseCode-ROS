#include "FingerController.h"

FingerController::FingerController() {
  dt = 1.0;
  max = 10.0;
  min = -10.0;
  previousError = 0;
  integral = 0;
  Kp = 2.5;
  Kd = 0;
  Ki = 0;
}

FingerController::FingerController(float _dt, float _max, float _min, float _Kp, float _Kd, float _Ki){
  dt = _dt;
  max = _max;
  min = _min;
  previousError = 0;
  integral = 0;
  Kp = 0;
  Kd = 0;
  Ki = 0;
}
