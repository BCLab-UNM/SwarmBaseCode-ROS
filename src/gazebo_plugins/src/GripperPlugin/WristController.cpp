#include "WristController.h"

WristController::WristController() {
  dt = 1.0;
  max = 1.0;
  min = -1.0;
  Kp = 2.5;
  Kd = 0;
  Ki = 0;
  previousError = 0;
  integral = 0;
}

WristController::WristController(float _dt, float _max, float _min, float _Kp, float _Kd, float _Ki) {
  dt = _dt;
  max = _max;
  min = _min;
  Kp = _Kp;
  Kd = _Kd;
  Ki = _Ki;
  previousError = 0;
  integral = 0;
  Kp = 2.5;
}
