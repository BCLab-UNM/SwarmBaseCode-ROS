#include "PIDController.h"
#include <iostream>

using namespace std;

PIDController::PIDController() {
}

PIDController::PIDController(float _dt, float _max, float _min, float _Kp, float _Kd, float _Ki):
  dt(_dt),
  max(_max),
  min(_min),
  Kp(_Kp),
  Kd(_Kd),
  Ki(_Ki),
  previousError(0),
  integral(0) {
}

// setPoint - desired value we are trying to achive and maintain.
// currentValue - the current value.
float PIDController::update( float setPoint, float currentValue ) {
  // Calculate the error
  float error = setPoint - currentValue;
  
  //cout << "[Gripper Plugin] Gains: Kp = " << Kp << ", Kd = " << Kd << ", Ki = " << Ki << endl;

  //cout << "[Gripper Plugin] PID Controller: Error = " << error << endl;

  // Calculate the proportional term - amount to adjust according to the 
  // difference between the desired value and the actual value.
  float prop = Kp*error;
  
  //cout << "[Gripper Plugin] PID Controller: Proportional Term = " << prop << endl;

  // Derivative term - amount to adjust according to the rate at which 
  // the difference between the desired value and the actual value is changing.
  float deriv = Kd*(error - previousError)/dt;
  
  //cout << "[Gripper Plugin] PID Controller: Derivative Term = " << deriv << endl;

  // Integral term - amount to adjust based on the total error seen so far.
  float integ = Ki*(integral += error*dt);

  //cout << "[Gripper Plugin] PID Controller: Integral Term = " << integ << endl;

  // Sum the terms
  float output = prop+deriv+integ;

  // Check value bounds
  if ( output > max) output = max;
  else if ( output < min ) output = min;

  // Record the error for derivative calc.
  previousError = error;

  //cout << "[Gripper Plugin] PID Controller: Output = " << output << endl;

  return output;
}

PIDController::~PIDController(){}
