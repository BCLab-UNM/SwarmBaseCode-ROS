#include "PIDController.h"
#include <iostream>

using namespace std;

/**
 * The default constructor; by design, if a PIDController is not instantiated
 * with a non-default constructor or initialized via setPIDControllerSettings()
 * then "isInitialized" will be false and an error should occur.
 */
PIDController::PIDController() {
  isInitialized = false;
}

/**
 * Constructor; this function initializes most values using the provided
 * PIDSettings structure and all others are set to default starting values.
 *
 * @param settings A PIDSettings struct defining six variables: dt, max, min,
 *                 Kp, Kd, and Ki.
 */
PIDController::PIDController(PIDController::PIDSettings settings) {
  // fatal error check; if the dt value is invalid, exit and/or crash the sim
  // especially if dt = 0, which would cause division by zero errors in the
  // update() function
  if(settings.dt <= 0.0) {
    ROS_ERROR_STREAM(
      "[Gripper Plugin]: In PIDController.cpp: PIDController(PIDSettings): "
      << "dt = " << settings.dt << ", dt cannot be <= 0.0"
    );
    exit(1);
  }

  isInitialized = true;
  dt  = settings.dt;
  max = settings.max;
  min = settings.min;
  Kp = settings.Kp;
  Kd = settings.Kd;
  Ki = settings.Ki;
  previousError = 0;
  integral = 0;
}

/**
 * This function calculates the amount of force in Newtons to apply to a joint.
 * The farther away a joint is from its desired position, the more force that
 * will be applied. The sign of the value returned from this function matters
 * as it will determine the direction of the applied force. The amount of force
 * issued is limited by the "min" and "max" variables defined in this class.
 *
 * @param setPoint     The desired value we are trying to achieve and maintain.
 * @pram  currentValue The current value.
 * @return The amount of force in Newtons to be applied to a joint to move
 *         closer to the setPoint from the currentValue.
 */
float PIDController::update(float setPoint, float currentValue) {
  if(isInitialized == false) {
    ROS_ERROR_STREAM(
      "[Gripper Plugin]: In PIDController.cpp: update(): PIDController "
      << "was not properly initialized in the GripperManager class!"
    );
    exit(1);
  }

  // Calculate the error
  float error = setPoint - currentValue;
  
  // Calculate the proportional term: the amount to adjust according to the 
  // difference between the desired value and the actual value.
  float proportionalTerm = Kp*error;
  
  // Calculate the derivative term: the amount to adjust according to the rate
  // at which the difference between the desired value and the actual value is
  // changing.
  float derivativeTerm = Kd*(error - previousError)/dt;
  
  // Calculate the integral term: the amount to adjust based on the total error
  // seen so far.
  integral += (error*dt);
  float integralTerm = Ki*integral;

  // Sum all of the terms.
  float output = proportionalTerm + derivativeTerm + integralTerm;

  // Check the bounds for the maximum and minimum forces.
  if(output > max) {
    output = max;
  } else if(output < min) {
    output = min;
  }

  // Record the error for the derivative calculation.
  previousError = error;

  return output;
}
