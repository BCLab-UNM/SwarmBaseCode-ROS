#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <ros/console.h>

/**
 * This class defines the base PID controller to be used with all of the joints
 * in the GripperPlugin.
 *
 * @author Matthew Fricke
 * @author Antonio Griego
 */
class PIDController {

  public:

    // this struct defines all of the key variables for the PID controller and
    // simplifies passing around the entire series of values between functions
    struct PIDSettings {
      float Kp;
      float Ki;
      float Kd;
      float dt;
      float max;
      float min;
    };

    // constructors
    PIDController();
    PIDController(PIDController::PIDSettings settings);

    // returns a force to apply based on the setPoint vs. currentValue
    float update(float setPoint, float currentValue);

  private:

    float dt;  // Update time
    float max; // Max setpoint
    float min; // Min setpoint
    float Kp;  // Proportional gain
    float Kd;  // Derivative gain
    float Ki;  // Integral gain

    float previousError; // Previous error (starts at 0.0)
    float integral;      // Sum of the error seen so far
    bool  isInitialized; // PID controller status flag

};

#endif /* PID_CONTROLLER_H */
