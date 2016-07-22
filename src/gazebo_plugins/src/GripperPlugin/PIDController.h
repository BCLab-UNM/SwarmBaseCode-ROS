#ifndef PIDController_h
#define PIDController_h

class PIDController {
 public:
  PIDController();
  PIDController( float dt, float max, float min, float Kp, float Kd, float Ki );
  ~PIDController();
  float update( float setPoint, float currentValue );
  
  void setProportionalGain( float );
  void setDerivativeGain( float );
  void setIntegralGain( float );
  
  float getProportionalGain();
  float getDerivativeGain();
  float getIntegralGain();
  
 protected:
  float dt; // Update time
  float max; // Max setpoint
  float min; // Min setpoint
  float Kp; // Proportional gain
  float Kd; // Derivative gain
  float Ki; // Integral gain
  float previousError; // Previous error
  float integral; // Sum of error seen so far
};

#endif // PIDController_h
