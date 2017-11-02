#include "PID.h"

PID::PID() {   }

PID::PID(PIDConfig config)
{
  this->config = config;
  integralErrorHistArray.resize(config.integralErrorHistoryLength, 0.0);
}

float PID::PIDOut(float calculatedError, float setPoint)
{

  if (Error.size() >= config.errorHistLength)
  {
    Error.pop_back();
  }
  Error.insert(Error.begin(), calculatedError); //insert new error into vector for history purposes.

  float P = 0; //proportional yaw output
  float I = 0; //Integral yaw output
  float D = 0; //Derivative yaw output

  if (setPoint != prevSetPoint && config.resetOnSetpoint)
  {
    Error.clear();
    integralErrorHistArray.clear();
    prevSetPoint = setPoint;
    step = 0;
    integralErrorHistArray.resize(config.integralErrorHistoryLength, 0.0);
  }

  //feed forward
  float FF = config.feedForwardMultiplier * setPoint;

  //error averager
  float avgError = 0;
  if (Error.size() >= config.errorHistLength) {
    for (int i = 0; i < config.errorHistLength; i++) {
      avgError += Error[i];
    }
    avgError /= config.errorHistLength;//config.errorHistLength;
  }
  else {
    avgError = calculatedError;
  }


  // ----- BEGIN PID CONTROLLER CODE -----


  //GenPID ------------


  //Proportional
  P = config.Kp * (avgError);  //this is the proportional output
  if (P > config.satUpper) //limit the max and minimum output of proportional
    P = config.satUpper;
  if (P < config.satLower)
    P = config.satLower;
  //Integral
  bool integralOn = false;


  //only use integral when error is larger than presumed noise.
  if (fabs(Error.front()) > config.integralDeadZone)
  {
    integralErrorHistArray[step] = Error.front(); //add error into the error Array.
    step++;

    if (step >= config.integralErrorHistoryLength) step = 0;
    if (!config.alwaysIntegral) {
      integralOn = true;
    }
  }


  float sum = 0;
  for (int i= 0; i < integralErrorHistArray.size(); i++) //sum the array to get the error over time from t = 0 to present.
  {
    sum += integralErrorHistArray[i];
  }

  if (config.alwaysIntegral || integralOn){
    I = config.Ki * sum; //this is integrated output
  }
  else {
    integralErrorHistArray.clear();
    integralErrorHistArray.resize(config.integralErrorHistoryLength, 0.0);
    I = 0;
    step = 0;
  }

  //anti windup
  //anti windup reduces overshoot by limiting the acting time of the integral to areas where the
  //proportional term is less than half its saturation point.

  //if P is already commanding greater than half max PWM dont use the integral
  if (fabs(I) > config.integralMax || fabs(P) > config.antiWindup) //reset the integral to 0 if it hits its cap of half max PWM
  {
    integralErrorHistArray.clear();
    integralErrorHistArray.resize(config.integralErrorHistoryLength, 0.0);
    I = 0;
    step = 0;
  }

  //Derivative
  if (fabs(P) < config.antiWindup)
  {
    float avgPrevError = 0;
    for (int i = 1; i < Error.size(); i++)
    {
      avgPrevError += Error[i];
    }
    if (Error.size() > 1)
    {
      avgPrevError /= Error.size()-1;
    }
    else
    {
      avgPrevError = Error[0];
    }

    D = config.Kd * (Error[0] - Error[1]) * hz;
  }

  float PIDOut = P + I + D + FF;

  if (PIDOut > config.satUpper) //cap vel command
  {
    PIDOut = config.satUpper;
  }
  else if (PIDOut < config.satLower)
  {
    PIDOut = config.satLower;
  }

  cout << "PID OUTPUT:  " << PIDOut << endl;

  return PIDOut;
}
