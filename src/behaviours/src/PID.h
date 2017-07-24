#ifndef PID_H
#define PID_H

#include <vector>
#include <ros/ros.h>

using namespace std;

struct PIDConfig {
  float Kp = 0;
  float Ki = 0;
  float Kd = 0;
  float satUpper = 255;
  float satLower = -255;
  float antiWindup = satUpper/2;
  int errorHistLength = 4;
  bool alwaysIntegral = false;
  bool resetOnSetpoint = true;
  float feedForwardMultiplier = 0;
  float integralDeadZone = 0.01;
  float integralErrorHistoryLength = 10000;
  float integralMax = 255/2;
  float derivativeAlpha = 0.7;
};

class PID
{
public:



  PIDConfig config;

  PID();
  PID(PIDConfig config);

  float PIDOut(float calculatedError, float setPoint);

  void SetConfiguration(PIDConfig config) {this->config = config;}

private:

  vector<float> Error;
  float prevSetPoint = std::numeric_limits<float>::min();
  vector<float> integralErrorHistArray;
  int step = 0;
  float hz = 10;

};

#endif // PID_H
