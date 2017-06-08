#ifndef PID_H
#define PID_H

#include <vector>
#include <ros/ros.h>

using namespace std;

struct PIDConfig {
    float Kp,Ki,Kd = 0;
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
};

class PID
{
public:
    


    PIDConfig config;
    
    PID(PIDConfig config);
    
    float PIDOut(float calculatedError, float setPoint);
    
private:
    
    vector<float> Error;
    float prevSetPoint;
    vector<float> integralErrorHistArray;
    int step = 0;
    float hz = 10;
    
};

#endif // PID_H
