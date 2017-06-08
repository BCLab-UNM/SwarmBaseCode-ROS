#include "PID.h"

PID::PID(PIDConfig config)
{
    this->config = config; 
    integralErrorHistArray.resize(config.integralErrorHistoryLength);
}

float PID::PIDOut(float calculatedError, float setPoint) {
    
    Error.pop_back();
    Error.insert(Error.begin(), calculatedError); //insert new error into vector for history purposes.
    
    float P = 0; //proportional yaw output 
    float I = 0; //Integral yaw output
    float D = 0; //Derivative yaw output
    
    if (setPoint != prevSetPoint && config.resetOnSetpoint) {
        for (int i = 0; i < config.errorHistLength; i++) {
            Error.at(i) = 0;
        }
        for (int i = 0; i < config.integralErrorHistoryLength; i++) {
            integralErrorHistArray.at(i) = 0;
        }
        prevSetPoint = setPoint;
        
    }
    
    
    //feed forward
    float FF = config.feedForwardMultiplier * setPoint;
    
    //error averager
    float avgError;
    bool checkExists = true;
    for (int i = 0; i < (config.errorHistLength); i++) {
        avgError += Error.at(i);
        if (Error.at(i) == 0) {
            checkExists = false;
            break;
        }
    }
    if (!checkExists) {
        avgError = calculatedError;
    }
    else {
        avgError /= config.errorHistLength;
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
    if (Error.front() > config.integralDeadZone || Error.front() < -config.integralDeadZone)
    {
        integralErrorHistArray.at(step) = Error.front(); //add error into the error Array.
        step++;
        
        if (step >= config.integralErrorHistoryLength) step = 0;
        
        if (!config.alwaysIntegral) {
            integralOn = true;
        }
    }
    
    float sum = 0;
    for (int i= 0; i < config.integralErrorHistoryLength; i++) //sum the array to get the error over time from t = 0 to present.
    {
        sum += integralErrorHistArray.at(i);
    }
    
    if (config.alwaysIntegral || integralOn){
        I = config.Ki * sum; //this is integrated output
    }
    
    //anti windup 
    //anti windup reduces overshoot by limiting the acting time of the integral to areas where the 
    //proportional term is less than half its saturation point.           
    
    //if P is already commanding greater than half max PWM dont use the integral       
    if (fabs(I) > config.integralMax || fabs(P) > config.antiWindup) //reset the integral to 0 if it hits its cap of half max PWM
    {
        for (int i = 0; i < config.integralErrorHistoryLength; i++) {
            integralErrorHistArray.at(i) = 0;
        }         
        I = 0;
    }
    
    //Derivative
    if (fabs(P) < config.antiWindup) 
    {
        //10 being the frequency of the system giving us a one second prediction base.
        //calculates the derivative of the error using average of last 2 error values for current error
        //and average of error 2 and 3 steps in the past as previouse error
        D = config.Kd * (avgError) * hz; 
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
   
    return PIDOut;
}
