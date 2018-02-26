#ifndef Odometry_h
#define Odometry_h

#include "Arduino.h"

class Odometry {
public:
    //Constructors
    Odometry(byte rightEncoderAPin, byte rightEncoderBPin, byte leftEncoderAPin, byte leftEncoderBPin, float wheelBase, float wheelDiameter, int cpr);
    
    //Functions
    void update();
    
    //Variables
    float x, y, theta;
    float vx, vy, vtheta;
    long clock;

private:
    //Variables
    float _wheelBase, _wheelDiameter;
    int _cpr;
};

#endif