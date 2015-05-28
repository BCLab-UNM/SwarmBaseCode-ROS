/**
 *	Library for Utility functions related to AntBot project
 **/

#ifndef Utilities_h
#define Utilities_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

class Utilities
{
public:
    //Constructor
    Utilities();
    
    //Structs
    struct Cartesian {
        Cartesian(float a, float b):x(a),y(b){}
        float x,y;
    };
    struct Polar {
        Polar(float a, float b):r(a),theta(b){}
        float r,theta;
    };
    struct EvolvedParameters {
        EvolvedParameters(float tGUP, float sGUP,
                          float uSC, float iSC, float sSV,
                          float sFR):
        travelGiveUpProbability(tGUP), searchGiveUpProbability(sGUP),
        uninformedSearchCorrelation(uSC), informedSearchCorrelationDecayRate(iSC), stepSizeVariation(sSV),
        siteFidelityRate(sFR) {}
        float travelGiveUpProbability;
        float searchGiveUpProbability;
        float uninformedSearchCorrelation;
        float informedSearchCorrelationDecayRate;
        float stepSizeVariation;
        float siteFidelityRate;
    };
    
    //Functions
    float angle(float start_angle, float end_angle);
    float deg2rad(float degree);
    float rad2deg(float radian);
    Utilities::Cartesian pol2cart(Utilities::Polar pol);
    Utilities::Polar cart2pol(Utilities::Cartesian cart);
    float pmod(float dividend, float divisor);
    float poissonCDF(float k, float lambda);
    float exponentialDecay(float quantity, float time, float lambda);
    
    //Legacy Functions
    //byte* parseIP(char* address);
    //int availableMemory();
    //Utilities::Polar gps2pol(float lat1, float lon1, float lat2, float lon2);
    //void onLED(byte pinLED);
    //void offLED(byte pinLED);
    
    //Timer functions
    void tic();
    void tic(long timerLength);
    long toc();
    bool isTime();
    
private:
    //Timer variables
    unsigned long timerStart;
    unsigned long timerStop;
};

#endif