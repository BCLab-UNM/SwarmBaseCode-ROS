#include <Odometry.h>

//Global Functions
void rightEncoderAChange();
void setupPinChangeInterrupt(byte pin);
void leftEncoderAChange();
void leftEncoderBChange();

//Global Variables
int rightEncoderCounter;
int leftEncoderCounter;
byte _rightEncoderAPin;
byte _rightEncoderBPin;
byte _leftEncoderAPin;
byte _leftEncoderBPin;

/**
 *	Constructor arg are pins for channels A and B on right and left encoders
 **/
Odometry::Odometry(byte rightEncoderAPin, byte rightEncoderBPin, byte leftEncoderAPin, byte leftEncoderBPin, float wheelBase, float wheelDiameter, int cpr) {
    pinMode(rightEncoderAPin, INPUT);
    pinMode(rightEncoderBPin, INPUT);
    pinMode(leftEncoderAPin, INPUT);
    pinMode(leftEncoderBPin, INPUT);
    digitalWrite(leftEncoderBPin, HIGH);
    rightEncoderCounter = 0.;
    leftEncoderCounter = 0.;
    attachInterrupt(digitalPinToInterrupt(rightEncoderAPin), rightEncoderAChange, CHANGE);
    setupPinChangeInterrupt(rightEncoderBPin);
    attachInterrupt(digitalPinToInterrupt(leftEncoderAPin), leftEncoderAChange, CHANGE);
    attachInterrupt(digitalPinToInterrupt(leftEncoderBPin), leftEncoderBChange, CHANGE);
    _rightEncoderAPin = rightEncoderAPin;
    _rightEncoderBPin = rightEncoderBPin;
    _leftEncoderAPin = leftEncoderAPin;
    _leftEncoderBPin = leftEncoderBPin;
    _wheelBase = wheelBase;
    _wheelDiameter = wheelDiameter;
    _cpr = cpr;
    
    theta = 0;
    clock = millis();
}

void Odometry::update() {
    //Calculate linear distance that each wheel has traveled
    float rightWheelDistance = ((float)rightEncoderCounter / _cpr) * _wheelDiameter * PI;
    float leftWheelDistance = ((float)leftEncoderCounter / _cpr) * _wheelDiameter * PI;
    
    //Calculate relative angle that robot has turned
    float dtheta = (rightWheelDistance - leftWheelDistance) / _wheelBase;
    //Calculate angular velocity
    vtheta = dtheta / (millis() - clock) * 1000;
    //Accumulate angles to calculate absolute heading
    theta += dtheta;
    
    //Decompose linear distance into its component values
    float meanWheelDistance = (rightWheelDistance + leftWheelDistance) / 2;
    x = meanWheelDistance * cos(dtheta);
    y = meanWheelDistance * sin(dtheta);
    //Calculate linear velocity
    vx = x / (millis() - clock) * 1000;
    vy = y / (millis() - clock) * 1000;
    
    //Reset counters
    rightEncoderCounter = 0;
    leftEncoderCounter = 0;
    
    //Reset clock
    clock = millis();
}

void rightEncoderAChange() {
    bool rightEncoderAStatus = digitalRead(_rightEncoderAPin);
    bool rightEncoderBStatus = digitalRead(_rightEncoderBPin);
    if (((rightEncoderAStatus == HIGH) && (rightEncoderBStatus == LOW)) || ((rightEncoderAStatus == LOW) && (rightEncoderBStatus == HIGH))) {
        rightEncoderCounter++;
    }
    else {
        rightEncoderCounter--;
    }
}

ISR (PCINT0_vect) {
    bool rightEncoderAStatus = digitalRead(_rightEncoderAPin);
    bool rightEncoderBStatus = digitalRead(_rightEncoderBPin);
    if (((rightEncoderAStatus == HIGH) && (rightEncoderBStatus == HIGH)) || ((rightEncoderAStatus == LOW) && (rightEncoderBStatus == LOW))) {
        rightEncoderCounter++;
    }
    else {
        rightEncoderCounter--;
    }
}

void leftEncoderAChange() {
    bool leftEncoderAStatus = digitalRead(_leftEncoderAPin);
    bool leftEncoderBStatus = digitalRead(_leftEncoderBPin);
    if (((leftEncoderAStatus == HIGH) && (leftEncoderBStatus == HIGH)) || ((leftEncoderAStatus == LOW) && (leftEncoderBStatus == LOW))) {
        leftEncoderCounter++;
    }
    else {
        leftEncoderCounter--;
    }
}

void leftEncoderBChange() {
    bool leftEncoderAStatus = digitalRead(_leftEncoderAPin);
    bool leftEncoderBStatus = digitalRead(_leftEncoderBPin);
    if (((leftEncoderAStatus == HIGH) && (leftEncoderBStatus == LOW)) || ((leftEncoderAStatus == LOW) && (leftEncoderBStatus == HIGH))) {
        leftEncoderCounter++;
    }
    else {
        leftEncoderCounter--;
    }
}

void setupPinChangeInterrupt(byte pin) {
    *digitalPinToPCMSK(pin) |= bit (digitalPinToPCMSKbit(pin));  // enable pin
    PCIFR  |= bit (digitalPinToPCICRbit(pin)); // clear any outstanding interrupt
    PCICR  |= bit (digitalPinToPCICRbit(pin)); // enable interrupt for the group
}