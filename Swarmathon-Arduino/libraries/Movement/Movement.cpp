#include <Movement.h>

/**
 *	Constructor args are pins for left and right motor speed and direction
 **/
Movement::Movement(byte rightSpeedPin, byte rightDirectionA, byte rightDirectionB, byte leftSpeedPin, byte leftDirectionA, byte leftDirectionB) {
    pinMode(rightSpeedPin, OUTPUT);
    pinMode(rightDirectionA, OUTPUT);
    pinMode(rightDirectionB, OUTPUT);
    pinMode(leftSpeedPin, OUTPUT);
    pinMode(leftDirectionA, OUTPUT);
    pinMode(leftDirectionB, OUTPUT);
    _rightSpeedPin = rightSpeedPin;
    _rightDirectionA = rightDirectionA;
    _rightDirectionB = rightDirectionB;
    _leftSpeedPin = leftSpeedPin;
    _leftDirectionA = leftDirectionA;
    _leftDirectionB = leftDirectionB;
}

/**
 *	Sets forward motion at speed, args are for left and right tread speed
 **/
void Movement::forward(byte leftSpeed, byte rightSpeed) {
    digitalWrite(_leftDirectionA, LOW);
    digitalWrite(_leftDirectionB, HIGH);
    analogWrite(_leftSpeedPin, leftSpeed);
    digitalWrite(_rightDirectionA, HIGH);
    digitalWrite(_rightDirectionB, LOW);
    analogWrite(_rightSpeedPin, rightSpeed);
}

/**
 *	Sets backward motion at speed, args are for left and right tread speed
 **/
void Movement::backward(byte leftSpeed, byte rightSpeed) {
    digitalWrite(_leftDirectionA, HIGH);
    digitalWrite(_leftDirectionB, LOW);
    analogWrite(_leftSpeedPin, leftSpeed);
    digitalWrite(_rightDirectionA, LOW);
    digitalWrite(_rightDirectionB, HIGH);
    analogWrite(_rightSpeedPin, rightSpeed);
}

/**
 *	Rotates right at speed.  Treads will move in opposite directions
 **/
void Movement::rotateRight(byte leftSpeed, byte rightSpeed) {
    digitalWrite(_leftDirectionA, LOW);
    digitalWrite(_leftDirectionB, HIGH);
    analogWrite(_leftSpeedPin, leftSpeed);
    digitalWrite(_rightDirectionA, LOW);
    digitalWrite(_rightDirectionB, HIGH);
    analogWrite(_rightSpeedPin, rightSpeed);
}

/**
 *	Rotates left at speed.  Treads will move in opposite directions
 **/
void Movement::rotateLeft(byte leftSpeed, byte rightSpeed) {
    digitalWrite(_leftDirectionA, HIGH);
    digitalWrite(_leftDirectionB, LOW);
    analogWrite(_leftSpeedPin, leftSpeed);
    digitalWrite(_rightDirectionA, HIGH);
    digitalWrite(_rightDirectionB, LOW);
    analogWrite(_rightSpeedPin, rightSpeed);
}

/**
 *	Stops all motor motion
 **/
void Movement::stop() {
    digitalWrite(_leftDirectionA, LOW);
    digitalWrite(_leftDirectionB, LOW);
    digitalWrite(_rightDirectionA, LOW);
    digitalWrite(_rightDirectionB, LOW);
}
