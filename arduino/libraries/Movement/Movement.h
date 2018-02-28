#ifndef Movement_h
#define Movement_h

#include "Arduino.h"

class Movement
{
	public:
		//Constructors
		Movement(byte rightSpeedPin, byte rightDirectionA, byte rightDirectionB, byte leftSpeedPin, byte leftDirectionA, byte leftDirectionB);
		
		//Functions
		void forward(byte leftSpeed, byte rightSpeed);
		void backward(byte leftSpeed, byte rightSpeed);
		void rotateRight(byte leftSpeed, byte rightSpeed);
		void rotateLeft(byte leftSpeed, byte rightSpeed);
		void stop();
		
	private:
        byte _rightSpeedPin, _rightDirectionA, _rightDirectionB, _leftSpeedPin, _leftDirectionA, _leftDirectionB;
};

#endif
