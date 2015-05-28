/**
*	Library for Utility functions related to AntBot project
**/

#include <Utilities.h>

//Constructor
Utilities::Utilities(){}

/**
*	Calculates angle between two compass headings in degrees
*	Return value is bounded between -180 and 180
**/
float Utilities::angle(float start_angle, float end_angle)
{
	float angle = rad2deg(atan2(pol2cart(Polar(1,end_angle)).y,pol2cart(Polar(1,end_angle)).x) - 
					atan2(pol2cart(Polar(1,start_angle)).y,pol2cart(Polar(1,start_angle)).x));
	if (angle > 180) return(angle - 360);
	else if (angle < -180) return(angle + 360);
	else return(angle);
}

/**
*	Converts degrees to radians
**/
float Utilities::deg2rad(float degree)
{
	return (degree * (PI/180));
}

/**
*	Converts radians to degrees
**/
float Utilities::rad2deg(float radian)
{
	return (radian * (180/PI));
}

/**
*	Converts polar to cartesian coordinates
**/
Utilities::Cartesian Utilities::pol2cart(Utilities::Polar pol)
{
	if ((pol.theta == 90.0) || (pol.theta == 270.0))
		return (Cartesian(0,pol.r*sin(deg2rad(pol.theta))));
	else if ((pol.theta == 90.0) || (pol.theta == 270.0))
		return (Cartesian(pol.r*cos(deg2rad(pol.theta)),0));
	else
		return (Cartesian(pol.r*cos(deg2rad(pol.theta)),pol.r*sin(deg2rad(pol.theta))));
}

/**
*	Converts cartesian to polar coordinates
**/
Utilities::Polar Utilities::cart2pol(Utilities::Cartesian cart)
{
	if (cart.y < 0)
	{
		if (cart.x < 0) return Polar(sqrt(sq(cart.x)+sq(cart.y)),rad2deg(atan(cart.y/cart.x))+180);
		else if (cart.x > 0) return Polar(sqrt(sq(cart.x)+sq(cart.y)),rad2deg(atan(cart.y/cart.x))+360);
		else return Polar(sqrt(sq(cart.x)+sq(cart.y)),270);
	}
	else if (cart.y > 0)
	{
		if (cart.x < 0) return Polar(sqrt(sq(cart.x)+sq(cart.y)),rad2deg(atan(cart.y/cart.x))+180);
		else if (cart.x > 0) return Polar(sqrt(sq(cart.x)+sq(cart.y)),rad2deg(atan(cart.y/cart.x)));
		else return Polar(sqrt(sq(cart.x)+sq(cart.y)),90);
	}
	else
	{
		if (cart.x < 0) return Polar(sqrt(sq(cart.x)+sq(cart.y)),180);
		else return Polar(sqrt(sq(cart.x)+sq(cart.y)),0);
	}
}

/**
*	Mod function that behaves as in MATLAB, i.e. returns only positive results
**/
float Utilities::pmod(float dividend, float divisor)
{
	float temp = fmod(dividend,divisor);
	while (temp < 0) temp += divisor;
	return temp;
}

/**
*	Starts timer, returns id number associated with specific timer, takes optional stop time
*	This behaves exactly as in MATLAB, except for the optional stop time extension
**/
void Utilities::tic()
{
	timerStart = millis();
}

/**
*	Overloaded version of tic() above, receives stop time as input
**/
void Utilities::tic(long timerLength)
{
	timerStart = millis();
	timerStop = timerStart + timerLength;
}

/**
*	Returns amount of time passed (in milliseconds) since tic() was executed
*	Identical to MATLAB version
**/
long Utilities::toc()
{
	return millis()-timerStart;
}


/**
*	Returns true if timer has expired, false otherwise
*	If length of timer has not been set, return false
**/
bool Utilities::isTime()
{
	if ((timerStop == 0) || (millis() < timerStop)) {
		return false;
	}
	else {
		return true;
	}
}

/*
 * Returns Poisson cumulative probability at a given k and lambda
 */
float Utilities::poissonCDF(float k, float lambda) {
    float sumAccumulator = 1;
    float factorialAccumulator = 1;
    
    for (int i = 1; i <= floor(k); i++) {
        factorialAccumulator *= i;
        sumAccumulator += pow(lambda, i) / factorialAccumulator;
    }
    
    return (exp(-lambda) * sumAccumulator);
}

/**
 *   Implements exponential decay function
 *   Returns decay of quantity at time given rate of change lambda
 **/
float Utilities::exponentialDecay(float quantity, float time, float lambda) {
    return (quantity * exp(-lambda*time));
}

///////////////////
////LEGACY CODE////
///////////////////

// /**	
// *	Takes IP address in string form, tokenizes it using a user-selected delimiter,
// *	then converts the string tokens to ints
// *	Returns IP address as 4-element byte array
// **/
// byte* Utilities::parseIP(char* address)
// {
// 	//Variables
// 	byte* ip = (byte*)calloc(4,sizeof(int));
// 	char* pch;
// 	char* copy = (char*)calloc(strlen(address)+1,sizeof(int));
// 	
// 	//Make copy of address to avoid damaging original
// 	strcpy(copy,address);
// 	
// 	//Split address using "." as the delimiter
// 	pch = strtok(copy,".");
// 	
// 	//Convert string to int to produce IP in byte form
// 	for (byte i=0; i<4; i++)
// 	{
// 		ip[i] = atoi(pch);
// 		pch = strtok(NULL,".");
// 	}
// 	
// 	//Free memory
// 	free(copy);
// 	
// 	return ip;
// }
// 
// /**
// *	Calculates amount of free memory available
// **/
// int Utilities::availableMemory()
// {
// 	int size = 1024;
// 	byte *buf;
// 	
// 	while ((buf = (byte *)malloc(--size)) == NULL){}
// 	
// 	free(buf);
// 	return size;
// }
// 
// /** 
// * Converts GPS lat/lon pair to polar coordinates (r,theta)
// **/
// Utilities::Polar Utilities::gps2pol(float start_lat, float start_lon, float end_lat, float end_lon)
// {
// 	float latSq = pow((end_lat-start_lat),2);
// 	float lonSq = pow((end_lon-start_lon),2);
// 	
// 	float deltPhi = log(tan(end_lat/2+M_PI/4)/tan(start_lat/2+M_PI/4));
// 	if (start_lat == end_lat) deltPhi = 0;
// 	float deltLon = start_lon - end_lon;
// 	
// 	return Utilities::Polar(rad2deg(sqrt(latSq+lonSq))*1852,rad2deg(atan2(deltLon, deltPhi))+180);	
// }
// 
// /**
// *	Turns LED at specified pin number on
// **/
// void Utilities::onLED(byte pinLED) 
// {
// 	digitalWrite(pinLED,HIGH);
// }
// 
// /**
// *	Turns LED at specified pin number off
// **/
// void Utilities::offLED(byte pinLED)
// {
// 	digitalWrite(pinLED,LOW);
// }