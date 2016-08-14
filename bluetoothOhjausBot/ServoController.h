#ifndef SERVOCONTROLLER_H
#define SERVOCONTROLLER_H

#include "Arduino.h"
#include <Servo.h>

class ServoController {
public:
	ServoController();
	void setPanAndTilt(int pan, int tilt);
private:
	unsigned long sonarPanLastMovement = millis();
	Servo panServo;  // create servo object to control a servo 
	                  // a maximum of eight servo objects can be created 
	Servo tiltServo;
};
#endif
