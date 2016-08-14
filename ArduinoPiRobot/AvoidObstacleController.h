#ifndef AVOIDOBSTACLECONTROLLER_H
#define AVOIDOBSTACLECONTROLLER_H

#include <SonarSRF08.h>
#include "/Users/Jarno/Arduino/bluetoothOhjausBot/ServoController.h"
#include "/Users/Jarno/Arduino/bluetoothOhjausBot/MotorController.h"
#include "/Users/Jarno/Arduino/bluetoothOhjausBot/BluetoothController.h"

class AvoidObstacleController {
public:
        AvoidObstacleController(MotorController* motorController, ServoController* servoController, SonarSRF08* MainSonar, BluetoothController*);
	void moveRobot();

private:
	int tiltServoAngle = 90;
	int panServoAngle = 90;
	int previousPanServoAngle = 140;
	unsigned long time;
        unsigned long sonarPanLastMovement;
        MotorController* motorController;
        ServoController* servoController;
        BluetoothController* bluetoothController;
        SonarSRF08* mainSonar;
        char unit = 'c'; // 'i' for inches, 'c' for centimeters, 'm' for micro-seconds
};

#endif
