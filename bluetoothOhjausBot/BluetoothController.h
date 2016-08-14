#ifndef BLUETOOTHCONTROLLER_H
#define BLUETOOTHCONTROLLER_H

#include <SoftwareSerial.h>
#include "/Users/Jarno/Arduino/bluetoothOhjausBot/MotorController.h"

class BluetoothController {
public:
	BluetoothController(MotorController* motorController);
	int getAndExecuteBluetoothCommand(int controlMode);
	void sendMessage(String message);
	
private:
	MotorController* motorController;

        SoftwareSerial* bluetooth; 

};
#endif
