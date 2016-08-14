// BUDGETCACTUS Robot Code. 

#define MAIN_08_ADDRESS (0x70)
#define GAIN_REGISTER 0x09
#define LOCATION_REGISTER 0x8C

#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include <Servo.h>
#include <SonarSRF08.h>
#include <SoftwareSerial.h>
#include <MemoryFree.h>
#include "utility/Adafruit_PWMServoDriver.h"
#include "/Users/Jarno/Arduino/bluetoothOhjausBot/MotorController.h"
#include "/Users/Jarno/Arduino/bluetoothOhjausBot/AvoidObstacleController.h"
#include "/Users/Jarno/Arduino/bluetoothOhjausBot/BluetoothController.h"

int controlMode;  // 1=Bluetooth control; 2=Avoid obstacles 

MotorController* motorController; 
AvoidObstacleController* avoidObstacleController;
SonarSRF08* mainSonar;
ServoController* servoController;
BluetoothController* bluetoothController;
unsigned long lastStatusInfo = millis();

void setup() 
{ 
      Serial.begin(9600);  // Begin the serial monitor at 9600bps
      Serial.println("*** BUDGETCACTUS setup... ***");

      controlMode = 1; 
      motorController = new MotorController();
      servoController = new ServoController();
      avoidObstacleController = new AvoidObstacleController(motorController, servoController, mainSonar);
      bluetoothController = new BluetoothController(motorController);
      
      mainSonar = new SonarSRF08();
      mainSonar->connect(MAIN_08_ADDRESS, GAIN_REGISTER, LOCATION_REGISTER); // Start Sonar Range Finder:
      
      Serial.println("\n*** BUDGETCACTUS setup complete ***");
}  

void loop() 
{ 
  unsigned long time = millis();
  if (time - lastStatusInfo > 1000) {
       lastStatusInfo = time;
       Serial.print("BUDGETCACTUS live. Free memory=");
       Serial.println(freeMemory());
    }

// CHECK FOR BLUETOOTH COMMANDS

    controlMode = bluetoothController->getAndExecuteBluetoothCommand(controlMode);

//  OBSTACLE AVOIDANCE MODE

  if (controlMode == 2) {
         avoidObstacleController->moveRobot();  
      }
}
