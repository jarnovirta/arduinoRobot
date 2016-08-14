// BUDGETCACTUS Robot Code. 

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

#define MAIN_08_ADDRESS (0xF8) // CUSTOM SRF08 I2C ADDRESS!
#define GAIN_REGISTER 0x09
#define LOCATION_REGISTER 0x8C

int controlMode;  // 1=Bluetooth control; 2=Avoid obstacles 

MotorController* motorController; 
AvoidObstacleController* avoidObstacleController;
SonarSRF08 mainSonar;
ServoController* servoController;
BluetoothController* bluetoothController;
unsigned long lastStatusInfo = millis();

char unit = 'c'; // 'i' for inches, 'c' for centimeters, 'm' for micro-seconds

void setup() 
{ 
      // Serial.begin(9600);  // Begin the serial monitor at 9600bps
      // Serial.println("*** BUDGETCACTUS setup... ***");
      mainSonar.connect(MAIN_08_ADDRESS, GAIN_REGISTER, LOCATION_REGISTER); // Start Sonar Range Finder:
       controlMode = 1; 
      motorController = new MotorController();
      servoController = new ServoController();
      avoidObstacleController = new AvoidObstacleController(motorController, servoController, &mainSonar, bluetoothController);
      bluetoothController = new BluetoothController(motorController);
      
      // Begin bluetooth connection on hardware serial pins 1 (RX) and 2 (TX)
      
      Serial.begin(115200);  
      
      // Serial.println("\n*** BUDGETCACTUS setup complete ***");
}  

void loop() 
{ 

  unsigned long time = millis();
  if (time - lastStatusInfo > 1000) {
       lastStatusInfo = time;
       // bluetoothController->sendMessage("BUDGETCACTUS live. Free memory=");

    }

// CHECK FOR BLUETOOTH COMMANDS
    int oldControlMode = controlMode;
    controlMode = bluetoothController->getAndExecuteBluetoothCommand(controlMode);
    if (controlMode != oldControlMode) {
          switch(controlMode) {
              case 1:
                motorController->stopRobot();
                servoController->setPanAndTilt(90, 90);
            }
      }
//  OBSTACLE AVOIDANCE MODE

  if (controlMode == 2) {
         avoidObstacleController->moveRobot();  
      }
}
