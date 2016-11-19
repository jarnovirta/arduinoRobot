// BUDGETCACTUS Robot Code. 

#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include <Servo.h>
#include <SonarSRF08.h>
#include <SoftwareSerial.h>
#include <MemoryFree.h>
#include "utility/Adafruit_PWMServoDriver.h"
#include "/Users/Jarno/Arduino/ArduinoPiRobot/MotorController.h"
#include "/Users/Jarno/Arduino/ArduinoPiRobot/AvoidObstacleController.h"
#include "/Users/Jarno/Arduino/ArduinoPiRobot/BluetoothController.h"

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

SoftwareSerial *raspberry;

const int CONTROL_MODE_LED_BLUETOOTH_PIN = 4;
const int CONTROL_MODE_LED_OBSTACLE_AVOIDANCE_PIN = 5;
const int CONTROL_MODE_LED_FIND_OBJECT_PIN = 6;
const int RGB_LED_GREEN_PIN = 7;
const int RGB_LED_BLUE_PIN = 8;
const int RGB_LED_RED_PIN = 11;

int controlModeLedPins[] = { CONTROL_MODE_LED_BLUETOOTH_PIN, CONTROL_MODE_LED_OBSTACLE_AVOIDANCE_PIN, CONTROL_MODE_LED_FIND_OBJECT_PIN };
char unit = 'c'; // 'i' for inches, 'c' for centimeters, 'm' for micro-seconds

const boolean DEBUG = false;

void setup() 
{ 
      raspberry = new SoftwareSerial(13, 12);
      raspberry->begin(89600);
      if (DEBUG) {
        Serial.begin(9600);  // Begin the serial monitor at 9600bps
        Serial.println("*** BUDGETCACTUS setup... ***");
      }
      setupControlModeLedPins();
      mainSonar.connect(MAIN_08_ADDRESS, GAIN_REGISTER, LOCATION_REGISTER); // Start Sonar Range Finder:
      controlMode = 0; 
      motorController = new MotorController();
      servoController = new ServoController();
      avoidObstacleController = new AvoidObstacleController(motorController, servoController, &mainSonar, bluetoothController);
      bluetoothController = new BluetoothController(motorController);
      
      if (DEBUG) {
        Serial.println("\n*** BUDGETCACTUS setup complete ***");
        }
}  
boolean vasen = true;
void loop() 
{ 
  
 while(true) {
      raspberry->println("A");
      Serial.println("A sent");
      delay(1000);
      if (raspberry->available()) {
          raspberry->write(raspberry.read());
          
        }
    }
  
  unsigned long time = millis();
  if (DEBUG && time - lastStatusInfo > 1000) {
      lastStatusInfo = time;
      Serial.print("BUDGETCACTUS live - free memory: ");
      Serial.println(freeMemory());
    }
  
  
// CHECK FOR BLUETOOTH COMMANDS
    int oldControlMode = controlMode;
    controlMode = bluetoothController->getAndExecuteBluetoothCommand(controlMode);
    if (controlMode != oldControlMode) {
          switch(controlMode) {
              case 0:
                motorController->stopRobot();
                servoController->setPanAndTilt(90, 90);
              case 1:
                servoController->setPanAndTilt(90, 100);
            }
          setControlModeLeds();
      }
//  OBSTACLE AVOIDANCE MODE

  if (controlMode == 1) {
         avoidObstacleController->moveRobot();  
      }
}

void setControlModeLeds() {
    for (int i = 0; i < 2; i++) {
        if (i == controlMode) digitalWrite(controlModeLedPins[i], HIGH);
        else digitalWrite(controlModeLedPins[i], LOW);
      }
  }
void setupControlModeLedPins() {
    pinMode(CONTROL_MODE_LED_BLUETOOTH_PIN, OUTPUT);
    pinMode(CONTROL_MODE_LED_OBSTACLE_AVOIDANCE_PIN, OUTPUT);
    pinMode(CONTROL_MODE_LED_FIND_OBJECT_PIN, OUTPUT);
    pinMode(RGB_LED_GREEN_PIN, OUTPUT);
    pinMode(RGB_LED_BLUE_PIN, OUTPUT);
    pinMode(RGB_LED_RED_PIN, OUTPUT);   
    
    digitalWrite(CONTROL_MODE_LED_BLUETOOTH_PIN, HIGH);
    digitalWrite(CONTROL_MODE_LED_OBSTACLE_AVOIDANCE_PIN, HIGH);
    digitalWrite(CONTROL_MODE_LED_FIND_OBJECT_PIN, HIGH);
    digitalWrite(RGB_LED_GREEN_PIN, HIGH);    
    delay(300);
    digitalWrite(RGB_LED_GREEN_PIN, LOW);    
    digitalWrite(RGB_LED_BLUE_PIN, HIGH);    
    delay(300);
    digitalWrite(RGB_LED_BLUE_PIN, LOW);    
    digitalWrite(RGB_LED_RED_PIN, HIGH);    
    delay(300);
    digitalWrite(RGB_LED_RED_PIN, LOW);   

    digitalWrite(CONTROL_MODE_LED_OBSTACLE_AVOIDANCE_PIN, LOW);
    digitalWrite(CONTROL_MODE_LED_FIND_OBJECT_PIN, LOW);

  }
