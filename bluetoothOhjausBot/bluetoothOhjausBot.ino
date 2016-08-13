
#include <Servo.h> 
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_PWMServoDriver.h"
#include <SonarSRF08.h>
#include <SoftwareSerial.h>  
#include "\Users\Jarno\Arduino\bluetoothOhjausBot\RobotMotorControl.h"

#define MAIN_08_ADDRESS (0x70)
#define GAIN_REGISTER 0x09
#define LOCATION_REGISTER 0x8C
char unit = 'c'; // 'i' for inches, 'c' for centimeters, 'm' for micro-seconds

SonarSRF08 MainSonar;

MotorControl motorController;
int controlMode;  // 1=avoid obstacles; 2=Bluetooth control

// Bluetooth settings

int bluetoothTx = 2;  // TX-O pin of bluetooth mate, Arduino D2
int bluetoothRx = 3;  // RX-I pin of bluetooth mate, Arduino D3

int tiltServoAngle = 90;
int panServoAngle = 90;
int previousPanServoAngle = 140;

SoftwareSerial bluetooth(bluetoothTx, bluetoothRx);
void setup() 
{ 
      motorController.start(); // Cannot use initialiser to call Motor Shield Library begin() on Motor Shield because it
                                // must be called in setup
      Serial.begin(9600);  // Begin the serial monitor at 9600bps
      Serial.println("*** BUDGETCACTUS setup... ***");
      MainSonar.connect(MAIN_08_ADDRESS, GAIN_REGISTER, LOCATION_REGISTER); // Start Sonar Range Finder:

      // Bluetooth settings:
      bluetooth.begin(115200);
      bluetooth.begin(115200);  // The Bluetooth Mate defaults to 115200bps
      bluetooth.print("$");  // Print three times individually
      bluetooth.print("$");
      bluetooth.print("$");  // Enter command mode
      delay(100);  // Short delay, wait for the Mate to send back CMD
      bluetooth.println("U,9600,N");  // Temporarily Change the baudrate to 9600, no parity

      // 115200 can be too fast at times for NewSoftSerial to relay the data reliably
      
      bluetooth.begin(9600);  // Start bluetooth serial at 9600  
    
      controlMode = 2; // Set robot to Bluetooth control mode
      
      Serial.print("*** BUDGETCACTUS setup complete ***");
}  
 
void loop() 
{ 

  if(bluetooth.available())  {
    char command = (char) bluetooth.read();
    
// BLUETOOTH CONTROL MODE
//
    if (controlMode == 2) {
        int result = motorController.executeBluetoothCommand(command);

        if (result == -1) bluetooth.println("*** BUDGETCACTUS v3.5.16;SUPO;seuranta#156445: UNAUTHORIZED ACCESS ***\n"); 
        while(bluetooth.available()) {
          bluetooth.read(); // Clear buffer of newline and other extra characters 
                            // to prevent the message from being sent for every character
                 }
      }
    }
  
 /*
//  OBSTACLE AVOIDANCE MODE
// 
  if (controlMode == 1) {
      unsigned long time = millis();
     
     if (time - sonarPanLastMovement > 400) {
         sonarPanLastMovement = time;
        
         if (panServoAngle == 90 && previousPanServoAngle == 140) {
           panServoAngle = 40;
           previousPanServoAngle = 90;
           }
         else if (panServoAngle == 90 && previousPanServoAngle == 40) {
           panServoAngle = 140;
           previousPanServoAngle = 40;
           }
         else if (panServoAngle == 140) {
           panServoAngle = 90;
           previousPanServoAngle = 140;
           }
         else {
           panServoAngle = 90;
           previousPanServoAngle = 40;
           }
           
           
         setPanAndTilt(panServoAngle, tiltServoAngle);
      
         float sensorReading = 0;
        sensorReading = MainSonar.getRange(unit);
         
        if (sensorReading > 30) {
              AFMS.begin();
          moveBackward();
          }
        if (sensorReading <=30 && sensorReading != 0) {
          AFMS.begin();    
          turnLeft();
          delay(700);
            
            } 
         }
         
      }
      */
}



  
  

