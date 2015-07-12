
#include <Servo.h> 
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_PWMServoDriver.h"
#include <SonarSRF08.h>


#define MAIN_08_ADDRESS (0x70)
#define GAIN_REGISTER 0x09
#define LOCATION_REGISTER 0x8C
// #include <SoftwareSerial.h>  

char unit = 'c'; // 'i' for inches, 'c' for centimeters, 'm' for micro-seconds

int tiltServoAngle = 90;
int panServoAngle = 90;
int previousPanServoAngle = 140;
unsigned long sonarPanLastMovement = millis();
    SonarSRF08 MainSonar;
    
    Servo panServo;  // create servo object to control a servo 
                // a maximum of eight servo objects can be created 
    Servo tiltServo;

    // Motor settings
    Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
    
    Adafruit_DCMotor *leftRearMotor = AFMS.getMotor(1);
    Adafruit_DCMotor *rightRearMotor = AFMS.getMotor(2);
    Adafruit_DCMotor *rightFrontMotor = AFMS.getMotor(3);
    Adafruit_DCMotor *leftFrontMotor = AFMS.getMotor(4);
    
    int controlMode;  // 1=avoid obstacles; 2=Bluetooth control
    
    // Bluetooth settings
    
   /* int bluetoothTx = 4;  // TX-O pin of bluetooth mate, Arduino D2
    int bluetoothRx = 5;  // RX-I pin of bluetooth mate, Arduino D3 */
   
void setup() 
{ 
    MainSonar.connect(MAIN_08_ADDRESS, GAIN_REGISTER, LOCATION_REGISTER);
   
      AFMS.begin();
      setRobotSpeed(80,80);
      tiltServo.attach(9);  // attaches the servo on pin 9 to the servo object 
      panServo.attach(10);
      setPanAndTilt(panServoAngle, tiltServoAngle);
     
      Serial.begin(115200);  
      
      Serial.print("$");
      Serial.print("$");
      Serial.print("$");
      
      delay(100); 
      
    /*  bluetooth.begin(115200);
      
     bluetooth.begin(115200);  // The Bluetooth Mate defaults to 115200bps
      bluetooth.print("$");  // Print three times individually
      bluetooth.print("$");
      bluetooth.print("$");  // Enter command mode
      delay(100);  // Short delay, wait for the Mate to send back CMD
      bluetooth.println("U,9600,N");  // Temporarily Change the baudrate to 9600, no parity
      // 115200 can be too fast at times for NewSoftSerial to relay the data reliably
      bluetooth.begin(9600);  // Start bluetooth serial at 9600  
     */
     controlMode = 2; // Set robot to Bluetooth control mode
     
} 
 
 
void loop() 
{ 

  if(Serial.available() > 0)  // If the bluetooth sent any characters
  {
      char command = Serial.read();
    
    if (command == 'w') {
         AFMS.begin();
         stopRobot();
        controlMode = 2;
        tiltServoAngle = 90;
        panServoAngle = 90;
        setPanAndTilt(panServoAngle, tiltServoAngle);
    }
     // Bluetooth control
    if (controlMode == 2) {
    
    switch(command) {
      case 'F':
        AFMS.begin();
        moveForward(); 
        break;    
            
      case 'B':
        AFMS.begin();
        moveBackward();
        break;       
      
      case 'L':
        AFMS.begin();
        turnLeft();
        break;       
      
      case 'R':
        AFMS.begin();
        turnRight();
        break;     
      case 'S':
        AFMS.begin();
        stopRobot();
        break;  
      case 'W':
        AFMS.begin();
        stopRobot();
        controlMode = 1;
        tiltServoAngle = 120;
        panServoAngle = 90;
        setPanAndTilt(panServoAngle, tiltServoAngle);
        
      }
      }
  
  }
  
  
//  Automatic obstacle avoidance mode
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
  }
   
  

void setPanAndTilt(int pan, int tilt) {
       panServo.write(pan);
       tiltServo.write(tilt); 
  }
  
void setRobotSpeed(int leftWheelsSpeed, int rightWheelsSpeed) {
        leftRearMotor->setSpeed(leftWheelsSpeed);
        rightRearMotor->setSpeed(rightWheelsSpeed);
        rightFrontMotor->setSpeed(rightWheelsSpeed);
        leftFrontMotor->setSpeed(leftWheelsSpeed); 
  }
void moveForward() {
        leftRearMotor->run(FORWARD);
        rightRearMotor->run(FORWARD);
        rightFrontMotor->run(FORWARD);
        leftFrontMotor->run(FORWARD);
        setRobotSpeed(80,80);

  }
  
void moveBackward() {
        leftRearMotor->run(BACKWARD);
        rightRearMotor->run(BACKWARD);
        rightFrontMotor->run(BACKWARD);
        leftFrontMotor->run(BACKWARD);
        setRobotSpeed(80,80);
  }

void turnLeft() {
      leftRearMotor->run(BACKWARD);
      rightRearMotor->run(FORWARD);
      rightFrontMotor->run(FORWARD);
      leftFrontMotor->run(BACKWARD);
      setRobotSpeed(80, 80);
      
      
  }
  
void turnRight() {
    leftRearMotor->run(FORWARD);
    rightRearMotor->run(BACKWARD);
    rightFrontMotor->run(BACKWARD);
    leftFrontMotor->run(FORWARD);
    setRobotSpeed(80, 80);
    
  }

void stopRobot() {
    leftRearMotor->run(RELEASE);
    rightRearMotor->run(RELEASE);
    rightFrontMotor->run(RELEASE);
    leftFrontMotor->run(RELEASE);
  
  }

  
  

