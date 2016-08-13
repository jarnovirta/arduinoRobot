#ifndef BUDGETCACTUSMotorControl
#define BUDGETCACTUSMotorControl

#include "Arduino.h"
#include <Servo.h> 
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_PWMServoDriver.h"

class MotorControl {
    public:
      int executeBluetoothCommand(char command);
      void setPanAndTilt(int pan, int tilt);
      void start();
      
    private:
      void turnRight();
      void turnLeft();
      void moveForward();
      void moveBackward();
      void stopRobot();
      void setRobotSpeed(int leftWheelsSpeed, int rightWheelsSpeed);

      Adafruit_MotorShield AFMS;
      Adafruit_DCMotor *leftRearMotor;
      Adafruit_DCMotor *rightRearMotor;
      Adafruit_DCMotor *rightFrontMotor;
      Adafruit_DCMotor *leftFrontMotor;

      unsigned long sonarPanLastMovement = millis();
      Servo panServo;  // create servo object to control a servo 
                  // a maximum of eight servo objects can be created 
      Servo tiltServo;
  };
  
  #endif
