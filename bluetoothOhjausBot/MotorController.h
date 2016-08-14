#ifndef MOTORCONTROLLER_H
#define MOTORCONTROLLER_H

#include <Adafruit_MotorShield.h>

class MotorController {
    public:
      MotorController();
      void turnRight();
      void turnLeft();
      void moveForward();
      void moveBackward();
      void stopRobot();
      void setRobotSpeed(int leftWheelsSpeed, int rightWheelsSpeed);   
    private:
      Adafruit_MotorShield AFMS;
      Adafruit_DCMotor* leftRearMotor;
      Adafruit_DCMotor* rightRearMotor;
      Adafruit_DCMotor* rightFrontMotor;
      Adafruit_DCMotor* leftFrontMotor;
  };
  
#endif
