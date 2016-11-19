#include "/Users/Jarno/Arduino/ArduinoPiRobot/MotorController.h"

MotorController::MotorController() {
      AFMS = Adafruit_MotorShield();
      rightRearMotor = AFMS.getMotor(1);
      leftRearMotor = AFMS.getMotor(2);
      rightFrontMotor = AFMS.getMotor(3);
      leftFrontMotor = AFMS.getMotor(4);
      AFMS.begin(); 
  }

void MotorController::setRobotSpeed(int leftWheelsSpeed, int rightWheelsSpeed) {
        leftFrontMotor->setSpeed(leftWheelsSpeed);       
        leftRearMotor->setSpeed(leftWheelsSpeed);
        rightRearMotor->setSpeed(rightWheelsSpeed);
        rightFrontMotor->setSpeed(rightWheelsSpeed);
  }
void MotorController::moveForward() {
        setRobotSpeed(80,80);
        leftRearMotor->run(FORWARD);
        rightRearMotor->run(FORWARD);
        leftFrontMotor->run(FORWARD);
        rightFrontMotor->run(FORWARD);

  }
  
void MotorController::moveBackward() {
        leftRearMotor->run(BACKWARD);
        rightRearMotor->run(BACKWARD);
        rightFrontMotor->run(BACKWARD);
        leftFrontMotor->run(BACKWARD);
        setRobotSpeed(80,80);
  }

void MotorController::turnLeft() {
        leftRearMotor->run(BACKWARD);
        rightRearMotor->run(FORWARD);
        rightFrontMotor->run(FORWARD);
        leftFrontMotor->run(BACKWARD);
        setRobotSpeed(80, 80);
  }
  
void MotorController::turnRight() {
        leftRearMotor->run(FORWARD);
        rightRearMotor->run(BACKWARD);
        rightFrontMotor->run(BACKWARD);
        leftFrontMotor->run(FORWARD);
        setRobotSpeed(80, 80);
    
  }

void MotorController::stopRobot() {
        leftRearMotor->run(RELEASE);
        rightRearMotor->run(RELEASE);
        rightFrontMotor->run(RELEASE);
        leftFrontMotor->run(RELEASE);
  }
