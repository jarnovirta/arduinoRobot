
#include "Arduino.h"
#include "\Users\Jarno\Arduino\bluetoothOhjausBot\RobotMotorControl.h"
#include <Adafruit_MotorShield.h>

  
  // Motor controller setup (cannot use constructor because AFMS.begin() must be called from the setup() function
void MotorControl::start() {
      AFMS = Adafruit_MotorShield();
      leftRearMotor = AFMS.getMotor(2);
      rightRearMotor = AFMS.getMotor(1);
      rightFrontMotor = AFMS.getMotor(3);
      leftFrontMotor = AFMS.getMotor(4);

      setRobotSpeed(80,80);
      tiltServo.attach(9);  // attaches the servo on pin 9 to the servo object 
      panServo.attach(10);
      setPanAndTilt(90, 90);
      AFMS.begin();
  }
void MotorControl::setPanAndTilt(int pan, int tilt) {
       panServo.write(pan);
       tiltServo.write(tilt); 
  }

int MotorControl::executeBluetoothCommand(char command) {
        switch(command) {
            case 'F':
              moveForward(); 
              break;    
                  
            case 'B':
              moveBackward();
              break;       
            
            case 'L':
              turnLeft();
              break;       
            
            case 'R':
              turnRight();
              break;     
            
            case 'S':
              stopRobot();
              break;  
            
            /*case 'W':
              stopRobot();
              controlMode = 1;
              tiltServoAngle = 120;
              panServoAngle = 90;
              motorController.setPanAndTilt(panServoAngle, tiltServoAngle);
            */
            default:
              return -1;
              }
    return 0;              
             
}
void MotorControl::setRobotSpeed(int leftWheelsSpeed, int rightWheelsSpeed) {
        leftRearMotor->setSpeed(leftWheelsSpeed);
        rightRearMotor->setSpeed(rightWheelsSpeed);
        rightFrontMotor->setSpeed(rightWheelsSpeed);
        leftFrontMotor->setSpeed(leftWheelsSpeed); 
  }
void MotorControl::moveForward() {

  AFMS.begin();
        leftRearMotor->run(FORWARD);
        rightRearMotor->run(FORWARD);
        rightFrontMotor->run(FORWARD);
        leftFrontMotor->run(FORWARD);
        setRobotSpeed(80,80);
  }
  
void MotorControl::moveBackward() {
     
  AFMS.begin();
        leftRearMotor->run(BACKWARD);
        rightRearMotor->run(BACKWARD);
        rightFrontMotor->run(BACKWARD);
        leftFrontMotor->run(BACKWARD);
        setRobotSpeed(80,80);
  }

void MotorControl::turnLeft() {
  AFMS.begin();
      leftRearMotor->run(BACKWARD);
      rightRearMotor->run(FORWARD);
      rightFrontMotor->run(FORWARD);
      leftFrontMotor->run(BACKWARD);
      setRobotSpeed(80, 80);
  }
  
void MotorControl::turnRight() {

  AFMS.begin();
    leftRearMotor->run(FORWARD);
    rightRearMotor->run(BACKWARD);
    rightFrontMotor->run(BACKWARD);
    leftFrontMotor->run(FORWARD);
    setRobotSpeed(80, 80);
    
  }

void MotorControl::stopRobot() {
   
  AFMS.begin();
    leftRearMotor->run(RELEASE);
    rightRearMotor->run(RELEASE);
    rightFrontMotor->run(RELEASE);
    leftFrontMotor->run(RELEASE);
  
  }
