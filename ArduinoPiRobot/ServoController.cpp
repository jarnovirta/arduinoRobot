#include "/Users/Jarno/Arduino/ArduinoPiRobot/ServoController.h"

ServoController::ServoController() {
      setPanAndTilt(90, 90); 
}

void ServoController::setPanAndTilt(int pan, int tilt) {
      tiltServo.attach(9);  // attaches the servo on pin 9 to the servo object 
      panServo.attach(10);
      panServo.write(pan);
      tiltServo.write(tilt);
      delay(100);
      tiltServo.detach();
      panServo.detach();
  }
