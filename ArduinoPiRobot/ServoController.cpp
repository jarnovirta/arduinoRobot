#include "/Users/Jarno/Arduino/bluetoothOhjausBot/ServoController.h"

ServoController::ServoController() {
      tiltServo.attach(9);  // attaches the servo on pin 9 to the servo object 
      panServo.attach(10);
      setPanAndTilt(90, 90); 
}

void ServoController::setPanAndTilt(int pan, int tilt) {
       panServo.write(pan);
       tiltServo.write(tilt);
  }
