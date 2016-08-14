#include "/Users/Jarno/Arduino/bluetoothOhjausBot/AvoidObstacleController.h"

AvoidObstacleController::AvoidObstacleController(MotorController* motorCtrl, ServoController* servoCtrl, SonarSRF08* sonar, BluetoothController* BT) {
      motorController = motorCtrl;
      servoController = servoCtrl;
      sonarPanLastMovement = millis();
      bluetoothController = BT;
      mainSonar = sonar;
   }
void AvoidObstacleController::moveRobot() {
     time = millis();
     
     if (time - sonarPanLastMovement > 400) {
       sonarPanLastMovement = time;
        
         if (panServoAngle == 90 && previousPanServoAngle == 140) {
           panServoAngle = 60;
           previousPanServoAngle = 90;
           }
         else if (panServoAngle == 90 && previousPanServoAngle == 60) {
           panServoAngle = 140;
           previousPanServoAngle = 60;
           }
         else if (panServoAngle == 140) {
           panServoAngle = 90;
           previousPanServoAngle = 140;
           }
         else {
           panServoAngle = 90;
           previousPanServoAngle = 60;
           }
           
          servoController->setPanAndTilt(panServoAngle, tiltServoAngle);
          
         float sensorReading = 0;
         sensorReading = mainSonar->getRange(unit);
        
        if (sensorReading > 30) {
          bluetoothController->sendMessage("Yli 30");
          motorController->moveForward();
          }
        if (sensorReading <=30 && sensorReading != 0) {
          bluetoothController->sendMessage("Alle 30");
          motorController->turnLeft();
          delay(700);
            
            } 
         } 
}
