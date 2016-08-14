#include "/Users/Jarno/Arduino/bluetoothOhjausBot/AvoidObstacleController.h"

AvoidObstacleController::AvoidObstacleController(MotorController* motorCtrl, ServoController* servoCtrl, SonarSRF08* sonar) {
      motorController = motorCtrl;
      servoController = servoCtrl;
      mainSonar = sonar;
   }
void AvoidObstacleController::moveRobot() {
     time = millis();
     Serial.println("avoid obstacle::moveRobot");
     /* if (time - sonarPanLastMovement > 400) {
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
         servoController->setPanAndTilt(panServoAngle, tiltServoAngle);
      
         float sensorReading = 0;
         sensorReading = mainSonar->getRange(unit);
         
        if (sensorReading > 30) {
          motorController->moveForward();
          }
        if (sensorReading <=30 && sensorReading != 0) {
          motorController->turnLeft();
          delay(700);
            
            } 
         } */
}
