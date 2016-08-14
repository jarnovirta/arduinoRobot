#include "/Users/Jarno/Arduino/bluetoothOhjausBot/BluetoothController.h"
BluetoothController::BluetoothController(MotorController *motor) {

    motorController = motor;
            
    /*int bluetoothTx = 2;  // TX-O pin of bluetooth mate, Arduino D2
    int bluetoothRx = 3;  // RX-I pin of bluetooth mate, Arduino D3
    bluetooth = new SoftwareSerial(bluetoothTx, bluetoothRx);
    */
    // BLUESMIRF MODULE SETTINGS:
    /*Serial.begin(115200);

    Serial.print("$");  // Print three times individually
    Serial.print("$"); 
    Serial.print("$"); 
    delay(100);  // Short delay, wait for the Mate to send back CMD
    Serial.println("U,9600,N");  // Temporarily Change the baudrate to 9600, no parity
    Serial.println("---");
    Serial.begin(9600);  // Start bluetooth serial at 9600  // 115200 can be too fast at times for NewSoftSerial to relay the data reliably 
    
    while(true) {
        Serial.println("TEST\n");
        delay(200);
      }
    /*
    bluetooth->begin(115200);  // The Bluetooth Mate defaults to 115200bps
    bluetooth->print("$");  // Print three times individually
    bluetooth->print("$");
    bluetooth->print("$");  // Enter command mode
    delay(100);  // Short delay, wait for the Mate to send back CMD
    bluetooth->println("U,9600,N");  // Temporarily Change the baudrate to 9600, no parity
    bluetooth->begin(9600);  // Start bluetooth serial at 9600  // 115200 can be too fast at times for NewSoftSerial to relay the data reliably 
    */
}

// Takes control mode as argument (1=Bluetooth control; 2=Avoid obstacles) and returns 
// control mode (possibly changed by user)

int BluetoothController::getAndExecuteBluetoothCommand(int controlMode) {
	// if (bluetooth->available()) {
        if (Serial.available() > 0) {
          // char command = (char) bluetooth->read();
          char command = (char) Serial.read();
                if (command == 'W' || command == 'w') {
                        if (command == 'W') {
                            motorController->stopRobot();
                            controlMode = 2;
                          }
                        else controlMode = 1;
                  }
             
                 else if (controlMode == 1) {
        	        switch(command) {
        	            case 'F':
        	              motorController->moveForward(); 
        	              break;    
        	                  
        	            case 'B':
        	              motorController->moveBackward();
        	              break;       
        	            
        	            case 'L':
        	              motorController->turnLeft();
        	              break;       
        	            
        	            case 'R':
        	              motorController->turnRight();
        	              break;     
        	            
        	            case 'S':
        	              motorController->stopRobot();
        	              break;  
        
        	            // default:
        	              // sendMessage("*** BUDGETCACTUS v3.5.16;SUPO;seuranta#156445: UNAUTHORIZED ACCESS ***\n"); 
        	         }
                    }
               
     }
     return controlMode;
}
void BluetoothController::sendMessage(String message) {
               Serial.println(message);
               delay(50);
  		// bluetooth->println(message);
}
