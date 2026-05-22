// #include <esp_now.h>
// #include <WiFi.h>

#include <Dynamixel2Arduino.h>
#include <HardwareSerial.h>

#include "SLLB120100.h"
#include "PneumaticGripperController.h"




#define dynaSerial_TX_pin 17
#define dynaSerial_RX_pin 16
const int DXL_DIR_PIN = 21;  // DYNAMIXEL Shield DIR PIN
PneumaticGripperController PneumaticGripperController(dynaSerial_TX_pin, dynaSerial_RX_pin, DXL_DIR_PIN);



#define DEBUG_SERIAL Serial

SLLB120100 buttonController;

void setup() {

  // Setup serial communication with USB cable (for debug purposes)
  Serial.begin(57600);




  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  delay(500);

  PneumaticGripperController.initialization();
  PneumaticGripperController.limitsCalibration();
}


boolean stall_avoidance_counter_flag = false;

void loop() {
  
  
  ButtonState currentState = buttonController.getButtonStates(true);
  Serial.println(currentState);
  PneumaticGripperController.handleButtonStates(currentState);
  // PneumaticGripperController.readBluetoothCommand();
  PneumaticGripperController.updateGoalPositions();
  PneumaticGripperController.handleTorqueLimitStatus();
  PneumaticGripperController.printGripperStatus();
  delay(2);
}

