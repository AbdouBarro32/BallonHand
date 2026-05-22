#include "SLLB120100.h"
#include <Arduino.h>

//constructor SLLB120100::SLLB120100() for the SLLB120100 class declared in the header file SLLB120100.h
//we define a new costructor
SLLB120100::SLLB120100() {
  pinMode(cw_11deg_pin, INPUT);
  pinM/Users/abdoubarro/Desktop/vanni_simone/CodiceTirocinio/firmware_motori/SLLB120100.hode(cw_25deg_pin, INPUT);
  pinMode(push_pin, INPUT);
  pinMode(ccw_25deg_pin, INPUT);
  pinMode(ccw_11deg_pin, INPUT);
}

//we define the readState() function declared in class SLLB120100 inside SLLB120100.h
ButtonState SLLB120100::readState() {
  int cw_11deg_state = digitalRead(cw_11deg_pin);
  int cw_25deg_state = digitalRead(cw_25deg_pin);
  int push_state = digitalRead(push_pin);
  int ccw_25deg_state = digitalRead(ccw_25deg_pin);
  int ccw_11deg_state = digitalRead(ccw_11deg_pin);

  if (cw_11deg_state == HIGH) {
    if (cw_25deg_state == HIGH) {
      return CW_25DEG;
    }
    return CW_11DEG;
  } else if (push_state == HIGH) {
    return PUSH;
  } else if (ccw_11deg_state == HIGH) {
    if (ccw_25deg_state == HIGH) {
      return CCW_25DEG;
    }
    return CCW_11DEG;
  } else {
    return UNKNOWN;
  }
}
//we define the getButtonStates() function declared in class SLLB120100 inside SLLB120100.h
ButtonState SLLB120100::getButtonStates(bool debug_print) {
  ButtonState currentState = readState();

  if (debug_print) {
    switch (currentState) {
      case CW_11DEG:
        Serial.println("+11°");
        break;
      case CW_25DEG:
        Serial.println("+25°");
        break;
      case PUSH:
        Serial.println("push!");
        break;
      case CCW_11DEG:
        Serial.println("-11°");
        break;
      case CCW_25DEG:
        Serial.println("-25°");
        break;
      case UNKNOWN:
        Serial.println("0°");
        break;
    }
  }

  return currentState;
}
