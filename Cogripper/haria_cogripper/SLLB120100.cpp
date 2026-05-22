#include "SLLB120100.h"
#include <Arduino.h>

SLLB120100::SLLB120100() {
  pinMode(cw_11deg_pin, INPUT);
  pinMode(cw_25deg_pin, INPUT);
  pinMode(push_pin, INPUT);
  pinMode(ccw_25deg_pin, INPUT);
  pinMode(ccw_11deg_pin, INPUT);
}

ButtonState SLLB120100::readState() {
  int cw_11deg_state = digitalRead(cw_11deg_pin);
  int cw_25deg_state = digitalRead(cw_25deg_pin);
  push_state_prec = push_state;
  push_state = digitalRead(push_pin);
  int ccw_25deg_state = digitalRead(ccw_25deg_pin);
  int ccw_11deg_state = digitalRead(ccw_11deg_pin);

  if (cw_11deg_state == HIGH) {
    if (cw_25deg_state == HIGH) {
      return CW_25DEG;
    }
    return CW_11DEG;
  } else if (push_state > push_state_prec) {
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
