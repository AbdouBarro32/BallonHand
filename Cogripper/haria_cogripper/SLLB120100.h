#ifndef SLLB120100_H
#define SLLB120100_H

enum ButtonState {
  CW_11DEG,
  CW_25DEG,
  PUSH,
  CCW_11DEG,
  CCW_25DEG,
  UNKNOWN
};

class SLLB120100 {
private:
  const int cw_11deg_pin = 26;
  const int cw_25deg_pin = 18;
  const int push_pin = 19;
  const int ccw_25deg_pin = 23;
  const int ccw_11deg_pin = 5;
  int push_state = false;
  int push_state_prec = false;
public:
  SLLB120100();
  ButtonState readState();
  ButtonState getButtonStates(bool debug_print = false);
};

#endif
