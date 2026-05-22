#ifndef SLLB120100_H
#define SLLB120100_H
//ceck if the marcro SLLB120100_H arleady exist if not execute the lines between #define SLLB120100_H to #endif otherwise skip it
//it serves to/Users/abdoubarro/Desktop/vanni_simone/CodiceTirocinio/firmware_motori/firmware_motori.ino ensure that the header file is include only once to avoid issue


//
enum ButtonState {
  CW_11DEG,
  CW_25DEG,
  PUSH,
  CCW_11DEG,
  CCW_25DEG,
  UNKNOWN
};

//we make the class SLLB120100 and we declare the costructor and two function(they are defined in SLLB120100.cpp ) 

class SLLB120100 {
private:
  const int cw_11deg_pin = 26;
  const int cw_25deg_pin = 18;
  const int push_pin = 19;
  const int ccw_25deg_pin = 23;
  const int ccw_11deg_pin = 5;

public:
  SLLB120100();
  ButtonState readState();
  ButtonState getButtonStates(bool debug_print = false);
};

#endif
