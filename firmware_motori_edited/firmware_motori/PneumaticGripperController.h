#ifndef SIXTHFINGER_CONTROLLER_H
#define SIXTHFINGER_CONTROLLER_H

#include <Dynamixel2Arduino.h>
#include <HardwareSerial.h>
#include "SLLB120100.h"
#include "BluetoothSerial.h"

//This namespace is required to use Control table item names
using namespace ControlTableItem;

class PneumaticGripperController {
  private:
    HardwareSerial dynaSerial;
    Dynamixel2Arduino dxl;
    BluetoothSerial SerialBT;

    bool linear_mode;
    //I PUT THE THREE IDS
    const uint8_t LINEAR_DYNA_ID_1 = 1;
    const uint8_t LINEAR_DYNA_ID_2 = 2;
    const uint8_t LINEAR_DYNA_ID_3 = 3;

    const float DXL_PROTOCOL_VERSION = 2.0;

    int dyna_1_present_position = 0;
    int dyna_2_present_position = 0;
    int dyna_3_present_position = 0;

    int goal_position_1 = 2300;
    int goal_position_2 = 2300;
    int goal_position_3 = 2300;

    int pfinger=1;

    int motor_1_position = 0;
    int motor_2_position = 0;
    int motor_3_position = 0;

    int motor_1_load = 0;
    int motor_2_load = 0;
    int motor_3_load = 0;

    int linear_end_position_min_1 = 0;
    int linear_end_position_max_1 = 0;

    int linear_end_position_min_2 = 0;
    int linear_end_position_max_2 = 0;

    int linear_end_position_min_3 = 0;
    int linear_end_position_max_3 = 0;

    int stall_avoidance_counter = 0;


  public:
    PneumaticGripperController(uint8_t txPin, uint8_t rxPin, uint8_t dirPin);

    void initialization();
    void limitsCalibration();
    int GetAbsoluteLoad(int id, boolean debug_print);
    void controlLoop();
    void handleButtonStates(ButtonState currentState);
    void handleTorqueLimitStatus(void);
    void updateGoalPositions();
    void printGripperStatus();
    void readBluetoothCommand();
    int pushMode(int fC);
};

#endif  // SIXTHFINGER_CONTROLLER_H
