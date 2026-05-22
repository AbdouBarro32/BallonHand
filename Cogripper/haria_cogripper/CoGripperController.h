#ifndef COGRIPPER_CONTROLLER_H
#define COGRIPPER_CONTROLLER_H

#include <Dynamixel2Arduino.h>
#include <HardwareSerial.h>
#include "SLLB120100.h"
#include "Motor.h"

enum class CoGripperStatus {
  CLOSED,
  OPEN
};

struct CoGripperData {
  MotorData motor1;
  MotorData motor2;
  MotorData motor3;  
  CoGripperStatus status = CoGripperStatus::OPEN;
};

class CoGripperController {
public:
  CoGripperController(uint8_t txPin, uint8_t rxPin, uint8_t dirPin);

  void initialize();
  void limitsCalibration();

  void controlLoop();
  void handleButtonStates(ButtonState currentState);
  void handleTorqueLimitStatus();

  void updateData();
  CoGripperData getData();

  void setGripperMaxPWM(int pwmGoal);
  void setGoalPositionMotor1(int position);
  void setGoalPositionMotor2(int position);
  void setGoalPositionMotor3(int position);
  void openAllMotors();
  void closeAllMotors();
  void stopAllMotors();

  const char* motorStatusToString(MotorStatus status) const;
  void printGripperStatus();
  void printGripperData();

  int getMotor1EndPositionMin() const { return motor1.getEndPositionMin(); };
  int getMotor1EndPositionMax() const { return motor1.getEndPositionMax(); };
  int getMotor2EndPositionMin() const { return motor2.getEndPositionMin(); };
  int getMotor2EndPositionMax() const { return motor2.getEndPositionMax(); };
  int getMotor3EndPositionMin() const { return motor3.getEndPositionMin(); };
  int getMotor3EndPositionMax() const { return motor3.getEndPositionMax(); };
  Motor motor1;
  Motor motor2;
  Motor motor3;

private:
  HardwareSerial dynaSerial;
  Dynamixel2Arduino dxl;
  CoGripperStatus status;
  CoGripperData data;
};

#endif  // COGRIPPER_CONTROLLER_H
