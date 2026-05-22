#pragma once

enum class CoGripperStatus {
  CLOSED,
  OPEN
};

enum class MotorStatus {
  GRASP,
  CLOSING,
  OPEN,
  HALF_CLOSED, // CLOSED SENZA GRASP
  OPENING
};

struct MotorData {
  int load = 0;
  int position = 0;
  int positionPercentage = 0;
  MotorStatus status = MotorStatus::OPEN;
};

struct CoGripperData {
  MotorData motor1;
  MotorData motor2;
  CoGripperStatus status = CoGripperStatus::OPEN;
};

struct CoGripperCMD {
  char cmd = 'O';
  char aux = 'O';
};

struct CoGripperPos {
  int motor1_pos;
  int motor2_pos;
};
