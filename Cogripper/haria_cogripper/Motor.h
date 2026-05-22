#ifndef MOTOR_H
#define MOTOR_H

#include <Dynamixel2Arduino.h>

//This namespace is required to use Control table item names
using namespace ControlTableItem;

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

class Motor {
public:
  Motor(uint8_t id, Dynamixel2Arduino& dxl);

  void initialize();
  void limitsCalibration();
  void updatePosition();
  void updateLoad();
  void handleTorqueLimitStatus();
  void handleTorqueLimitStatus_new();
  void changeMotorVelocity(int vel);
  
  int getCurrentLoad();
  int getCurrentPosition() const;
  int getGoalPosition() const { return goalPosition; }
  int getEndPositionMin() const { return endPositionMin; }
  int getEndPositionMax() const { return endPositionMax; }
  void setGoalPosition(int position);
  void goToGoalPosition();
  int getCurrentPositionPercentage() const;

  uint8_t getID() const { return id; }
  void setGoalPWM(int pwmGoal);
  bool getMoving() const;

  MotorStatus getStatus();
  void updateStatus();

  MotorData getData();
  void updateData();


private:
  uint8_t id;
  Dynamixel2Arduino& dxl;
  int goalPosition;
  int currentPosition;
  int currentPositionPercentage;
  int load;
  int endPositionMin;
  int endPositionMax;
  int stallAvoidanceCounter;
  int stallDetectionCounter;
  bool secondary_goal;
  bool stall_avoided;
  MotorStatus status;
  MotorData data;
};

#endif  // MOTOR_H
