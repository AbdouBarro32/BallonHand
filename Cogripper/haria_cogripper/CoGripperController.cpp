#include "CoGripperController.h"

CoGripperController::CoGripperController(uint8_t txPin, uint8_t rxPin, uint8_t dirPin)
  : dynaSerial(1), dxl(dynaSerial, dirPin), motor1(1, dxl), motor2(2, dxl), motor3(3, dxl), status(CoGripperStatus::OPEN) {
  dynaSerial.begin(57600, SERIAL_8N1, txPin, rxPin);
  dxl.begin(57600);
  dxl.setPortProtocolVersion(2.0);
}

void CoGripperController::initialize() {
  motor1.initialize();
  motor2.initialize();
  motor3.initialize();  // Initialize motor3
}

void CoGripperController::limitsCalibration() {
  motor1.limitsCalibration();
  motor2.limitsCalibration();
  motor3.limitsCalibration();  // Calibrate motor3
}

void CoGripperController::controlLoop() {
  motor1.updatePosition();
  motor2.updatePosition();
  motor3.updatePosition();  // Update motor3 position
  motor1.updateLoad();
  motor2.updateLoad();
  motor3.updateLoad();  // Update motor3 load
  motor1.updateStatus();
  motor2.updateStatus();
  motor3.updateStatus();  // Update motor3 status
  motor1.updateData();
  motor2.updateData();
  motor3.updateData();  // Update motor3 data
  updateData();
  handleTorqueLimitStatus();
  motor1.goToGoalPosition();
  motor2.goToGoalPosition();
  motor3.goToGoalPosition();  // Go to goal position for motor3
}

void CoGripperController::handleButtonStates(ButtonState currentState) {
  switch (currentState) {
    case ButtonState::CW_25DEG:
      if (motor1.getGoalPosition() + 70 < motor1.getEndPositionMax()) {
        motor1.setGoalPosition(motor1.getGoalPosition() + 70);
      }
      if (motor2.getGoalPosition() + 70 < motor2.getEndPositionMax()) {
        motor2.setGoalPosition(motor2.getGoalPosition() + 70);
      }
      if (motor3.getGoalPosition() + 70 < motor3.getEndPositionMax()) {
        motor3.setGoalPosition(motor3.getGoalPosition() + 70);
      }
      break;
    case ButtonState::PUSH:
      if (status == CoGripperStatus::OPEN) {
        status = CoGripperStatus::CLOSED;
        closeAllMotors();
      } else if (status == CoGripperStatus::CLOSED) {
        status = CoGripperStatus::OPEN;
        openAllMotors();
      }
      break;
    case ButtonState::CCW_25DEG:
      if (motor1.getGoalPosition() - 70 > motor1.getEndPositionMin()) {
        motor1.setGoalPosition(motor1.getGoalPosition() - 70);
      }
      if (motor2.getGoalPosition() - 70 > motor2.getEndPositionMin()) {
        motor2.setGoalPosition(motor2.getGoalPosition() - 70);
      }
      if (motor3.getGoalPosition() - 70 > motor3.getEndPositionMin()) {
        motor3.setGoalPosition(motor3.getGoalPosition() - 70);
      }
      break;
    default:
      break;
  }
}

void CoGripperController::handleTorqueLimitStatus() {
  motor1.handleTorqueLimitStatus_new();
  motor2.handleTorqueLimitStatus_new();
  motor3.handleTorqueLimitStatus_new();  // Handle torque limit for motor3
}

void CoGripperController::printGripperStatus() {
  MotorStatus status1, status2, status3;
  status1 = motor1.getStatus();
  status2 = motor2.getStatus();
  status3 = motor3.getStatus();

  Serial.print("Motor 1 - Pos: ");
  Serial.print(motor1.getCurrentPosition());
  Serial.print(", Goal Pos: ");
  Serial.print(motor1.getGoalPosition());
  Serial.print(", Load: ");
  Serial.print(motor1.getCurrentLoad());
  Serial.print(", Pos%: ");
  Serial.print(motor1.getCurrentPositionPercentage());
  Serial.print(", Move: ");
  Serial.print(motor1.getMoving());
  Serial.print(", Status: ");
  Serial.print(motorStatusToString(status1));

  Serial.print("\t\tMotor 2 - Position: ");
  Serial.print(motor2.getCurrentPosition());
  Serial.print(", Goal Position: ");
  Serial.print(motor2.getGoalPosition());
  Serial.print(", Load: ");
  Serial.print(motor2.getCurrentLoad());
  Serial.print(", Position %: ");
  Serial.print(motor2.getCurrentPositionPercentage());
  Serial.print(", Moving: ");
  Serial.print(motor2.getMoving());
  Serial.print(", Status: ");
  Serial.print(motorStatusToString(status2));

  Serial.print("\t\tMotor 3 - Position: ");
  Serial.print(motor3.getCurrentPosition());
  Serial.print(", Goal Position: ");
  Serial.print(motor3.getGoalPosition());
  Serial.print(", Load: ");
  Serial.print(motor3.getCurrentLoad());
  Serial.print(", Position %: ");
  Serial.print(motor3.getCurrentPositionPercentage());
  Serial.print(", Moving: ");
  Serial.print(motor3.getMoving());
  Serial.print(", Status: ");
  Serial.print(motorStatusToString(status3));

  Serial.print("\t\tGRIPPER STATUS: ");
  if (status == CoGripperStatus::OPEN) {
    Serial.println("OPEN");
  }
  if (status == CoGripperStatus::CLOSED) {
    Serial.println("CLOSE");
  }
}

void CoGripperController::printGripperData() {
  Serial.print("Motor 1 - Pos: ");
  Serial.print(data.motor1.position);
  Serial.print(", Pos%: ");
  Serial.print(data.motor1.positionPercentage);
  Serial.print(", Load: ");
  Serial.print(data.motor1.load);
  Serial.print(", Status: ");
  Serial.print(motorStatusToString(data.motor1.status));

  Serial.print("\t\tMotor 2 - Position: ");
  Serial.print(data.motor2.position);
  Serial.print(", Pos%: ");
  Serial.print(data.motor2.positionPercentage);
  Serial.print(", Load: ");
  Serial.print(data.motor2.load);
  Serial.print(", Status: ");
  Serial.print(motorStatusToString(data.motor2.status));

  Serial.print("\t\tMotor 3 - Position: ");
  Serial.print(data.motor3.position);
  Serial.print(", Pos%: ");
  Serial.print(data.motor3.positionPercentage);
  Serial.print(", Load: ");
  Serial.print(data.motor3.load);
  Serial.print(", Status: ");
  Serial.print(motorStatusToString(data.motor3.status));

  Serial.print("\t\tGRIPPER STATUS: ");
  if (data.status == CoGripperStatus::OPEN) {
    Serial.println("OPEN");
  }
  if (data.status == CoGripperStatus::CLOSED) {
    Serial.println("CLOSE");
  }
}

void CoGripperController::setGripperMaxPWM(int pwmGoal) {
  motor1.setGoalPWM(pwmGoal);
  motor2.setGoalPWM(pwmGoal);
  motor3.setGoalPWM(pwmGoal);  // Set PWM for motor3
}

void CoGripperController::updateData() {
  data.motor1 = motor1.getData();
  data.motor2 = motor2.getData();
  data.motor3 = motor3.getData();  // Update data for motor3
  data.status = status;
}

CoGripperData CoGripperController::getData() {
  return data;
}

void CoGripperController::setGoalPositionMotor1(int position) {
  motor1.setGoalPosition(position);
}

void CoGripperController::setGoalPositionMotor2(int position) {
  motor2.setGoalPosition(position);
}

void CoGripperController::setGoalPositionMotor3(int position) {
  motor3.setGoalPosition(position);
}

void CoGripperController::openAllMotors(void) {
  motor1.setGoalPosition(motor1.getEndPositionMax());
  motor2.setGoalPosition(motor2.getEndPositionMax());
  motor3.setGoalPosition(motor3.getEndPositionMax());
}

void CoGripperController::closeAllMotors(void) {
  motor1.setGoalPosition(motor1.getEndPositionMin());
  motor2.setGoalPosition(motor2.getEndPositionMin());
  motor3.setGoalPosition(motor3.getEndPositionMin());
}

void CoGripperController::stopAllMotors(void) {
  motor1.setGoalPosition(motor1.getCurrentPosition());
  motor2.setGoalPosition(motor2.getCurrentPosition());
  motor3.setGoalPosition(motor3.getCurrentPosition());
}

const char* CoGripperController::motorStatusToString(MotorStatus status) const {
  switch (status) {
    case MotorStatus::GRASP:
      return "GRASP";
    case MotorStatus::CLOSING:
      return "CLOSING";
    case MotorStatus::OPEN:
      return "OPEN";
    case MotorStatus::OPENING:
      return "OPENING";
    case MotorStatus::HALF_CLOSED:
      return "HALF_CLOSED";
    default:
      return "UNKNOWN";
  }
}