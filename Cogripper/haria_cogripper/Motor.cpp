#include "Motor.h"

Motor::Motor(uint8_t id, Dynamixel2Arduino &dxl)
  : id(id), dxl(dxl), goalPosition(2300), currentPosition(0), currentPositionPercentage(0), load(0), endPositionMin(0), endPositionMax(0), stallAvoidanceCounter(0), stallDetectionCounter(0), secondary_goal(false), stall_avoided(false), status(MotorStatus::OPEN) {}


void Motor::initialize() {
  // Set the desired maximum current value
  uint16_t max_current = 800;
  uint32_t moving_threshold = 3;
  // Set the positive motion direction
  uint8_t inverted = 0;

  dxl.torqueOff(id);                                
  dxl.setOperatingMode(id, OP_EXTENDED_POSITION);
  dxl.write(id, 36, (uint8_t *)&max_current, sizeof(max_current));            // Write maximum current (register address 36, 2 bytes, ID 1) 	PWM Limit	Maximum - PWM Limit -	RW - 885
  dxl.write(id, 10, (uint8_t *)&moving_threshold, sizeof(moving_threshold));  // Write maximum current (register address 36, 2 bytes, ID 1) 	PWM Limit	Maximum - PWM Limit -	RW - 885
    // dxl.write(DYNA_1_ID, 10, (uint8_t *)&inverted, sizeof(inverted));        // Write maximum current (register address 38, 2 bytes, ID 1)
  delay(500);
  dxl.torqueOn(id);

  // buona velocità per event-driven control
  // dxl.writeControlTableItem(PROFILE_VELOCITY, id, 30);
  // dxl.writeControlTableItem(PROFILE_ACCELERATION, id, 25);
  changeMotorVelocity(HIGH);

  // buona velocità per continuous control
  // dxl.writeControlTableItem(PROFILE_VELOCITY, id, 300);
  // dxl.writeControlTableItem(PROFILE_ACCELERATION, id, 250);

  currentPosition = dxl.getPresentPosition(id);
  goalPosition = currentPosition;
  dxl.setGoalPosition(id, goalPosition);
}

void Motor::changeMotorVelocity(int vel) {
  Serial.print("Motor ");
  Serial.print(id);
  Serial.print(": ");
  bool check1 = false;
  bool check2 = false;
  int del = 100;
  uint32_t profile_vel;
  uint32_t profile_acc;

  if (vel == HIGH) {
    profile_vel = 300;
    profile_acc = 70;
  }
  if (vel == LOW) {
    profile_vel = 30;
    profile_acc = 25;
  }

  while (!dxl.write(id, 112, (uint8_t *)&profile_vel, sizeof(profile_vel))) {
    delay(50);
  }
  while (!dxl.write(id, 108, (uint8_t *)&profile_acc, sizeof(profile_acc))) {
    delay(50);
  }
  Serial.println("velocity change success");

  /*
  check1 = dxl.write(id, 112, (uint8_t *)&profile_vel, sizeof(profile_vel));
  // check1 = dxl.writeControlTableItem(PROFILE_VELOCITY, id, 300);
  delay(500);
  check2 = dxl.write(id, 108, (uint8_t *)&profile_acc, sizeof(profile_acc));
  // check2 = dxl.writeControlTableItem(PROFILE_ACCELERATION, id, 250);
  if (check1 && check2) {
    Serial.println("velocity change success");
  }
  */
}

void Motor::limitsCalibration() {
  currentPosition = dxl.getPresentPosition(id);
  goalPosition = currentPosition;
  dxl.setGoalPosition(id, goalPosition);

  int THRESHOLD_LOAD = 50;
  Serial.println("SEARCHING LOAD");
  do {
    Serial.print(".");
    goalPosition -= 15;
    dxl.setGoalPosition(id, goalPosition);
    currentPosition = dxl.getPresentPosition(id);
    // load = getCurrentLoad();
    updateLoad();
  } while (load <= THRESHOLD_LOAD);
  Serial.println();
  Serial.print("LOAD FOUND!");
  delay(500);

  goalPosition = currentPosition - 20;
  endPositionMin = goalPosition;
  endPositionMax = goalPosition + 1000;
  Serial.print("COMPUTED END POSITION MIN: ");
  Serial.println(endPositionMin);
  delay(200);
  Serial.print("COMPUTED END POSITION MAX: ");
  Serial.println(endPositionMax);
  delay(200);
  goalPosition = endPositionMax;
  Serial.println("GOING TO MAX POSITION");
  setGoalPosition(goalPosition);
  goToGoalPosition();
  Serial.println();
  Serial.println("DONE!");
  delay(1000);
}

void Motor::updatePosition() {
  currentPosition = dxl.getPresentPosition(id);
  currentPositionPercentage = map(currentPosition, endPositionMin, endPositionMax, 0, 100);
}

void Motor::updateLoad() {
  uint16_t motor_current;
  dxl.read(id, 126, 2, (uint8_t *)&motor_current, sizeof(motor_current));

  if (motor_current >= 1023) {
    motor_current = 65535 - motor_current;
  }

  load = motor_current;
}

void Motor::updateData() {
  data.load = load;
  data.position = currentPosition;
  data.positionPercentage = currentPositionPercentage;
  data.status = status;
}

MotorData Motor::getData() {
  return data;
}

void Motor::handleTorqueLimitStatus() {
  int tolerance = 6;

  if (load > 400 && (currentPosition > goalPosition + tolerance)) {
    Serial.println("dyna_1_stall_avoidance_counter");
    stallDetectionCounter++;
  }

  if (stallDetectionCounter > 0) {
    stallDetectionCounter = 0;
    setGoalPosition(currentPosition + 2);
    // dxl.setGoalPosition(DYNA_1_ID, dyna_1_present_position + 2);
    delay(200);
  }

  if (stallDetectionCounter == 0) {
    // dxl.setGoalPosition(DYNA_1_ID, goal_position_1);
    setGoalPosition(goalPosition);
  }
}

void Motor::handleTorqueLimitStatus_new() {
  int tolerance = 6;
  // Serial.print("detection counter: ");
  // Serial.print(stallDetectionCounter);
  // Serial.print(", avoidance counter: ");
  // Serial.print(stallAvoidanceCounter);
  // Serial.print(", secondary goal: ");
  // Serial.print(secondary_goal);
  // Serial.print(", stall_avoided: ");
  // Serial.print(stall_avoided);


  if (load > 400 && (currentPosition > goalPosition + tolerance)) {
    stallDetectionCounter++;
  }

  if (stallDetectionCounter > 10) {
    stallDetectionCounter = 0;
    setGoalPosition(currentPosition + 2);
    secondary_goal = true;
  }

  // RESTRINGIMENTO SECONDARIO
  // questo comportamento ci assicura che i dynamixel
  // continuino a stringere mantenendo una posizione
  // e una coppia dei motori che evitano lo stallo
  if (secondary_goal) {
    if (load < 340) {
      setGoalPosition(goalPosition - 1);
    }
    if (load > 360) {
      setGoalPosition(currentPosition + 1);
    }
    if (load > 340 && load < 360) {
      stallAvoidanceCounter++;
      if (stallAvoidanceCounter > 20) {
        stallAvoidanceCounter = 0;
        stall_avoided = true;
      }
    }
    if (stall_avoided || status == MotorStatus::OPENING || status == MotorStatus::OPEN) {
      stall_avoided = false;
      secondary_goal = false;
    }
  }
}

int Motor::getCurrentLoad() {
  return load;
}

int Motor::getCurrentPosition() const {
  return currentPosition;
}

void Motor::setGoalPosition(int position) {
  goalPosition = position;
}

void Motor::goToGoalPosition() {
  dxl.setGoalPosition(id, goalPosition);
}

int Motor::getCurrentPositionPercentage() const {
  return currentPositionPercentage;
}

void Motor::setGoalPWM(int pwmGoal) {
  dxl.setGoalPWM(id, pwmGoal);
}

bool Motor::getMoving() const {
  bool moving;
  uint8_t moving_val;
  dxl.read(id, 122, 1, (uint8_t *)&moving_val, sizeof(moving_val));
  moving = moving_val == 1 ? true : false;
  return moving;
}

void Motor::updateStatus() {
  if (getMoving()) {
    if (currentPosition < goalPosition) {
      status = MotorStatus::OPENING;
    } else {
      if (!secondary_goal) {
        status = MotorStatus::CLOSING;
      }
    }
  } else {
    if (currentPositionPercentage >= 95) {  // da commentare
      status = MotorStatus::OPEN;
    } else {
      if (load < 200 && !secondary_goal) {  // da commentare
        status = MotorStatus::HALF_CLOSED;
      } else {
        status = MotorStatus::GRASP;
      }
    }
  }
}

MotorStatus Motor::getStatus() {
  return status;
}
