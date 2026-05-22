#include "PneumaticGripperController.h"

PneumaticGripperController::PneumaticGripperController(uint8_t txPin, uint8_t rxPin, uint8_t dirPin)
  : dynaSerial(1), dxl(dynaSerial, dirPin), linear_mode(true) {

  //Users/abdoubarro/Desktop/vanni_simone/CodiceTirocinio/firmware_motori/PneumaticGripperController.h/ Setup serial communication with the Dynamixels
  // Set Port baudrate to 57600bps. This has to match with DYNAMIXEL baudrate.
  dynaSerial.begin(57600, SERIAL_8N1, txPin, rxPin);
  dxl.begin(57600);
  // Set Port Protocol Version. This has to match with DYNAMIXEL protocol version.
  dxl.setPortProtocolVersion(2.0);
}

void PneumaticGripperController::initialization() {

  // Initialize Bluetooth Serial device
  Serial.println("\nInitialize Bluetooth Serial device");
  delay(500);
  Serial.println(SerialBT.begin("ESP32_SSF_LEFT"));  //Bluetooth device name
  delay(500);
  Serial.println(" Bluetooth Serial device OK");

  // Set the desired maximum current value
  uint16_t max_current = 800;

  // Set the positive motion direction
  uint8_t inverted = 0;

  // Turn off torque when configuring items in EEPROM area
  dxl.torqueOff(LINEAR_DYNA_ID_1);
  dxl.setOperatingMode(LINEAR_DYNA_ID_1, OP_EXTENDED_POSITION);
  dxl.write(LINEAR_DYNA_ID_1, 36, (uint8_t *)&max_current, sizeof(max_current));  // Write maximum current (register address 38, 2 bytes, ID 1)
  dxl.write(LINEAR_DYNA_ID_1, 10, (uint8_t *)&inverted, sizeof(inverted));        // Write maximum current (register address 38, 2 bytes, ID 1)
  dxl.torqueOn(LINEAR_DYNA_ID_1);

  dxl.torqueOff(LINEAR_DYNA_ID_2);
  dxl.setOperatingMode(LINEAR_DYNA_ID_2, OP_EXTENDED_POSITION);
  dxl.write(LINEAR_DYNA_ID_2, 36, (uint8_t *)&max_current, sizeof(max_current));  // Write maximum current (register address 38, 2 bytes, ID 1)
  dxl.write(LINEAR_DYNA_ID_2, 10, (uint8_t *)&inverted, sizeof(inverted));        // Write maximum current (register address 38, 2 bytes, ID 1)
  dxl.torqueOn(LINEAR_DYNA_ID_2);

  dxl.torqueOff(LINEAR_DYNA_ID_3);
  dxl.setOperatingMode(LINEAR_DYNA_ID_3, OP_EXTENDED_POSITION);
  dxl.write(LINEAR_DYNA_ID_3, 36, (uint8_t *)&max_current, sizeof(max_current));  // Write maximum current (register address 38, 2 bytes, ID 1)
  dxl.write(LINEAR_DYNA_ID_3, 10, (uint8_t *)&inverted, sizeof(inverted));        // Write maximum current (register address 38, 2 bytes, ID 1)
  dxl.torqueOn(LINEAR_DYNA_ID_3);



  // Limit the maximum velocity in Position Control Mode. Use 0 for Max speed
  dxl.writeControlTableItem(PROFILE_VELOCITY, LINEAR_DYNA_ID_1, 600);
  dxl.writeControlTableItem(PROFILE_ACCELERATION, LINEAR_DYNA_ID_1, 25);

  dxl.writeControlTableItem(PROFILE_VELOCITY, LINEAR_DYNA_ID_2, 600);
  dxl.writeControlTableItem(PROFILE_ACCELERATION, LINEAR_DYNA_ID_2, 25);

  dxl.writeControlTableItem(PROFILE_VELOCITY, LINEAR_DYNA_ID_3, 600);
  dxl.writeControlTableItem(PROFILE_ACCELERATION, LINEAR_DYNA_ID_3, 25);



  dyna_1_present_position = dxl.getPresentPosition(LINEAR_DYNA_ID_1);
  Serial.print("dyna 1 initial position:\t");
  Serial.print(dyna_1_present_position);
  goal_position_1 = dyna_1_present_position;
  dxl.setGoalPosition(LINEAR_DYNA_ID_1, goal_position_1);
  
  
  dyna_2_present_position = dxl.getPresentPosition(LINEAR_DYNA_ID_2);
  Serial.print("dyna 2 initial position:\t");
  Serial.print(dyna_2_present_position);
  goal_position_2 = dyna_2_present_position;
  dxl.setGoalPosition(LINEAR_DYNA_ID_2, goal_position_2);
  
  dyna_3_present_position = dxl.getPresentPosition(LINEAR_DYNA_ID_3);
  Serial.print("dyna 3 initial position:\t");
  Serial.print(dyna_3_present_position);
  goal_position_3 = dyna_3_present_position;
  dxl.setGoalPosition(LINEAR_DYNA_ID_3, goal_position_3);
}

void PneumaticGripperController::limitsCalibration() {
  motor_1_position = dxl.getPresentPosition(LINEAR_DYNA_ID_1);
  goal_position_1 = motor_1_position ;
  dxl.setGoalPosition(LINEAR_DYNA_ID_1, goal_position_1);

  motor_2_position = dxl.getPresentPosition(LINEAR_DYNA_ID_2);
  goal_position_2 = motor_2_position ;
  dxl.setGoalPosition(LINEAR_DYNA_ID_2, goal_position_2);

  
  motor_3_position = dxl.getPresentPosition(LINEAR_DYNA_ID_3);
  goal_position_3 = motor_3_position ;
  dxl.setGoalPosition(LINEAR_DYNA_ID_3, goal_position_3);

//##################################################################################################################

//------------------------------>>>>>>> DYNA_1
  delay(500);
  //Start the linear joint limit calibration for dyna_1
  int THRESHOLD_LOAD = 50;
  // Control loop
  do {
    // Increment the goal position
    goal_position_1 -= 15;

    // Set the goal position
    dxl.setGoalPosition(LINEAR_DYNA_ID_1, goal_position_1);

    // Get the present potition
    motor_1_position = dxl.getPresentPosition(LINEAR_DYNA_ID_1);
    Serial.print("\n M1Pos:");
    Serial.print(motor_1_position);

    // Read the motor load
    motor_1_load = GetAbsoluteLoad(LINEAR_DYNA_ID_1, false);
    Serial.print(" M1Load:");
    Serial.println(motor_1_load);
    Serial.print(" threshold:");
    Serial.println(THRESHOLD_LOAD);

  } while (motor_1_load <= THRESHOLD_LOAD);

  delay(500);

  goal_position_1 = motor_1_position - 20;


  // Set the linear joint limits
  int retract_position_1 = motor_1_position;
  linear_end_position_min_1 = goal_position_1;  //maximum retract position
  linear_end_position_max_1 = goal_position_1 + 1300; //maximum relax position

  goal_position_1 = linear_end_position_max_1;
  // Retract the linear joint a little bit

  do {
    retract_position_1 += 10;
    dxl.setGoalPosition(LINEAR_DYNA_ID_1, retract_position_1);
    motor_1_position = dxl.getPresentPosition(LINEAR_DYNA_ID_1);
  } while (motor_1_position < goal_position_1);

  delay(500);
//------------------------------>>>>>>> DYNA_2
  do {
    // Increment the goal position
    goal_position_2 -= 15;

    // Set the goal position
    dxl.setGoalPosition(LINEAR_DYNA_ID_2, goal_position_2);

    // Get the present potition
    motor_2_position = dxl.getPresentPosition(LINEAR_DYNA_ID_2);
    Serial.print("\n M2Pos:");
    Serial.print(motor_2_position);

    // Read the motor load
    motor_2_load = GetAbsoluteLoad(LINEAR_DYNA_ID_2, false);
    Serial.print(" M2Load:");
    Serial.println(motor_2_load);
    Serial.print(" threshold:");
    Serial.println(THRESHOLD_LOAD);

  } while (motor_2_load <= THRESHOLD_LOAD);

  delay(500);

   goal_position_2 = motor_2_position - 20;


  // Set the linear joint limits
  int retract_position_2 = motor_2_position;
  linear_end_position_min_2 = goal_position_2 ;
  linear_end_position_max_2 = goal_position_2 + 1300;

  goal_position_2 = linear_end_position_max_2;
  // Retract the linear joint a little bit
  


  do {
    retract_position_2 += 10;
    dxl.setGoalPosition(LINEAR_DYNA_ID_2, retract_position_2);
    motor_2_position = dxl.getPresentPosition(LINEAR_DYNA_ID_2);
  } while (motor_2_position < goal_position_2);

  delay(500);

//------------------------------>>>>>>> DYNA_3
  do {
    // Increment the goal position
    goal_position_3 -= 15;

    // Set the goal position
    dxl.setGoalPosition(LINEAR_DYNA_ID_3, goal_position_3);

    // Get the present potition
    motor_3_position = dxl.getPresentPosition(LINEAR_DYNA_ID_3);
    Serial.print("\n M2Pos:");
    Serial.print(motor_3_position);

    // Read the motor load
    motor_3_load = GetAbsoluteLoad(LINEAR_DYNA_ID_3, false);
    Serial.print(" M2Load:");
    Serial.println(motor_3_load);
    Serial.print(" threshold:");
    Serial.println(THRESHOLD_LOAD);

  } while (motor_3_load <= THRESHOLD_LOAD);

  delay(500);

   goal_position_3 = motor_3_position - 20;


  // Set the linear joint limits
  int retract_position_3 = motor_3_position;
  linear_end_position_min_3 = goal_position_3 ;
  linear_end_position_max_3 = goal_position_3 + 1300;

  goal_position_3 = linear_end_position_max_3;
  // Retract the linear joint a little bit
  
  do {
    retract_position_3 += 10;
    dxl.setGoalPosition(LINEAR_DYNA_ID_3, retract_position_3);
    motor_3_position = dxl.getPresentPosition(LINEAR_DYNA_ID_3);
  } while (motor_3_position < goal_position_3);

  delay(500);
  
}


int PneumaticGripperController::GetAbsoluteLoad(int id, boolean debug_print) {
  uint16_t motor_current;
  dxl.read(id, 126, 2, (uint8_t *)&motor_current, sizeof(motor_current));

  if (motor_current >= 1023) {
    motor_current = 65535 - motor_current;
  }

  if (debug_print) {
    Serial.print("Load: ");
    if (motor_current >= 0) {
      Serial.print("Clockwise ");
    } else {
      Serial.print("CounterClockwise ");
      motor_current = -motor_current;
    }
    Serial.println(motor_current);
  }

  return motor_current;
}

void PneumaticGripperController::controlLoop() {
  // Your control loop code here
  // ...
}

void PneumaticGripperController::handleButtonStates(ButtonState currentState) {
  switch (currentState) {
    case CW_11DEG:
      if (linear_mode) {
        if (goal_position_1 + 7 < linear_end_position_max_1) {
          goal_position_1 = goal_position_1 + 7;
        }
        if (goal_position_2 + 7 < linear_end_position_max_2) {
          goal_position_2 = goal_position_2 + 7;
        }
        if (goal_position_3 + 7 < linear_end_position_max_3) {
          goal_position_3 = goal_position_3 + 7;
        }
        break;
        case CW_25DEG:
          if (linear_mode) {
            if (goal_position_1 + 70 < linear_end_position_max_1) {
              goal_position_1 = goal_position_1 + 70;
            }
            if (goal_position_2 + 70 < linear_end_position_max_2) {
              goal_position_2 = goal_position_2 + 70;
            }
            if (goal_position_3 + 70 < linear_end_position_max_3) {
              goal_position_3 = goal_position_3 + 70;
            }
          }
          break;
        case PUSH:
          linear_mode = !linear_mode;
          delay(500);
          break;
        case CCW_11DEG:
          if (linear_mode) {
            if (goal_position_1 - 7 > linear_end_position_min_1) {
              goal_position_1 = goal_position_1 - 7;
            }
            if (goal_position_2 - 7 > linear_end_position_min_2) {
              goal_position_2 = goal_position_2 - 7;
            }
            if (goal_position_3 - 7 > linear_end_position_min_3) {
              goal_position_3 = goal_position_3 - 7;
            }
          }
          break;
        case CCW_25DEG:
          if (linear_mode) {
            if (goal_position_1 - 70 > linear_end_position_min_1) {
              goal_position_1 = goal_position_1 - 70;
            }
            if (goal_position_2 - 70 > linear_end_position_min_2) {
              goal_position_2 = goal_position_2 - 70;
            }
            if (goal_position_3 - 70 > linear_end_position_min_3) {
              goal_position_3 = goal_position_3 - 70;
            }
          }
          break;
        case UNKNOWN:
          //do nothing
          break;
      }
  }
}

void PneumaticGripperController::handleTorqueLimitStatus(void) {
  int tolerance = 6;
  motor_1_load = GetAbsoluteLoad(LINEAR_DYNA_ID_1, false);
  dyna_1_present_position = dxl.getPresentPosition(LINEAR_DYNA_ID_1);

  if (motor_1_load > 400 && (dyna_1_present_position > goal_position_1 + tolerance)) {
    Serial.println("stall_avoidance_counter");
    stall_avoidance_counter++;
  }

  if (stall_avoidance_counter > 0) {
    stall_avoidance_counter = 0;
    dxl.setGoalPosition(LINEAR_DYNA_ID_1, dyna_1_present_position + 2);
    delay(200);
  }

  if (stall_avoidance_counter == 0) {
    dxl.setGoalPosition(LINEAR_DYNA_ID_1, goal_position_1);
  }
}

void PneumaticGripperController::updateGoalPositions() {
  // Update goal positions based on current state
  
  dxl.setGoalPosition(LINEAR_DYNA_ID_1, goal_position_1);
  dxl.setGoalPosition(LINEAR_DYNA_ID_2, goal_position_2);
  dxl.setGoalPosition(LINEAR_DYNA_ID_3, goal_position_3);
}

void PneumaticGripperController::printGripperStatus() {
  motor_1_load = GetAbsoluteLoad(LINEAR_DYNA_ID_1, false);
  int motor_1_position = dxl.getPresentPosition(LINEAR_DYNA_ID_1);
  Serial.print("\n M1Load:");
  Serial.print(motor_1_load);
  //Serial.print(" M1:");
  //Serial.print(goal_position_1);
  Serial.print(" M1Actual:");
  Serial.println(motor_1_position);
  
  motor_2_load = GetAbsoluteLoad(LINEAR_DYNA_ID_2, false);
  int motor_2_position = dxl.getPresentPosition(LINEAR_DYNA_ID_2);
  Serial.print("     M2Load:");
  Serial.print(motor_2_load);
 // Serial.print(" M1:");
  //Serial.print(goal_position_1);
  Serial.print(" M2Actual:");
  Serial.println(motor_2_position);

  motor_3_load = GetAbsoluteLoad(LINEAR_DYNA_ID_3, false);
  int motor_3_position = dxl.getPresentPosition(LINEAR_DYNA_ID_3);
  Serial.print("     M3Load:");
  Serial.print(motor_3_load);
  //Serial.print(" M1:");
  //Serial.print(goal_position_1);
  Serial.print(" M3Actual:");
  Serial.println(motor_3_position);
}





void PneumaticGripperController::readBluetoothCommand() {
  int result = -1;
  if (SerialBT.available() >= 5) {  // Wait until there are at least 5 characters available in the serial buffer

    char a = SerialBT.read();
    char b = SerialBT.read();
    char e = SerialBT.read();
    int num1 = a - '0';        // Read the first character and convert it to a number
    int num2 = b - '0';        // Read the second character and convert it to a number
    int num3 = e - '0';        // Read the third character and convert it to a number
    char c = SerialBT.read();  // Read the fourth character (should be '\r')
    char d = SerialBT.read();  // Read the fifth character (should be '\n')

    // Check if the last two characters are "\r\n"
    if (c == '\r' && d == '\n') {
      result = num3;  // Combine the three numbers into an integer

      if (num2 > 0) {
        result += num2 * 10;
      }

      if (num1 > 0) {
        result += num1 * 100;
      }
      // Serial.print("\n Result:\t");
      // Serial.println(map(result, 0, 100, linear_end_position_max, linear_end_position_min));
      goal_position_1 = map(result, 0, 100, linear_end_position_max_1, linear_end_position_min_1);
    
  }
}

}
