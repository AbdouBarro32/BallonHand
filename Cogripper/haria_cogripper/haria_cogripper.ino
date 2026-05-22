#include <Dynamixel2Arduino.h>
#include <HardwareSerial.h>

#include "SLLB120100.h"
#include "CoGripperController.h"

#include "EspNowManager.h" 
#include <WiFi.h>
#include <esp_now.h>


#define dynaSerial_TX_pin 17
#define dynaSerial_RX_pin 16
const int DXL_DIR_PIN = 21;  // DYNAMIXEL Shield DIR PIN
CoGripperController controller(dynaSerial_TX_pin, dynaSerial_RX_pin, DXL_DIR_PIN);
QueueHandle_t commandQueue; // Handle per la coda di comandi
const int COMMAND_QUEUE_LENGTH = 10;

#define DEBUG_SERIAL Serial

SLLB120100 buttonController;

// Define a struct named 'CoGripperCMD'
struct CoGripperCMD {
  char cmd = 'O';
  char aux = 'O';
};

struct CoGripperPos {
  int motor1_pos;
  int motor2_pos;
  int motor3_pos;
};

// Create an instance of the 'CoGripperCMD' struct named 'cogripperCMD'
CoGripperCMD cogripperCMD;

// Create an instance of the 'CoGripperPos' struct
CoGripperPos coGripperPos;




// ESP32 Mac address (first peer)
uint8_t mac_peer1[] = { 0xDC, 0xDA, 0x0C, 0x30, 0xAD, 0x90 };
EspNowManager espNowManager(mac_peer1, 1, false); 
// esp now protocol peers definition
esp_now_peer_info_t peer1;

// Definizione della variabile per il timer hardware
hw_timer_t *timer = NULL;

// Flag per servire la callback di ricezione
bool service_CMD = false;

// callback when data is sent
void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  // Per debug, se necessario:
  Serial.print("Packet send status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Success" : "Fail");
}

// callback when data is received
void onDataReceiver(const esp_now_recv_info_t *recv_info, const uint8_t *incomingData, int len) {

  if (len == sizeof(CoGripperCMD)) {
    CoGripperCMD receivedCmd;
    memcpy(&receivedCmd, incomingData, sizeof(receivedCmd));
    
    // Invia il comando alla coda. Non bloccare se la coda è piena.
    xQueueSendFromISR(commandQueue, &receivedCmd, NULL);
    Serial.print("Comando ricevuto: ");
    Serial.println(cogripperCMD.cmd);
    //ervice_CMD = true;

  } else if (len == sizeof(CoGripperPos)) {
    memcpy(&coGripperPos, incomingData, len);
    coGripperPos.motor1_pos = intConstrain(coGripperPos.motor1_pos, 0, 2400);
    coGripperPos.motor2_pos = intConstrain(coGripperPos.motor2_pos, 0, 2400);
    coGripperPos.motor3_pos = intConstrain(coGripperPos.motor3_pos, 0, 2400);
    // Serial.print("\ncommand1: ");
    // Serial.print(coGripperPos.motor1_pos);
    // Serial.print("\ncommand2: ");
    // Serial.println(coGripperPos.motor2_pos);
    controller.setGoalPositionMotor1(map(coGripperPos.motor1_pos, 0, 2400, controller.getMotor1EndPositionMin(), controller.getMotor1EndPositionMax()));
    controller.setGoalPositionMotor2(map(coGripperPos.motor2_pos, 0, 2400, controller.getMotor2EndPositionMin(), controller.getMotor2EndPositionMax()));
    controller.setGoalPositionMotor3(map(coGripperPos.motor3_pos, 0, 2400, controller.getMotor3EndPositionMin(), controller.getMotor3EndPositionMax()));  // Added for motor3
  } else {
    Serial.println("Messaggio di lunghezza non supportata ricevuto");
  }
}

// Funzione di callback eseguita ogni 30 ms
void espNowSendTask(void *pvParameters) {
  for (;;) {
    CoGripperData dataToSend = controller.getData();
    espNowManager.sendData((uint8_t *)&dataToSend, sizeof(dataToSend));
    vTaskDelay(pdMS_TO_TICKS(30));
  }
}







void setup() {
  // Setup serial communication with USB cable (for debug purposes)
  Serial.begin(57600);
  delay(2000);

  //pinMode(LED_BUILTIN, OUTPUT);
  //digitalWrite(LED_BUILTIN, LOW);

  commandQueue = xQueueCreate(COMMAND_QUEUE_LENGTH, sizeof(CoGripperCMD));

  if (commandQueue == NULL) {
    Serial.println("Errore nella creazione della coda!");
    while(1);
  }

  // Controlla se la libreria WiFi è inclusa

  Serial.println("\nUSING ESP NOW!");

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);
  // Get Mac Add
  Serial.println();
  Serial.print("Mac Address: ");
  Serial.println(WiFi.macAddress());


  // Init ESP-NOW
  if (!espNowManager.begin()) {
    Serial.println("Error initializing ESP‑NOW");
    return;
  } else {
    Serial.println("ESP‑NOW Initialized!");
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet

  // Registra le callback di invio e ricezione tramite EspNowManager
  espNowManager.registerCallbacks(onDataSent, onDataReceiver);

  // Aggiunge il peer
  if (!espNowManager.addPeer()) {
    Serial.println("Failed to add ESP‑NOW peer");
  } else {
    Serial.println("Peer added");
  }

  // Crea il task FreeRTOS per inviare periodicamente i dati via ESP‑NOW
  xTaskCreate(espNowSendTask, "espNowSendTask", 2048, NULL, 1, NULL);


  //timer = timerBegin(0, 80, true);              // Configura il timer 0, con prescaler 80 (80 MHz / 80 = 1 MHz -> 1 tick per microsecondo)
  //timerAttachInterrupt(timer, &onTimer, true);  // Attacca la funzione di callback al timer
  //timerAlarmWrite(timer, 30000, true);          // Imposta il timer per generare un interrupt ogni 30000 microsecondi (300 ms)
  //timerAlarmEnable(timer);                      // Abilita l'interrupt del timer



  controller.initialize();
  controller.limitsCalibration();
  controller.setGripperMaxPWM(375);
}



void loop() {
  ButtonState currentState = buttonController.getButtonStates(false);
  controller.handleButtonStates(currentState);
  // if (service_CMD) {
  //   serviceCMD();
  //   service_CMD = false;
  // }
  CoGripperCMD receivedCmd;
  if (xQueueReceive(commandQueue, &receivedCmd, 0) == pdPASS) {
    // Se abbiamo ricevuto un comando, processiamolo
    // La vecchia funzione serviceCMD() ora riceve il comando come argomento
    processCMD(receivedCmd); 
  }

  controller.controlLoop();
  delay(2);
}

int intConstrain(int value, int min, int max) {
  if (value < min) {
    return min;
  }
  if (value > max) {
    return max;
  } else {
    return value;
  }
}
void processCMD(const CoGripperCMD& cmdToProcess) {
  if (cmdToProcess.cmd == 'C') {
    if (cmdToProcess.aux == '1') {
      controller.motor1.setGoalPosition(controller.motor1.getEndPositionMin());
      Serial.println("CLOSE 1");
    } else if (cmdToProcess.aux == '2') {
      controller.motor2.setGoalPosition(controller.motor2.getEndPositionMin());
      Serial.println("CLOSE 2");
    } else if (cmdToProcess.aux == '3') {
      controller.motor3.setGoalPosition(controller.motor3.getEndPositionMin());
      Serial.println("CLOSE 3");
    } else {
      Serial.println("CLOSE ALL");
      controller.closeAllMotors();
    }
  }

  if (cmdToProcess.cmd == 'O') {
    if (cmdToProcess.aux == '1') {
      controller.motor1.setGoalPosition(controller.motor1.getEndPositionMax());
      Serial.println("OPEN 1");
    } else if (cmdToProcess.aux == '2') {
      controller.motor2.setGoalPosition(controller.motor2.getEndPositionMax());
      Serial.println("OPEN 2");
    } else if (cmdToProcess.aux == '3') {
      controller.motor3.setGoalPosition(controller.motor3.getEndPositionMax());
      Serial.println("OPEN 3");
    } else {
      Serial.println("OPEN ALL, BIASCICA!");
      controller.openAllMotors();
    }
  }
  if (cmdToProcess.cmd == 'S') {
    if (cmdToProcess.aux == '1') {
      controller.motor1.setGoalPosition(controller.motor1.getCurrentPosition());
      Serial.println("STOP 1");
    } else if (cmdToProcess.aux == '2') {
      controller.motor2.setGoalPosition(controller.motor2.getCurrentPosition());
      Serial.println("STOP 2");
    } else if (cmdToProcess.aux == '3') {
      controller.motor3.setGoalPosition(controller.motor3.getCurrentPosition());
      Serial.println("STOP 3");
    } else {
      Serial.println("STOP ALL");
      controller.stopAllMotors();
    }
  }
  if (cmdToProcess.cmd == 'V') {
    if (cmdToProcess.aux == 'S') {
      Serial.println("SLOW");
      //timerAlarmDisable(timer);
      delay(50);
      controller.motor1.changeMotorVelocity(LOW);
      delay(50);
      controller.motor2.changeMotorVelocity(LOW);
      delay(50);
      controller.motor3.changeMotorVelocity(LOW);
      //timerAlarmEnable(timer);
    }
    if (cmdToProcess.aux == 'F') {
      Serial.println("FAST");
      //timerAlarmDisable(timer);
      delay(50);
      controller.motor1.changeMotorVelocity(HIGH);
      delay(50);
      controller.motor2.changeMotorVelocity(HIGH);
      delay(50);
      controller.motor3.changeMotorVelocity(HIGH);
      //timerAlarmEnable(timer);
    }
  }
}

// void serviceCMD() {
//   if (cogripperCMD.cmd == 'C') {
//     if (cogripperCMD.aux == '1') {
//       controller.motor1.setGoalPosition(controller.motor1.getEndPositionMin());
//       Serial.println("CLOSE 1");
//     } else if (cogripperCMD.aux == '2') {
//       controller.motor2.setGoalPosition(controller.motor2.getEndPositionMin());
//       Serial.println("CLOSE 2");
//     } else if (cogripperCMD.aux == '3') {
//       controller.motor3.setGoalPosition(controller.motor3.getEndPositionMin());
//       Serial.println("CLOSE 3");
//     } else {
//       Serial.println("CLOSE ALL");
//       controller.closeAllMotors();
//     }
//   }

//   if (cogripperCMD.cmd == 'O') {
//     if (cogripperCMD.aux == '1') {
//       controller.motor1.setGoalPosition(controller.motor1.getEndPositionMax());
//       Serial.println("OPEN 1");
//     } else if (cogripperCMD.aux == '2') {
//       controller.motor2.setGoalPosition(controller.motor2.getEndPositionMax());
//       Serial.println("OPEN 2");
//     } else if (cogripperCMD.aux == '3') {
//       controller.motor3.setGoalPosition(controller.motor3.getEndPositionMax());
//       Serial.println("OPEN 3");
//     } else {
//       Serial.println("OPEN ALL, BIASCICA!");
//       controller.openAllMotors();
//     }
//   }
//   if (cogripperCMD.cmd == 'S') {
//     if (cogripperCMD.aux == '1') {
//       controller.motor1.setGoalPosition(controller.motor1.getCurrentPosition());
//       Serial.println("STOP 1");
//     } else if (cogripperCMD.aux == '2') {
//       controller.motor2.setGoalPosition(controller.motor2.getCurrentPosition());
//       Serial.println("STOP 2");
//     } else if (cogripperCMD.aux == '3') {
//       controller.motor3.setGoalPosition(controller.motor3.getCurrentPosition());
//       Serial.println("STOP 3");
//     } else {
//       Serial.println("STOP ALL");
//       controller.stopAllMotors();
//     }
//   }
//   if (cogripperCMD.cmd == 'V') {
//     if (cogripperCMD.aux == 'S') {
//       Serial.println("SLOW");
//       //timerAlarmDisable(timer);
//       delay(50);
//       controller.motor1.changeMotorVelocity(LOW);
//       delay(50);
//       controller.motor2.changeMotorVelocity(LOW);
//       delay(50);
//       controller.motor3.changeMotorVelocity(LOW);
//       //timerAlarmEnable(timer);
//     }
//     if (cogripperCMD.aux == 'F') {
//       Serial.println("FAST");
//       //timerAlarmDisable(timer);
//       delay(50);
//       controller.motor1.changeMotorVelocity(HIGH);
//       delay(50);
//       controller.motor2.changeMotorVelocity(HIGH);
//       delay(50);
//       controller.motor3.changeMotorVelocity(HIGH);
//       //timerAlarmEnable(timer);
//     }
//   }
// }
// if (cogripperCMD.cmd == 'L') {
//   Serial.println("SLOW");
//   timerAlarmDisable(timer);
//   delay(50);
//   controller.motor1.changeMotorVelocity(LOW);
//   delay(50);
//   controller.motor2.changeMotorVelocity(LOW);
//   timerAlarmEnable(timer);
// }
// if (cogripperCMD.cmd == 'F') {
//   Serial.println("FAST");
//   timerAlarmDisable(timer);
//   delay(50);
//   controller.motor1.changeMotorVelocity(HIGH);
//   delay(50);
//   controller.motor2.changeMotorVelocity(HIGH);
//   timerAlarmEnable(timer);
//