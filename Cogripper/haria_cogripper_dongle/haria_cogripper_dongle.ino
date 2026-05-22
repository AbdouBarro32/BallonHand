#include "Arduino.h"
#include "WiFi.h"
#include "pin_config.h"
#include "haria_logo.h"
#include "CoGripperTypes.h"
#include <esp_now.h>

/* external library */
/* To use Arduino, you need to place lv_conf.h in the \Arduino\libraries directory */
#include "TFT_eSPI.h"  // https://github.com/Bodmer/TFT_eSPI
TFT_eSPI tft = TFT_eSPI();


// ESP32 Mac address (first peer)
uint8_t mac_peer1[] = { 0x40, 0x22, 0xD8, 0x7B, 0x5B, 0xC0 };

// esp now protocol peers definition
esp_now_peer_info_t peer1;

// Create an instance of the 'CoGripperCMD' struct
CoGripperCMD coGripperCMD;

// Create an instance of the 'CoGripperData' struct
CoGripperData coGripperData;

// Create an instance of the 'CoGripperPos' struct
CoGripperPos coGripperPos;

bool check_sent = true;
bool check_sent_prec = false;
bool check_received = true;
bool check_received_prec = false;
int check_received_counter = 0;  // Contatore per check_received
bool print_timer_enabled = false;
bool serial_print_flag = false;
char sf_data_str_buf[20];         // Buffer per la stringa formattata
unsigned long lastPrintTime = 0;  // Variabile globale per tracciare l'ultimo tempo di stampa
int start_byte = 0;

QueueHandle_t sendQueue;      // Coda per i messaggi da inviare
TaskHandle_t sendTaskHandle;  // Handle per il task di invio

struct DataPacket {
  uint8_t mac_addr[6];
  uint8_t *data;
  size_t len;
};

String intToStringThreeDigits(int val) {
  // Convert the value to a string
  char formattedString[4];                // Adjusted size for 3 digits
  sprintf(formattedString, "%03d", val);  // Format with three digits
  String str = String(formattedString);

  return str;
}

// Definizione della variabile per il timer hardware
hw_timer_t *timer = NULL;
hw_timer_t *timer1 = NULL;  // Secondo timer

// Prototipo della funzione di callback
void IRAM_ATTR onTimer();
void IRAM_ATTR onTimer1();


// callback when data is received
void onDataReceiver(const uint8_t *mac, const uint8_t *incomingData, int len) {
  memcpy(&coGripperData, incomingData, sizeof(coGripperData));

  // Aggiorna la stringa formattata
  snprintf(sf_data_str_buf, sizeof(sf_data_str_buf), "$%s %s %s %s %c\n",
           intToStringThreeDigits(coGripperData.motor1.load),
           intToStringThreeDigits(coGripperData.motor1.positionPercentage),
           intToStringThreeDigits(coGripperData.motor2.load),
           intToStringThreeDigits(coGripperData.motor2.positionPercentage),
           coGripperData.status == CoGripperStatus::CLOSED ? 'C' : 'O');
  check_received = true;
  check_received_counter++;  // Incrementa il contatore
}


void setup() {
  Serial.begin(115200);
  delay(2000);
  pinMode(TFT_LEDA_PIN, OUTPUT);

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);
  bool verbose_init = false;
  // Get Mac Add
  if (verbose_init) {
    Serial.println();
    Serial.print("Mac Address: ");
    Serial.println(WiFi.macAddress());
  }


  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    if (verbose_init) {
      Serial.println("Error initializing ESP-NOW");
    }
    return;
  } else {
    if (verbose_init) {
      Serial.println("ESP-NOW Initialized!");
    }
  }


  // Configura il timer 0, con prescaler 80 (80 MHz / 80 = 1 MHz -> 1 tick per microsecondo)
  timer = timerBegin(0, 80, true);
  // Attacca la funzione di callback al timer
  timerAttachInterrupt(timer, &onTimer, true);
  // Imposta il timer per generare un interrupt ogni 500000 microsecondi (500 ms)
  timerAlarmWrite(timer, 500000, true);
  // Abilita l'interrupt del timer
  timerAlarmEnable(timer);
  // Configura il secondo timer 1, con prescaler 80 (80 MHz / 80 = 1 MHz -> 1 tick per microsecondo)
  timer1 = timerBegin(1, 80, true);
  // Attacca la funzione di callback al secondo timer
  timerAttachInterrupt(timer1, &onTimer1, true);
  // Imposta il secondo timer per generare un interrupt ogni 30000 microsecondi (30 ms)
  timerAlarmWrite(timer1, 30000, true);
  // Abilita l'interrupt del secondo timer
  timerAlarmEnable(timer1);


  // We can register the receiver callback function
  esp_now_register_recv_cb(onDataReceiver);

  memcpy(peer1.peer_addr, mac_peer1, 6);
  peer1.channel = 1;
  peer1.encrypt = 0;
  // Register the peer
  if (verbose_init) {
    Serial.println("Registering a peer 1");
  }
  if (esp_now_add_peer(&peer1) == ESP_OK) {
    if (verbose_init) {
      Serial.println("Peer 1 added");
    }
  }

  // Initialise TFT
  tft.init();
  tft.setRotation(3);
  tft.fillScreen(TFT_DARKGREY);
  digitalWrite(TFT_LEDA_PIN, 0);
  tft.setTextFont(1);
  tft.setTextColor(TFT_BLACK, TFT_WHITE);

  // Draw HARIA logo
  tft.pushImage(0, 0, 160, 80, (uint16_t *)gHARIA_logo);
  tft.drawString("CoGrip", 120, 0, 2);
  tft.drawString("v1", 120, 13, 2);

  drawInDataArrow(true);

  // Crea la coda per i pacchetti da inviare
  sendQueue = xQueueCreate(10, sizeof(DataPacket));

  // Crea il task per l'invio dei dati
  xTaskCreate(sendTask, "SendTask", 4096, NULL, 1, &sendTaskHandle);
}

void loop() {
  int bytes_available = Serial.available();
  // if (bytes_available > 5)  //  wait for 6 bytes
  if (bytes_available > 11)  //  wait for 6 bytes
  {
    start_byte = (byte)(Serial.read());
    if (start_byte == '$')  //  36 is the dec for '$' ASCII character - START
    {
      char first_byte = (byte)(Serial.read());
      if (first_byte == 'O' || first_byte == 'C' || first_byte == 'S' || first_byte == 'V') {
        char second_byte = (byte)(Serial.read());
        coGripperCMD.cmd = first_byte;
        coGripperCMD.aux = second_byte;
        enqueueCoGripperCMD();
        for (int i = 0; i < 8; i++) {
          Serial.read();
        }
      } else if (first_byte == 'N') {
        coGripperPos.motor1_pos = Serial.parseInt();
        coGripperPos.motor2_pos = Serial.parseInt();
        enqueueCoGripperPos();
      }
      // tft.drawString(String(String(start_byte) + String(first_byte)), 0, 13, 2);
      // tft.drawString(String("           "), 0, 26, 2);
      // tft.drawString(String("Bytes: " + String(bytes_available)), 0, 26, 2);
    }
  }

  // Stampa la stringa formattata
  if (check_received && serial_print_flag) {
    Serial.print(sf_data_str_buf);
    serial_print_flag = false;
  }
}

// Funzione per inserire il messaggio CoGripperCMD nella coda
void enqueueCoGripperCMD() {
  DataPacket packet;
  memcpy(packet.mac_addr, peer1.peer_addr, 6);
  packet.data = (uint8_t *)malloc(sizeof(coGripperCMD));
  memcpy(packet.data, &coGripperCMD, sizeof(coGripperCMD));
  packet.len = sizeof(coGripperCMD);
  if (xQueueSend(sendQueue, &packet, portMAX_DELAY) != pdTRUE) {
    // Gestisci l'errore se necessario
    free(packet.data);
  }
}

// Funzione per inserire il messaggio CoGripperPos nella coda
void enqueueCoGripperPos() {
  DataPacket packet;
  memcpy(packet.mac_addr, peer1.peer_addr, 6);
  packet.data = (uint8_t *)malloc(sizeof(coGripperPos));
  memcpy(packet.data, &coGripperPos, sizeof(coGripperPos));
  packet.len = sizeof(coGripperPos);
  if (xQueueSend(sendQueue, &packet, portMAX_DELAY) != pdTRUE) {
    // Gestisci l'errore se necessario
    free(packet.data);
  }
}

// Task per l'invio dei dati
void sendTask(void *pvParameter) {
  DataPacket packet;
  while (true) {
    // Prendi un pacchetto dalla coda
    if (xQueueReceive(sendQueue, &packet, portMAX_DELAY) == pdTRUE) {
      // Invia i dati
      esp_err_t result = esp_now_send(packet.mac_addr, packet.data, packet.len);
      // Imposta la variabile check_sent se l'invio è avvenuto correttamente
      if (result == ESP_OK) {
        check_sent = true;
      }
      // Libera il buffer dei dati
      free(packet.data);
    }
    // Delay per limitare la frequenza di invio a circa 15 Hz
    vTaskDelay(pdMS_TO_TICKS(67));
  }
}


void drawOutDataArrow(bool en) {
  if (en) {
    //outgoing data arrow
    tft.fillRect(120, 63 - 1, 30, 3, TFT_DARKGREEN);
    tft.fillTriangle(120 + 30 - 5, 63 - 5 - 1, 120 + 30 - 5, 63 + 2 - 1, 120 + 30 + 2, 63 + 2 - 1, TFT_DARKGREEN);
  } else {
    //outgoing data arrow
    tft.fillRect(120, 63 - 1, 30, 3, TFT_WHITE);
    tft.fillTriangle(120 + 30 - 5, 63 - 5 - 1, 120 + 30 - 5, 63 + 2 - 1, 120 + 30 + 2, 63 + 2 - 1, TFT_WHITE);
  }
}

void drawInDataArrow(bool en) {
  //incoming data arrow
  tft.fillRect(123, 67, 30, 3, en ? TFT_DARKGREEN : TFT_WHITE);
  tft.fillTriangle(128, 74, 128, 67, 121, 67, en ? TFT_DARKGREEN : TFT_WHITE);
}

// Funzione di callback eseguita ogni 500 ms
void IRAM_ATTR onTimer() {
  if (check_received) {
    if (!check_received_prec) {
      drawInDataArrow(true);
    }
    if (check_received_counter == 0) {
      check_received_prec = check_received;
      check_received = false;
    }
  } else {
    if (!check_received_prec) {
      drawInDataArrow(false);
    }
    check_received_prec = check_received;
    check_received = false;
  }

  if (check_sent) {
    if (!check_sent_prec) {
      drawOutDataArrow(true);
    }
    check_sent_prec = check_sent;
    check_sent = false;
  } else {
    if (!check_sent_prec) {
      drawOutDataArrow(false);
    }
    check_sent_prec = check_sent;
    check_sent = false;
  }

  check_received_counter = 0;
}

void IRAM_ATTR onTimer1() {
  serial_print_flag = true;
}