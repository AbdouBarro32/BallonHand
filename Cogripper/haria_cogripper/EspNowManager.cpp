#include "EspNowManager.h"

/**
 * @brief Costruttore: copia l'indirizzo MAC del peer e imposta canale e crittografia.
 */
EspNowManager::EspNowManager(const uint8_t* peerMac, uint8_t channel, bool encrypt)
  : _channel(channel), _encrypt(encrypt), _sendTaskHandle(NULL) {
  memcpy(_peerMac, peerMac, 6);
}

/**
 * @brief Inizializza ESP-NOW e imposta la modalità WiFi in Station.
 */
bool EspNowManager::begin() {
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return false;
  }
  return true;
}

/**
 * @brief Registra i callback per l'invio e la ricezione dei dati.
 */
void EspNowManager::registerCallbacks(void (*sendCb)(const uint8_t*, esp_now_send_status_t),
                                        void (*recvCb)(const esp_now_recv_info_t*, const uint8_t*, int)) {
  esp_now_register_send_cb(sendCb);
  esp_now_register_recv_cb(recvCb);
}

/**
 * @brief Aggiunge il peer ESP-NOW utilizzando le informazioni configurate.
 */
bool EspNowManager::addPeer() {
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, _peerMac, 6);
  peerInfo.channel = _channel;
  peerInfo.encrypt = _encrypt;
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add ESP-NOW peer");
    return false;
  }
  return true;
}

/**
 * @brief Invia i dati specificati al peer.
 */
void EspNowManager::sendData(const uint8_t* data, size_t len) {
  esp_now_send(_peerMac, data, len);
}

/**
 * @brief Task che invia periodicamente i dati tramite ESP-NOW.
 *
 * @param param Puntatore all'istanza di EspNowManager.
 */
void sendTask(void* param) {
  EspNowManager* manager = static_cast<EspNowManager*>(param);
  for (;;) {
    // Inserisci qui la logica per inviare periodicamente i dati
    // manager->sendData(...);
    vTaskDelay(pdMS_TO_TICKS(30));  // Intervallo di 30 ms
  }
}

/**
 * @brief Avvia il task per l'invio periodico dei dati.
 */
void EspNowManager::startSendTask() {
  xTaskCreate(sendTask, "EspNowSendTask", 2048, this, 1, &_sendTaskHandle);
}

/**
 * @brief Ferma il task per l'invio periodico dei dati.
 */
void EspNowManager::stopSendTask() {
  if (_sendTaskHandle != NULL) {
    vTaskDelete(_sendTaskHandle);
    _sendTaskHandle = NULL;
  }
}
