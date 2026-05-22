#ifndef ESPNOWMANAGER_H
#define ESPNOWMANAGER_H

#include <esp_now.h>
#include <WiFi.h>

/**
 * @brief Classe per la gestione delle comunicazioni ESP-NOW.
 *
 * Questa classe incapsula la logica di inizializzazione, registrazione dei callback,
 * aggiunta dei peer e gestione dell'invio periodico dei dati tramite ESP-NOW.
 */
class EspNowManager {
public:
  /**
   * @brief Costruttore della classe EspNowManager.
   * @param peerMac Indirizzo MAC del peer con cui comunicare.
   * @param channel Canale WiFi da utilizzare (default 1).
   * @param encrypt Flag per abilitare la crittografia (default false).
   */
  EspNowManager(const uint8_t* peerMac, uint8_t channel = 1, bool encrypt = false);

  /**
   * @brief Inizializza ESP-NOW.
   * @return true se l'inizializzazione ha avuto successo, false altrimenti.
   */
  bool begin();

  /**
   * @brief Registra i callback per l'invio e la ricezione dei dati.
   * @param sendCb Callback chiamato dopo l'invio dei pacchetti.
   * @param recvCb Callback chiamato alla ricezione dei dati.
   */
  void registerCallbacks(void (*sendCb)(const uint8_t*, esp_now_send_status_t),
                         void (*recvCb)(const esp_now_recv_info_t*, const uint8_t*, int));

  /**
   * @brief Aggiunge un peer ESP-NOW.
   * @return true se il peer è stato aggiunto con successo, false altrimenti.
   */
  bool addPeer();

  /**
   * @brief Invia dei dati al peer.
   * @param data Puntatore ai dati da inviare.
   * @param len Lunghezza dei dati.
   */
  void sendData(const uint8_t* data, size_t len);

  /**
   * @brief Avvia il task per l'invio periodico dei dati.
   */
  void startSendTask();

  /**
   * @brief Ferma il task per l'invio periodico dei dati.
   */
  void stopSendTask();

private:
  uint8_t _peerMac[6];   ///< Indirizzo MAC del peer
  uint8_t _channel;      ///< Canale WiFi utilizzato
  bool _encrypt;         ///< Flag per la crittografia
  TaskHandle_t _sendTaskHandle;  ///< Handle del task per l'invio periodico
};

#endif  // ESPNOWMANAGER_H
