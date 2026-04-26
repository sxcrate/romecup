// esp32_trasmettitore.ino
#include <esp_now.h>
#include <WiFi.h>

// ⚠️  Sostituisci con il MAC del ricevitore (lo trovi nel monitor seriale del ricevitore)
uint8_t RECEIVER_MAC[] = {0xF0, 0x24, 0xF9, 0x43, 0x5E, 0x43};

esp_now_peer_info_t peerInfo;
bool espNowReady = false;

void onDataSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {
    Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Inviato OK" : "Errore invio");
}

void setup() {
    Serial.begin(115200);
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();

    if (esp_now_init() != ESP_OK) {
        Serial.println("[ERRORE] Inizializzazione ESP-NOW fallita");
        return;
    }
    esp_now_register_send_cb(onDataSent);

    memcpy(peerInfo.peer_addr, RECEIVER_MAC, 6);
    peerInfo.channel  = 0;
    peerInfo.encrypt  = false;

    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("[ERRORE] Aggiunta peer fallita");
        return;
    }

    espNowReady = true;
    Serial.println("[OK] Trasmettitore pronto");
    Serial.print("[INFO] MAC trasmettitore: ");
    Serial.println(WiFi.macAddress());
}

void loop() {
    if (!espNowReady) return;

    if (Serial.available() > 0) {
        char cmd = Serial.read();
        if (cmd == 'F' || cmd == 'B' || cmd == 'L' || cmd == 'R' || cmd == 'S') {
            esp_now_send(RECEIVER_MAC, (uint8_t*)&cmd, 1);
        }
    }
}
