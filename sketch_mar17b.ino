// esp32_robot.ino — DRV8833
#include <esp_now.h>
#include <WiFi.h>

// ========== CONFIGURAZIONE PIN DRV8833 ==========
#define SLEEP 32   // SLEEP — HIGH = driver attivo

// Motore A (ruota sinistra)
#define AIN1  14   // IN1 modulo
#define AIN2  27   // IN2 modulo

// Motore B (ruota destra)
#define BIN1  26   // IN3 modulo
#define BIN2  25   // IN4 modulo

#define VELOCITA           255   // 0–255
#define VELOCITA_ROTAZIONE 255   // 0–255
#define TRIM_SINISTRA    1.00f
#define TRIM_DESTRA      1.00f

// ========== FUNZIONI MOTORI ==========
// DRV8833: PWM su primo pin + LOW su secondo = avanti
//          LOW su primo + PWM su secondo    = indietro
//          LOW, LOW                         = coast (libero)
//          HIGH, HIGH                       = brake (freno attivo)

void stopMotori() {
    analogWrite(AIN1, 0); analogWrite(AIN2, 0);
    analogWrite(BIN1, 0); analogWrite(BIN2, 0);
}

void avanti() {
    analogWrite(AIN1, (int)(VELOCITA * TRIM_SINISTRA)); analogWrite(AIN2, 0);
    analogWrite(BIN1, (int)(VELOCITA * TRIM_DESTRA));   analogWrite(BIN2, 0);
}

void indietro() {
    analogWrite(AIN1, 0); analogWrite(AIN2, (int)(VELOCITA * TRIM_SINISTRA));
    analogWrite(BIN1, 0); analogWrite(BIN2, (int)(VELOCITA * TRIM_DESTRA));
}

void rotazioneSinistra() {
    analogWrite(AIN1, (int)(VELOCITA_ROTAZIONE * TRIM_SINISTRA)); analogWrite(AIN2, 0);
    analogWrite(BIN1, 0); analogWrite(BIN2, (int)(VELOCITA_ROTAZIONE * TRIM_DESTRA));
}

void rotazioneDestra() {
    analogWrite(AIN1, 0); analogWrite(AIN2, (int)(VELOCITA_ROTAZIONE * TRIM_SINISTRA));
    analogWrite(BIN1, (int)(VELOCITA_ROTAZIONE * TRIM_DESTRA)); analogWrite(BIN2, 0);
}

// ========== ESP-NOW CALLBACK ==========
void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
    if (len < 1) return;
    char cmd = (char)data[0];

    Serial.print("[CMD] Ricevuto: "); Serial.println(cmd);

    switch (cmd) {
        case 'F': avanti();            break;
        case 'B': indietro();          break;
        case 'L': rotazioneSinistra(); break;
        case 'R': rotazioneDestra();   break;
        case 'S': stopMotori();        break;
        default:  Serial.println("[WARN] Comando sconosciuto"); break;
    }

    // Serial2.write(cmd);  // ← decommenta per forwardare all'Arduino R4
}

// ========== SETUP ==========
void setup() {
    Serial.begin(115200);

    pinMode(SLEEP, OUTPUT);
    pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
    pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);

    digitalWrite(SLEEP, HIGH);  // Abilita il driver
    stopMotori();

    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    delay(100);

    Serial.print("[INFO] MAC Ricevitore: ");
    Serial.println(WiFi.macAddress());  // ← COPIA QUESTO nel trasmettitore!

    if (esp_now_init() != ESP_OK) {
        Serial.println("[ERRORE] ESP-NOW non inizializzato");
        return;
    }
    esp_now_register_recv_cb(onDataRecv);
    Serial.println("[OK] Robot in ascolto...");
}

void loop() {
    // Tutto gestito nel callback ESP-NOW
}