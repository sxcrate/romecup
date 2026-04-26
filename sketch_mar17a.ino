// ============================================================
//  esp32_trasmettitore.ino  —  v2
//  Funge da bridge tra ESP32-CAM e PC, e tra PC e Robot.
//
//  Flusso:
//    CAM  ──(ESP-NOW chunk)──►  [questo]  ──(Serial bin)──►  PC
//    PC   ──(Serial ASCII)──►  [questo]  ──(ESP-NOW 1B)──►  Robot
//
//  Protocollo Serial verso PC:
//    ┌──────────────────────────────────────────────────┐
//    │  0xAA  0xBB  │  len_hi  len_lo  │  JPEG...  │  0xCC  0xDD  │
//    └──────────────────────────────────────────────────┘
//    Lo script Python legge SOF (0xAA 0xBB), poi 2 byte lunghezza,
//    poi i byte del JPEG, poi EOF (0xCC 0xDD).
//
//  ⚠️  Usa Serial a 921600 baud — aggiorna il baud anche nel
//      tuo script Python (es. serial.Serial('/dev/cu.xxx', 921600))
// ============================================================

#include <esp_now.h>
#include <WiFi.h>

// ──────────────────────────────────────────────────────────
//  ⚠️  CONFIGURA QUESTI PRIMA DI CARICARE
// ──────────────────────────────────────────────────────────

// MAC del ricevitore (robot) — invariato dalla v1
uint8_t RECEIVER_MAC[] = {0xF0, 0x24, 0xF9, 0x43, 0x5E, 0x8C};

// MAC della CAM (leggi dal suo monitor seriale dopo aver caricato esp32_cam.ino)
// Serve solo per il log; il ricevitore accetta messaggi da qualsiasi peer.
// Se vuoi filtrare per sicurezza, decommenta la sezione "Aggiungi peer CAM" nel setup.
uint8_t CAM_MAC[] = {0xXX, 0xXX, 0xXX, 0xXX, 0xXX, 0xXX};

// ──────────────────────────────────────────────────────────
//  PROTOCOLLO SERIALE  (deve corrispondere a quello in Python)
// ──────────────────────────────────────────────────────────
#define SOF_0   0xAA
#define SOF_1   0xBB
#define EOF_0   0xCC
#define EOF_1   0xDD

// ──────────────────────────────────────────────────────────
//  STRUTTURA CHUNK  (identica all'esp32_cam.ino)
// ──────────────────────────────────────────────────────────
#define CHUNK_DATA_SIZE   240
#define MAX_CHUNKS        128

struct __attribute__((packed)) ImageChunk {
    uint8_t  frame_id;
    uint8_t  chunk_idx;
    uint8_t  total_chunks;
    uint8_t  data_len;
    uint8_t  data[CHUNK_DATA_SIZE];
};

// ──────────────────────────────────────────────────────────
//  BUFFER RIASSEMBLAGGIO FRAME
//  12 KB copre QQVGA JPEG con abbondante margine.
//  Se usi FRAMESIZE_QVGA (320×240) aumenta a 20480.
// ──────────────────────────────────────────────────────────
#define MAX_FRAME_SIZE    12288

static uint8_t  frame_buf[MAX_FRAME_SIZE];
static uint16_t frame_size      = 0;     // byte totali del frame corrente
static uint8_t  cur_frame_id    = 0;
static uint8_t  cur_total       = 0;     // chunk attesi per il frame corrente
static uint8_t  chunks_received = 0;

// Flag volatile: scritto nel callback ESP-NOW (task WiFi),
// letto nel loop() (task Arduino)
static volatile bool frame_pending = false;

bool espNowReady = false;

// ──────────────────────────────────────────────────────────
//  CALLBACK: invio confermato (verso ricevitore/robot)
// ──────────────────────────────────────────────────────────
void onDataSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {
    // Decommenta per debug, ma genera traffico seriale che interferisce col video:
    // Serial.println(status == ESP_NOW_SEND_SUCCESS ? "[TX] CMD ok" : "[TX] CMD errore");
}

// ──────────────────────────────────────────────────────────
//  CALLBACK: chunk ricevuto dalla CAM
//  ⚠️  Questo callback gira nel task WiFi, NON nel loop().
//      Non chiamare Serial.print qui dentro.
// ──────────────────────────────────────────────────────────
void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
    // Ignora pacchetti troppo corti per essere un ImageChunk valido
    if (len < 5) return;

    const ImageChunk *pkt = (const ImageChunk*)data;

    // ── Nuovo frame (cambio frame_id o buffer resettato) ──
    if (pkt->frame_id != cur_frame_id || chunks_received == 0) {
        // Se c'era un frame in corso non completato, lo scartiamo silenziosamente
        cur_frame_id    = pkt->frame_id;
        cur_total       = pkt->total_chunks;
        chunks_received = 0;
        frame_size      = 0;
        frame_pending   = false;
    }

    // ── Scrivi chunk nel buffer nella posizione corretta ──
    uint32_t offset = (uint32_t)pkt->chunk_idx * CHUNK_DATA_SIZE;
    if (offset + pkt->data_len > MAX_FRAME_SIZE) return;  // overflow: scarta

    memcpy(frame_buf + offset, pkt->data, pkt->data_len);
    chunks_received++;

    // Aggiorna dimensione effettiva del frame (funziona anche se i chunk
    // arrivano fuori ordine, perché teniamo il massimo)
    uint32_t end = offset + pkt->data_len;
    if (end > frame_size) frame_size = (uint16_t)end;

    // ── Frame completo? ──
    if (chunks_received >= cur_total) {
        frame_pending   = true;
        chunks_received = 0;   // resetta subito per il prossimo frame
    }
}

// ──────────────────────────────────────────────────────────
//  Invia il frame JPEG completo al PC via Serial binario
//  Protocollo: SOF(2) + len(2) + dati(N) + EOF(2)
// ──────────────────────────────────────────────────────────
void sendFrameToPC() {
    uint16_t len = frame_size;
    Serial.write(SOF_0);
    Serial.write(SOF_1);
    Serial.write((uint8_t)(len >> 8));
    Serial.write((uint8_t)(len & 0xFF));
    Serial.write(frame_buf, len);
    Serial.write(EOF_0);
    Serial.write(EOF_1);

    frame_pending = false;
    frame_size    = 0;
}

// ──────────────────────────────────────────────────────────
//  SETUP
// ──────────────────────────────────────────────────────────
void setup() {
    // ⚠️  921600 baud per non fare da bottleneck al video.
    //     Aggiorna il baud nel tuo script Python di conseguenza.
    Serial.begin(921600);

    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    delay(100);

    if (esp_now_init() != ESP_OK) {
        // Non possiamo usare Serial.println qui senza sporcare il canale binario.
        // Se stai debuggando il setup, abbassa il baud a 115200 temporaneamente.
        while (true) delay(1000);
    }

    esp_now_register_send_cb(onDataSent);
    esp_now_register_recv_cb(onDataRecv);

    // ── Aggiungi peer: ricevitore (robot) ──
    esp_now_peer_info_t peerRX = {};
    memcpy(peerRX.peer_addr, RECEIVER_MAC, 6);
    peerRX.channel = 0;
    peerRX.encrypt = false;
    if (esp_now_add_peer(&peerRX) != ESP_OK) {
        while (true) delay(1000);
    }

    // ── (Opzionale) Aggiungi peer: CAM — utile se vuoi inviare comandi alla CAM ──
    // esp_now_peer_info_t peerCAM = {};
    // memcpy(peerCAM.peer_addr, CAM_MAC, 6);
    // peerCAM.channel = 0;
    // peerCAM.encrypt = false;
    // esp_now_add_peer(&peerCAM);

    espNowReady = true;
}

// ──────────────────────────────────────────────────────────
//  LOOP
// ──────────────────────────────────────────────────────────
void loop() {
    if (!espNowReady) return;

    // 1) Se è disponibile un frame completo, mandalo al PC
    if (frame_pending) {
        sendFrameToPC();
    }

    // 2) Leggi comandi dal PC (ASCII: F B L R S) e forwardali al robot
    if (Serial.available() > 0) {
        char cmd = Serial.read();
        if (cmd == 'F' || cmd == 'B' || cmd == 'L' || cmd == 'R' || cmd == 'S') {
            esp_now_send(RECEIVER_MAC, (uint8_t*)&cmd, 1);
        }
        // Ignora silenziosamente qualsiasi altro byte (evita rumore accidentale)
    }
}
