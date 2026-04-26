# Controllo Rover ArUco - IIS Enzo Ferrari ROMA

Sistema di controllo per rover mobile sviluppato dall'**IIS Enzo Ferrari di Roma** per la **Rome Cup**.

## Architettura del sistema

```
ESP32-CAM
   │  (stream RTMP)
   ▼
PC  ←→  ffmpeg  →  main.py (OpenCV + ArUco)
   │  Serial 115200 baud — comandi ASCII (F B L R S)
   ▼
ESP32 Trasmettitore  (sketch_mar17a.ino)
   │  (ESP-NOW, 1 byte)
   ▼
ESP32 Robot  (sketch_mar17b.ino)
   │
   ▼
Driver DRV8833  →  Motori
```

Il PC esegue `main.py`, che riceve il video via RTMP tramite `ffmpeg`, analizza i frame in tempo reale con OpenCV e invia comandi di movimento all'ESP32 trasmettitore tramite porta seriale (115200 baud).

## Descrizione

Il progetto consente di pilotare un rover tramite una telecamera e marker ArUco, con un modulo aggiuntivo di rilevamento incendi. Il flusso video è ricevuto in streaming RTMP tramite `ffmpeg` e analizzato in tempo reale con OpenCV.

## Funzionalità

- **Modalità COMANDO**: il rover viene guidato rilevando un marker ArUco nel campo visivo della telecamera.
  - Marker in **alto** → movimento in avanti
  - Marker in **basso** → movimento in indietro
  - Zona centrale + inclinazione marker → sterzo destra/sinistra
- **Modalità RILEVAMENTO FUOCO**: il sistema analizza il frame alla ricerca di fiamme usando due motori:
  - **YOLO** (se disponibile il modello `fire_model.pt`) — rilevamento basato su rete neurale
  - **HSV** — rilevamento basato su analisi del colore e analisi del movimento

## File del progetto

| File | Descrizione |
|---|---|
| `main.py` | Script principale Python (PC) |
| `sketch_mar17a.ino` | Firmware ESP32 trasmettitore (riceve comandi dal PC via seriale, li forward al robot via ESP-NOW) |
| `sketch_mar17b.ino` | Firmware ESP32 robot (driver DRV8833, riceve comandi via ESP-NOW) |

## Requisiti

### PC

- Python 3.8+
- OpenCV (`opencv-python` con supporto ArUco)
- NumPy
- PySerial
- ffmpeg (installato nel sistema)
- Ultralytics YOLO *(opzionale, per il motore YOLO)*

```bash
pip install opencv-python numpy pyserial ultralytics
```

### Hardware

- ESP32 (trasmettitore) con connessione USB al PC
- ESP32-CAM (sorgente video)
- ESP32 (robot) collegato al driver DRV8833
- Driver motori DRV8833

## Configurazione

### sketch_mar17a.ino — ESP32 Trasmettitore

Prima di caricare il firmware, aggiornare il MAC del ricevitore (robot):

```cpp
// MAC del ricevitore (robot) — leggerlo dal monitor seriale di sketch_mar17b.ino
uint8_t RECEIVER_MAC[] = {0xF0, 0x24, 0xF9, 0x43, 0x5E, 0x8C};
```

Il baud rate seriale è **115200** baud, uguale a quello usato da `main.py`.

### sketch_mar17b.ino — ESP32 Robot

Configurare i pin del driver DRV8833 e i parametri di velocità:

| Parametro | Descrizione | Valore predefinito |
|---|---|---|
| `SLEEP` | Pin SLEEP del DRV8833 | `32` |
| `AIN1` / `AIN2` | Pin motore A (ruota sinistra) | `14` / `27` |
| `BIN1` / `BIN2` | Pin motore B (ruota destra) | `26` / `25` |
| `VELOCITA` | Velocità avanti/indietro (0–255) | `255` |
| `VELOCITA_ROTAZIONE` | Velocità rotazione (0–255) | `255` |
| `TRIM_SINISTRA` / `TRIM_DESTRA` | Fattore di correzione per bilanciare i motori | `1.00` |

Al primo avvio, il monitor seriale stampa il MAC address del robot: copiarlo nel campo `RECEIVER_MAC` di `sketch_mar17a.ino`.

### main.py — Script Python

| Parametro | Descrizione | Valore predefinito |
|---|---|---|
| `PORT` | Porta seriale dell'ESP32 trasmettitore | `/dev/cu.usbserial-120` |
| `BAUD` | Baud rate seriale | `115200` |
| `RTMP_URL` | URL dello stream RTMP dalla ESP32-CAM | `rtmp://172.20.10.2/live/drone` |
| `ANGLE_THRESHOLD` | Soglia angolo per lo sterzo (gradi) | `20` |
| `Y_THRESHOLD` | Soglia verticale per avanti/indietro (px) | `160` |
| `DURATA_COMANDO` | Durata comando standard (s) | `0.5` |
| `DURATA_COMANDO_STERZO` | Durata comando di sterzo (s) | `0.2` |
| `DELAY_COMANDO` | Pausa tra un comando e il successivo (s) | `0.5` |

## Avvio

1. Caricare `sketch_mar17b.ino` sull'ESP32 del robot e copiare il MAC address stampato sul monitor seriale.
2. Aggiornare `RECEIVER_MAC` in `sketch_mar17a.ino` e caricarlo sull'ESP32 trasmettitore.
3. Collegare l'ESP32 trasmettitore al PC tramite USB.
4. Avviare lo stream RTMP dalla ESP32-CAM.
5. Avviare lo script Python:

```bash
python main.py
```

Lo script si connette alla porta seriale e attende lo stream RTMP. Una volta ricevuto il primo frame, il sistema di controllo è operativo.

## Comandi da tastiera

| Tasto | Funzione |
|---|---|
| `p` | Cambia modalità (COMANDO / RILEVAMENTO FUOCO) |
| `k` | Metti in pausa / riprendi il controllo ArUco |
| `c` | Calibra l'angolo di riferimento del marker |
| `r` | Resetta la calibrazione |
| `f` | Cambia motore fuoco (YOLO / HSV) *(solo modalità FUOCO)* |
| `ESC` | Esci dal programma |

## Protocollo di comunicazione

### PC → ESP32 Trasmettitore → Robot (comandi movimento)

Comandi ASCII a singolo byte:

| Byte | Comando |
|---|---|
| `F` | Avanti |
| `B` | Indietro |
| `L` | Sinistra |
| `R` | Destra |
| `S` | Stop |

### ESP32-CAM → PC (flusso video)

La ESP32-CAM trasmette il video via **RTMP** direttamente al PC. `main.py` avvia `ffmpeg` in ascolto sull'URL configurato (`RTMP_URL`) e riceve i frame come rawvideo BGR24.
