# Controllo Rover ArUco - IIS Enzo Ferrari ROMA

Sistema di controllo per rover mobile sviluppato dall'**IIS Enzo Ferrari di Roma** per la **Rome Cup**.

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

## Requisiti

- Python 3.8+
- OpenCV (`opencv-python` con supporto ArUco)
- NumPy
- PySerial
- ffmpeg (installato nel sistema)
- Ultralytics YOLO *(opzionale, per il motore YOLO)*

Installazione dipendenze Python:

```bash
pip install opencv-python numpy pyserial ultralytics
```

## Configurazione

Nel file `main.py` è possibile modificare i seguenti parametri:

| Parametro | Descrizione | Valore predefinito |
|---|---|---|
| `PORT` | Porta seriale dell'ESP32 | `/dev/cu.usbserial-120` |
| `BAUD` | Baud rate seriale | `115200` |
| `RTMP_URL` | URL dello stream RTMP | `rtmp://172.20.10.2/live/drone` |
| `ANGLE_THRESHOLD` | Soglia angolo per lo sterzo (gradi) | `20` |
| `Y_THRESHOLD` | Soglia verticale per avanti/indietro (px) | `160` |
| `DURATA_COMANDO` | Durata comando standard (s) | `0.5` |
| `DURATA_COMANDO_STERZO` | Durata comando di sterzo (s) | `0.2` |
| `DELAY_COMANDO` | Pausa tra un comando e il successivo (s) | `0.5` |

## Avvio

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

## Comunicazione con ESP32

Il rover riceve comandi seriali a singolo byte:

| Byte | Comando |
|---|---|
| `F` | Avanti |
| `B` | Indietro |
| `L` | Sinistra |
| `R` | Destra |
| `S` | Stop |
