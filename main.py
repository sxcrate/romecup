import cv2
import numpy as np
import serial
import sys
import time
import subprocess
import threading
import queue
from collections import deque
import os

os.environ["ULTRALYTICS_AUTO_UPDATE"] = "0"
try:
    from ultralytics.utils import SETTINGS as _UL_SETTINGS
    _UL_SETTINGS["sync"] = False
except Exception:
    pass
try:
    from ultralytics import YOLO as _YOLO
    _ULTRALYTICS_OK = True
except ImportError:
    _ULTRALYTICS_OK = False

PORT = "/dev/cu.usbserial-120"
BAUD = 115200
ARUCO_DICT = cv2.aruco.DICT_6X6_250
ANGLE_THRESHOLD = 20
Y_THRESHOLD = 160

DURATA_COMANDO = 0.5
DURATA_COMANDO_STERZO = 0.2
DELAY_COMANDO = 0.5

LOWER_RED1 = np.array([0, 120, 70])
UPPER_RED1 = np.array([10, 255, 255])
LOWER_RED2 = np.array([170, 120, 70])
UPPER_RED2 = np.array([180, 255, 255])
LOWER_ORANGE_YEL = np.array([10, 120, 70])
UPPER_ORANGE_YEL = np.array([45, 255, 255])
FIRE_KERNEL = np.ones((3, 3), np.uint8)
FIRE_HISTORY_LEN = 5
FIRE_MOV_THRESH = 3000
FIRE_PERSIST_MIN = 3

DIRECTION_TO_CMD = {
    "AVANTI": b"F",
    "INDIETRO": b"B",
    "SINISTRA": b"L",
    "DESTRA": b"R",
    "FERMO": b"S",
}
CMD_LABELS = {b"F": "AVANTI", b"B": "INDIETRO", b"L": "SINISTRA", b"R": "DESTRA", b"S": "STOP"}

try:
    ser = serial.Serial(PORT, BAUD, timeout=0.1)
    print(f"[OK] Seriale aperta su {PORT}")
except Exception as e:
    print(f"[ERRORE] Impossibile aprire {PORT}: {e}")
    sys.exit(1)

aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT)
aruco_params = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)

W, H = 960, 540
FRAME_SIZE = W * H * 3
RTMP_URL = "rtmp://172.20.10.2/live/drone"

def _start_ffmpeg():
    return subprocess.Popen(
        [
            "ffmpeg",
            "-fflags",
            "nobuffer",
            "-flags",
            "low_delay",
            "-listen",
            "1",
            "-i",
            RTMP_URL,
            "-an",
            "-f",
            "rawvideo",
            "-pix_fmt",
            "bgr24",
            "-vf",
            f"scale={W}:{H}",
            "-",
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.DEVNULL,
    )

ffmpeg_proc = _start_ffmpeg()
print(f"[OK] In ascolto su {RTMP_URL} ...")

_MODEL_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "fire_model.pt")
yolo_model = None
YOLO_AVAILABLE = False
if _ULTRALYTICS_OK:
    if os.path.exists(_MODEL_PATH):
        try:
            yolo_model = _YOLO(_MODEL_PATH)
            YOLO_AVAILABLE = True
            print(f"[OK] Modello YOLO caricato da {_MODEL_PATH}")
        except Exception as _e:
            print(f"[WARN] Errore caricamento YOLO: {_e}")
    else:
        print("[INFO] fire_model.pt non trovato — esegui scarica_modello.py con connessione internet")
        print(f"       Percorso atteso: {_MODEL_PATH}")

frame_queue = queue.Queue(maxsize=2)
STALL_TIMEOUT = 6.0

def _frame_reader():
    global ffmpeg_proc
    last_frame_t = time.time()
    while True:
        proc = ffmpeg_proc
        try:
            raw = proc.stdout.read(FRAME_SIZE)
        except Exception:
            raw = b""

        if len(raw) == FRAME_SIZE:
            last_frame_t = time.time()
            f = np.frombuffer(raw, dtype=np.uint8).reshape((H, W, 3)).copy()
            if frame_queue.full():
                try:
                    frame_queue.get_nowait()
                except queue.Empty:
                    pass
            frame_queue.put(f)
        else:
            if time.time() - last_frame_t > STALL_TIMEOUT:
                print("[WARN] Stream bloccato, riavvio ffmpeg...")
                try:
                    proc.terminate()
                    proc.wait(timeout=2)
                except Exception:
                    pass
                ffmpeg_proc = _start_ffmpeg()
                last_frame_t = time.time()
                print("[OK] ffmpeg riavviato, in attesa stream...")
            else:
                time.sleep(0.05)

threading.Thread(target=_frame_reader, daemon=True).start()

reference_angle = None
calibration_mode = False

phase = "IDLE"
phase_start = 0.0
active_cmd = b"S"
active_dir = "FERMO"
active_duration = DURATA_COMANDO

pausa_aruco = False

mode = "COMANDO"
fire_engine = "YOLO" if YOLO_AVAILABLE else "HSV"
fire_mask_history = deque(maxlen=FIRE_HISTORY_LEN)
fire_persist_count = 0

print("\nIIS ENZO FERRARI - ROMA: CONTROLLO ROVER CON ARUCO MARKER")
print("  Marker in ALTO    -> Avanti")
print("  Marker in BASSO   -> Indietro")
print("  Zona centrale     -> Fermo / Sterzo (inclina il marker)")
print(f"  Durata standard: {DURATA_COMANDO}s | Durata sterzo: {DURATA_COMANDO_STERZO}s")
print(f"  Delay tra comandi: {DELAY_COMANDO}s")
print("  [p]=switch modalità (COMANDO/FUOCO) | [k]=pausa ArUco | [c]=calibra | [r]=reset | [ESC]=esci\n")


def calculate_angle(corners):
    top_left = corners[0][0]
    top_right = corners[0][1]
    dx = top_right[0] - top_left[0]
    dy = top_right[1] - top_left[1]
    return np.degrees(np.arctan2(dy, dx))


def get_direction(cx, cy, frame_center, current_angle, ref_angle):
    offset_y = cy - frame_center[1]

    if offset_y < -Y_THRESHOLD:
        return "INDIETRO"

    if offset_y > Y_THRESHOLD:
        return "AVANTI"

    if ref_angle is None:
        return "FERMO"

    diff = current_angle - ref_angle
    if diff > 180:
        diff -= 360
    if diff < -180:
        diff += 360

    if diff > ANGLE_THRESHOLD:
        return "SINISTRA"
    elif diff < -ANGLE_THRESHOLD:
        return "DESTRA"
    else:
        return "FERMO"


try:
    last_frame = None
    while True:
        try:
            frame = frame_queue.get(timeout=0.1)
            last_frame = frame
        except queue.Empty:
            if last_frame is not None:
                frame = last_frame.copy()
            else:
                waiting = np.zeros((H, W, 3), dtype=np.uint8)
                cv2.putText(
                    waiting,
                    "In attesa dello stream RTMP...",
                    (W // 2 - 220, H // 2),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.9,
                    (0, 200, 255),
                    2,
                )
                cv2.putText(
                    waiting,
                    "rtmp://172.20.10.2/live/drone",
                    (W // 2 - 190, H // 2 + 40),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.65,
                    (180, 180, 180),
                    1,
                )
                cv2.imshow("Controllo Rover ArUco - IIS Enzo Ferrari ROMA", waiting)
                if (cv2.waitKey(1) & 0xFF) == 27:
                    break
                continue
        now = time.time()
        h, w = frame.shape[:2]
        frame_center = (w // 2, h // 2)

        if mode == "FUOCO":
            fire_found = False

            if fire_engine == "YOLO" and YOLO_AVAILABLE:
                results = yolo_model(frame, conf=0.50, verbose=False)[0]
                for box in results.boxes:
                    cls_id = int(box.cls)
                    conf = float(box.conf)
                    label = yolo_model.names[cls_id]
                    x1, y1, x2, y2 = map(int, box.xyxy[0])

                    if label == "fire":
                        colore_box = (0, 0, 255)
                        fire_found = True
                    else:
                        colore_box = (80, 80, 80)

                    cv2.rectangle(frame, (x1, y1), (x2, y2), colore_box, 2)
                    cv2.putText(
                        frame,
                        f"{label} {conf:.0%}",
                        (x1, max(y1 - 10, 10)),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.8,
                        colore_box,
                        2,
                    )

                blur = cv2.GaussianBlur(frame, (9, 9), 0)
                hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
                mask = cv2.bitwise_or(
                    cv2.inRange(hsv, LOWER_RED1, UPPER_RED1),
                    cv2.inRange(hsv, LOWER_RED2, UPPER_RED2),
                )
                mask = cv2.bitwise_or(mask, cv2.inRange(hsv, LOWER_ORANGE_YEL, UPPER_ORANGE_YEL))
                mask = cv2.dilate(mask, FIRE_KERNEL, iterations=2)
                heat = np.zeros_like(frame)
                heat[mask > 0] = (0, 60, 180)
                cv2.addWeighted(heat, 0.25, frame, 0.75, 0, frame)

            else:
                blur = cv2.GaussianBlur(frame, (9, 9), 0)
                hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
                mask = cv2.bitwise_or(
                    cv2.inRange(hsv, LOWER_RED1, UPPER_RED1),
                    cv2.inRange(hsv, LOWER_RED2, UPPER_RED2),
                )
                mask = cv2.bitwise_or(mask, cv2.inRange(hsv, LOWER_ORANGE_YEL, UPPER_ORANGE_YEL))
                mask = cv2.erode(mask, FIRE_KERNEL, iterations=1)
                mask = cv2.dilate(mask, FIRE_KERNEL, iterations=2)

                movement = False
                if len(fire_mask_history) >= FIRE_HISTORY_LEN:
                    diff = cv2.absdiff(mask, fire_mask_history[0])
                    if np.sum(diff) / 255 > FIRE_MOV_THRESH:
                        movement = True
                fire_mask_history.append(mask.copy())

                contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                for contour in contours:
                    area = cv2.contourArea(contour)
                    if area < 500:
                        continue
                    x, y, bw, bh = cv2.boundingRect(contour)
                    perimeter = cv2.arcLength(contour, True)
                    circularity = (4 * np.pi * area / (perimeter**2)) if perimeter > 0 else 0
                    hull_area = cv2.contourArea(cv2.convexHull(contour))
                    solidity = area / hull_area if hull_area > 0 else 0
                    if circularity < 0.7 and solidity < 0.90 and movement:
                        colore_box = (0, 100, 255) if fire_persist_count < FIRE_PERSIST_MIN else (0, 0, 255)
                        cv2.rectangle(frame, (x, y), (x + bw, y + bh), colore_box, 2)
                        cv2.putText(
                            frame,
                            "Possibile fuoco" if fire_persist_count < FIRE_PERSIST_MIN else "FUOCO",
                            (x, max(y - 10, 10)),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.85,
                            colore_box,
                            2,
                        )
                        fire_found = True

            if fire_found:
                fire_persist_count = min(fire_persist_count + 1, FIRE_PERSIST_MIN + 5)
            else:
                fire_persist_count = max(0, fire_persist_count - 1)

            fire_confirmed = fire_persist_count >= FIRE_PERSIST_MIN

            if fire_confirmed:
                overlay = frame.copy()
                cv2.rectangle(overlay, (0, 0), (w - 1, h - 1), (0, 0, 255), 18)
                cv2.addWeighted(overlay, 0.5, frame, 0.5, 0, frame)

            overlay = frame.copy()
            cv2.rectangle(overlay, (0, 0), (w, 45), (20, 20, 20), -1)
            cv2.addWeighted(overlay, 0.5, frame, 0.5, 0, frame)
            motore = "YOLO" if (fire_engine == "YOLO" and YOLO_AVAILABLE) else "HSV"
            cv2.putText(
                frame,
                f"[ RILEVAMENTO FUOCO: {motore} ]  [F]=switch  [P]=Comando",
                (15, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 140, 255),
                2,
            )
            cv2.putText(frame, "IIS Enzo Ferrari - ROMA", (15, 75), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)

            if fire_confirmed:
                stato_fuoco, colore_stato = "FUOCO CONFERMATO!", (0, 0, 255)
            elif fire_found:
                stato_fuoco, colore_stato = "Possibile fuoco...", (0, 140, 255)
            else:
                stato_fuoco, colore_stato = "Nessun fuoco rilevato", (0, 220, 0)

            cv2.putText(frame, stato_fuoco, (15, 115), cv2.FONT_HERSHEY_SIMPLEX, 0.9, colore_stato, 2)
            cv2.putText(
                frame,
                f"Confidenza: {min(fire_persist_count, FIRE_PERSIST_MIN)}/{FIRE_PERSIST_MIN}",
                (15, 145),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (200, 200, 200),
                1,
            )
            cv2.putText(
                frame,
                "[ESC]=esci  [p]=switch modalita'  [f]=YOLO/HSV",
                (15, h - 15),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (255, 0, 0),
                1,
            )

        else:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = detector.detectMarkers(gray)

            detected_dir = "FERMO"
            cx, cy = frame_center[0], frame_center[1]
            current_angle = 0.0

            if ids is not None and len(ids) > 0:
                cv2.aruco.drawDetectedMarkers(frame, corners, ids)
                marker_corners = corners[0][0]
                cx = int(np.mean(marker_corners[:, 0]))
                cy = int(np.mean(marker_corners[:, 1]))
                current_angle = calculate_angle([marker_corners])

                if calibration_mode:
                    reference_angle = current_angle
                    calibration_mode = False
                    print(f"[OK] Calibrato: angolo={{reference_angle:.1f}}")

                detected_dir = get_direction(cx, cy, frame_center, current_angle, reference_angle)

                cv2.circle(frame, (cx, cy), 6, (0, 255, 0), -1)
                cv2.line(frame, (cx, cy), frame_center, (255, 255, 0), 2)
                cv2.putText(
                    frame,
                    f"A:{{current_angle:.1f}}",
                    (cx - 40, cy - 35),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 255, 255),
                    1,
                )
                cv2.putText(
                    frame,
                    f"Y offset: {{cy - frame_center[1]}}px",
                    (cx - 40, cy - 55),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 255, 255),
                    1,
                )

            control_dir = "FERMO" if pausa_aruco else detected_dir

            if phase == "IDLE":
                new_cmd = DIRECTION_TO_CMD[control_dir]
                if new_cmd != b"S":
                    if control_dir in ["SINISTRA", "DESTRA"]:
                        active_duration = DURATA_COMANDO_STERZO
                    else:
                        active_duration = DURATA_COMANDO
                    ser.write(new_cmd)
                    print(f"  [AR] -> {{CMD_LABELS[new_cmd]}} (durata {{active_duration}}s)")
                    active_cmd = new_cmd
                    active_dir = control_dir
                    phase = "RUNNING"
                    phase_start = now
                else:
                    ser.write(b"S")
                    active_cmd = b"S"
                    active_dir = "FERMO"

            elif phase == "RUNNING":
                elapsed = now - phase_start
                remaining = active_duration - elapsed
                if elapsed >= active_duration:
                    ser.write(b"S")
                    print(f"  -> STOP (pausa {{DELAY_COMANDO}}s)")
                    active_cmd = b"S"
                    active_dir = "FERMO"
                    phase = "PAUSA"
                    phase_start = now
                cv2.putText(frame, f"CMD: {{remaining:.1f}}s", (w - 160, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

            elif phase == "PAUSA":
                elapsed = now - phase_start
                remaining = DELAY_COMANDO - elapsed
                if elapsed >= DELAY_COMANDO:
                    phase = "IDLE"
                cv2.putText(frame, f"PAUSA: {{remaining:.1f}}s", (w - 170, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 100, 255), 2)

            zone_top = frame_center[1] - Y_THRESHOLD
            zone_bottom = frame_center[1] + Y_THRESHOLD
            cv2.rectangle(frame, (0, zone_top), (w, zone_bottom), (0, 80, 0), 1)
            cv2.putText(frame, "ZONA NEUTRA (sterzo/fermo)", (10, zone_top - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 150, 0), 1)
            cv2.putText(frame, "v INDIETRO", (w // 2 - 50, zone_top - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cv2.putText(frame, "^ AVANTI", (w // 2 - 60, zone_bottom + 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 100, 255), 2)
            cv2.line(frame, (frame_center[0], 0), (frame_center[0], h), (255, 255, 255), 1)
            cv2.line(frame, (0, frame_center[1]), (w, frame_center[1]), (255, 255, 255), 1)

            if pausa_aruco:
                overlay = frame.copy()
                cv2.rectangle(overlay, (0, 0), (w, 45), (0, 0, 160), -1)
                cv2.addWeighted(overlay, 0.45, frame, 0.55, 0, frame)
                cv2.putText(
                    frame,
                    "[ PAUSA - ArUco disabilitato ]  [K]=riprendi",
                    (15, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.65,
                    (255, 255, 0),
                    2,
                )

            hud_y = 65 if pausa_aruco else 30
            cv2.putText(frame, "IIS Enzo Ferrari - ROMA", (15, hud_y), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
            cv2.putText(frame, f"Rilevato: {{detected_dir}}", (15, hud_y + 35), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
            cv2.putText(frame, f"Cmd ESP32: {{CMD_LABELS[active_cmd]}}", (15, hud_y + 70), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 200, 255), 2)
            cv2.putText(frame, f"Fase: {{phase}}", (15, hud_y + 100), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 200, 0), 1)
            if reference_angle is not None:
                cv2.putText(frame, f"Ref angolo: {{reference_angle:.1f}}", (15, hud_y + 125), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 255), 1)
            cv2.putText(
                frame,
                "[ESC]=esci  [p]=Rilev.Fuoco  [c]=calibra  [r]=reset  [k]=pausa",
                (15, h - 15),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (255, 0, 0),
                1,
            )

        cv2.imshow("Controllo Rover ArUco - IIS Enzo Ferrari ROMA", frame)

        key = cv2.waitKey(1) & 0xFF

        if key == 27:
            break
        elif key == ord("p"):
            if mode == "COMANDO":
                mode = "FUOCO"
                ser.write(b"S")
                phase = "IDLE"
                fire_mask_history.clear()
                fire_persist_count = 0
                print("[INFO] Modalità: RILEVAMENTO FUOCO")
            else:
                mode = "COMANDO"
                print("[INFO] Modalità: STAZIONE DI COMANDO")
        elif mode == "FUOCO" and key == ord("f"):
            if fire_engine == "YOLO" and YOLO_AVAILABLE:
                fire_engine = "HSV"
            elif YOLO_AVAILABLE:
                fire_engine = "YOLO"
            else:
                print("[INFO] YOLO non disponibile, impossibile switchare")
            fire_mask_history.clear()
            fire_persist_count = 0
            print(f"[INFO] Motore fuoco: {fire_engine}")
        elif mode == "COMANDO":
            if key == ord("k"):
                pausa_aruco = not pausa_aruco
                phase = "IDLE"
                ser.write(b"S")
                print(f"[INFO] ArUco {'IN PAUSA' if pausa_aruco else 'ATTIVO'}")
            elif key == ord("c"):
                calibration_mode = True
                print("[INFO] Calibrazione: tieni il marker dritto nella zona neutra...")
            elif key == ord("r"):
                reference_angle = None
                print("[INFO] Calibrazione resettata")

finally:
    ser.write(b"S")
    ser.close()
    try:
        ffmpeg_proc.terminate()
        ffmpeg_proc.wait(timeout=3)
    except Exception:
        pass
    cv2.destroyAllWindows()
    print("Uscita.")
