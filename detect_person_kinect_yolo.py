#!/usr/bin/env python3
import freenect
import cv2
import numpy as np
import os

# ──────── 1. LOAD YOLOv5 ONNX MODEL ────────

# Path to your ONNX file (adjust if you used a different name/location)
YOLO_ONNX = os.path.join(os.path.expanduser("~"), "semilla_cerebral", "yolov5", "yolov5nu.onnx")

# Load the network once using cv2.dnn
net = cv2.dnn.readNetFromONNX(YOLO_ONNX)
net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)  # or DNN_TARGET_CUDA if available

# These are YOLOv5 defaults for 640×640 inference
INPUT_WIDTH = 640
INPUT_HEIGHT = 640
CONFIDENCE_THRESHOLD = 0.30
NMS_THRESHOLD = 0.45

# COCO class 0 is “person”
CLASS_PERSON = 0

# ──────── 2. UTILITY: RUN YOLOv5 ON A FRAME ────────

def detect_persons(frame_bgr):
    """
    Corre YOLOvONNX sobre frame_bgr y devuelve una lista de (x, y, w, h, score)
    solo para la clase PERSON (índice 0 en COCO).
    """

    h0, w0 = frame_bgr.shape[:2]

    # 1) Letterbox a 640×640: redimensionar + pad de 114
    r = min(INPUT_WIDTH / w0, INPUT_HEIGHT / h0)
    new_unpad = (int(w0 * r), int(h0 * r))
    dw, dh = INPUT_WIDTH - new_unpad[0], INPUT_HEIGHT - new_unpad[1]
    dw //= 2; dh //= 2

    resized = cv2.resize(frame_bgr, new_unpad, interpolation=cv2.INTER_LINEAR)
    padded  = cv2.copyMakeBorder(
        resized, dh, dh, dw, dw,
        cv2.BORDER_CONSTANT, value=(114, 114, 114)
    )

    # 2) blobFromImage normaliza [0..255]→[0..1], BGR→RGB automáticamente
    blob = cv2.dnn.blobFromImage(
        padded, 1/255.0, (INPUT_WIDTH, INPUT_HEIGHT),
        swapRB=True, crop=False
    )
    net.setInput(blob)
    outs = net.forward()     # outs es una lista, normalmente [array(C, N)]
    preds = outs[0]          # shape = (C, N). En tu caso (84, 8400)

    # DEBUG: ver shapes
    # print("outs shapes:", [x.shape for x in outs])
    # print("antes de trasponer, preds.shape:", preds.shape)

    # 3) Tranponer a (N, C) para iterar detección por detección
    preds = preds.T         # ahora shape = (8400, 84)
    # print("después de trasponer, preds.shape:", preds.shape)

    boxes = []
    for row in preds:  # cada row es un array de 84 elementos
        # 3.1) Extraer coords + scores por clase
        cx, cy, bw, bh = float(row[0]), float(row[1]), float(row[2]), float(row[3])
        class_scores = row[4:]           # un vector length=80 (clases COCO)

        # 3.2) Escoger la clase con mayor probabilidad
        class_id = int(np.argmax(class_scores))
        score = float(class_scores[class_id])
        if class_id != CLASS_PERSON or score < CONFIDENCE_THRESHOLD:
            continue

        # 3.3) Convertir de centro‐w/h (en escala 640) a píxeles en frame original
        x = int((cx - bw/2) * w0 / INPUT_WIDTH)
        y = int((cy - bh/2) * h0 / INPUT_HEIGHT)
        w = int(bw * w0 / INPUT_WIDTH)
        h = int(bh * h0 / INPUT_HEIGHT)

        boxes.append([x, y, w, h, score])

    # 4) Non‐maximum suppression si es necesario
    if not boxes:
        return []

    bboxes = [b[:4] for b in boxes]       # [[x,y,w,h], …]
    scores = [b[4] for b in boxes]        # [score, score, …]
    idxs = cv2.dnn.NMSBoxes(bboxes, scores,
                            CONFIDENCE_THRESHOLD,
                            NMS_THRESHOLD)

    dets = []
    for i in idxs.flatten():
        x, y, w, h, sc = boxes[i]
        dets.append((int(x), int(y), int(w), int(h), sc))
    return dets


# ──────── 3. UTILITY: DISTANCE ESTIMATE ────────

def estimate_distance(depth_mm, bbox):
    """
    Returns the median mm‐distance inside a shrunk ROI.
    """
    x, y, w, h = bbox
    pad_w = int(0.1 * w)
    pad_h = int(0.1 * h)
    x1 = max(x + pad_w, 0)
    y1 = max(y + pad_h, 0)
    x2 = min(x + w - pad_w, depth_mm.shape[1])
    y2 = min(y + h - pad_h, depth_mm.shape[0])
    roi = depth_mm[y1:y2, x1:x2]
    valid = roi[roi > 0]
    return int(np.median(valid)) if valid.size else 0

# ──────── 3. UTILITY: SERIAL ────────
import serial
import time

def send_distance(distance, port="/dev/ttyACM0", baudrate=9600, timeout=1):
    """
    Sends the command "Z X" over a USB serial port, where X is the distance.

    Args:
        distance (int or float): The distance value to send.
        port (str): The serial port device (e.g. "/dev/ttyUSB0").
        baudrate (int): The baud rate for the serial connection.
        timeout (float): Read/write timeout in seconds.

    Raises:
        serial.SerialException: If the port cannot be opened.
    """
    # Format the command, ending with a newline
    cmd = f"Z {distance}\n"
    try:
        with serial.Serial(port, baudrate, timeout=timeout) as ser:
            ser.write(cmd.encode("utf-8"))
    except serial.SerialException as e:
        print(f"Error: could not open {port}: {e}")

def send_cmd(cmd, port="/dev/ttyACM0", baudrate=9600, timeout=1):
    """
    Sends the command "Z X" over a USB serial port, where X is the distance.

    Args:
        distance (int or float): The distance value to send.
        port (str): The serial port device (e.g. "/dev/ttyUSB0").
        baudrate (int): The baud rate for the serial connection.
        timeout (float): Read/write timeout in seconds.

    Raises:
        serial.SerialException: If the port cannot be opened.
    """
    # Format the command, ending with a newline
    try:
        with serial.Serial(port, baudrate, timeout=timeout) as ser:
            ser.write(cmd.encode("utf-8"))
    except serial.SerialException as e:
        print(f"Error: could not open {port}: {e}")

def escalar_2_0_5_a_0_700(x):
    """
    Escala x ∈ [2, 0.5] al rango [0, 700], de modo que:
      - x=2   → y=0
      - x=0.5 → y=700
    """
    return np.clip((x - 4.5) * (90 / (1 - 2)),0,90)

# ──────── 4. MAIN LOOP ────────

def main():
    cv2.namedWindow("RGB")
    cv2.namedWindow("Depth")
    print("Press ESC to exit.")

    # 1. Configura el nombre del puerto y la velocidad (baudrate)
    #    Cambia 'COM3' por el puerto que use tu Arduino (en Windows), o '/dev/ttyACM0' en Linux/macOS
    puerto = "/dev/ttyACM0"        # Ejemplo en Windows
    baudrate = 9600        # Debe coincidir con el Serial.begin(9600) en tu sketch de Arduino

    # 2. Abre el puerto serie
    ser = serial.Serial(puerto, baudrate, timeout=1)

    # 3. Espera un par de segundos para que el Arduino reinicie
    time.sleep(5)

    # 4. Envía un comando al Arduino (por ejemplo, la cadena 'LED_ON\n')
    #    Fíjate en el sufijo '\n' si tu Arduino espera salto de línea para parsear
    comando = 'R\n'
    ser.write(comando.encode('utf-8'))
    send_distance(0)


    while True:
        # 4.1 Grab depth in mm and RGB in BGR
        depth_mm, _ = freenect.sync_get_depth(format=freenect.DEPTH_MM)
        rgb_raw, _ = freenect.sync_get_video()
        rgb_bgr = cv2.cvtColor(rgb_raw, cv2.COLOR_RGB2BGR)

        # 4.2 Detect persons
        detections = detect_persons(rgb_bgr)

        max_dist_mm = -1
        # 4.3 For each person, find median distance from depth map
        for (x, y, w, h, score) in detections:
            dist_mm = estimate_distance(depth_mm, (x, y, w, h))

            if (dist_mm > max_dist_mm):
                max_dist_mm = dist_mm

            # Draw bounding box
            cv2.rectangle(rgb_bgr, (x, y), (x + w, y + h), (0,255,0), 2)
            # Draw distance label (in metres, one decimal)
            label = f"{dist_mm/1000:.1f} m"
            cv2.putText(rgb_bgr, label, (x, y - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)

        print(f"Max distance: \r\n", max_dist_mm)

        if max_dist_mm > 0:
            print(int(escalar_2_0_5_a_0_700(max_dist_mm/1000)))
            send_distance(int(escalar_2_0_5_a_0_700(max_dist_mm/1000)))
        else:
            comando = 'R\n'
            ser.write(comando.encode('utf-8'))
            send_distance(0)
        
        # 4.4 Visualize depth as 8-bit
        depth_vis = np.clip(depth_mm, 0, 4000).astype(np.float32)
        depth_vis = (depth_vis * (255.0 / 4000.0)).astype(np.uint8)

        # 4.5 Show frames
        cv2.imshow("RGB", rgb_bgr)
        cv2.imshow("Depth", depth_vis)

        if cv2.waitKey(1) == 27:  # ESC to quit
            break

    cv2.destroyAllWindows()
    freenect.sync_stop()

if __name__ == "__main__":
    main()
