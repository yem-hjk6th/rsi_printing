import cv2
import os
import time
import numpy as np
from datetime import datetime

CAM_INDEX = 1
BASE_OUTPUT = os.path.join(os.path.dirname(__file__), "..", "vision_demo_test_res")

# Session subfolder created once at startup
session_ts = datetime.now().strftime("%Y%m%d_%H%M%S")
OUTPUT_DIR = os.path.join(BASE_OUTPUT, session_ts)
os.makedirs(OUTPUT_DIR, exist_ok=True)
print(f"Output dir: {OUTPUT_DIR}")

cap = cv2.VideoCapture(CAM_INDEX, cv2.CAP_DSHOW)
if not cap.isOpened():
    print(f"Failed to open camera at index {CAM_INDEX}")
    exit(1)

cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
print(f"Camera: {w}x{h}  |  Controls: [s] snapshot  [v] toggle record  [q] quit")

SCREEN_W, SCREEN_H = 1920, 1020
scale = min(SCREEN_W / w, SCREEN_H / h)
disp_w, disp_h = int(w * scale), int(h * scale)

cv2.namedWindow("Borescope Stream Info", cv2.WINDOW_NORMAL)
cv2.resizeWindow("Borescope Stream Info", disp_w, disp_h)

recording = False
writer = None
snapshot_count = 0

# FPS tracking
fps_buf = []
t_prev = time.perf_counter()

# ── Color thresholds ──────────────────────────────────────────
#  (green=good, yellow=marginal, red=bad)
def metric_color(val, green, yellow):
    """green_range=(lo,hi), yellow_range=(lo,hi); outside = red"""
    if green[0] <= val <= green[1]:
        return (0, 210, 0)
    elif yellow[0] <= val <= yellow[1]:
        return (0, 200, 220)
    else:
        return (0, 60, 220)

def compute_metrics(frame):
    gray_u8 = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    gray = gray_u8.astype(np.float32)
    sharpness  = float(cv2.Laplacian(gray_u8, cv2.CV_32F).var())
    brightness = float(gray.mean())
    contrast   = float(gray.std())
    overexpose = float((gray > 245).mean() * 100)
    return sharpness, brightness, contrast, overexpose

def draw_hud(frame, fps, sharpness, brightness, contrast, overexpose, recording):
    overlay = frame.copy()
    # Semi-transparent HUD background
    cv2.rectangle(overlay, (10, 10), (480, 185), (20, 20, 20), -1)
    cv2.addWeighted(overlay, 0.55, frame, 0.45, 0, frame)

    def put(text, y, color):
        cv2.putText(frame, text, (20, y), cv2.FONT_HERSHEY_SIMPLEX, 0.68, (0,0,0), 3)
        cv2.putText(frame, text, (20, y), cv2.FONT_HERSHEY_SIMPLEX, 0.68, color, 1)

    # FPS: green>=25, yellow>=15
    fps_c = metric_color(fps, (25, 999), (15, 25))
    put(f"FPS        {fps:5.1f}", 40, fps_c)

    # Sharpness: green>=150, yellow>=60
    sh_c = metric_color(sharpness, (150, 99999), (60, 150))
    put(f"Sharpness  {sharpness:7.1f}  (Lap.Var)", 70, sh_c)

    # Brightness: green=80-180, yellow=50-220
    br_c = metric_color(brightness, (80, 180), (50, 220))
    put(f"Brightness {brightness:5.1f} /255", 100, br_c)

    # Contrast: green>=40, yellow>=20
    co_c = metric_color(contrast, (40, 999), (20, 40))
    put(f"Contrast   {contrast:5.1f}  (StdDev)", 130, co_c)

    # Overexpose: green<5, yellow<15
    ov_c = metric_color(overexpose, (0, 5), (5, 15))
    put(f"Overexpose {overexpose:5.1f} %", 160, ov_c)

    # REC indicator
    if recording:
        cv2.circle(frame, (h - 30, 30), 12, (0, 0, 220), -1)
        cv2.putText(frame, "REC", (h - 10, 38),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 220), 2)

    # Bottom hint
    cv2.putText(frame, "[s] snapshot   [v] record   [q] quit",
                (20, h - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (160, 160, 160), 1)

    return frame


while True:
    ret, frame = cap.read()
    if not ret:
        print("Frame read failed.")
        break

    # FPS
    t_now = time.perf_counter()
    fps_buf.append(1.0 / max(t_now - t_prev, 1e-6))
    t_prev = t_now
    if len(fps_buf) > 30:
        fps_buf.pop(0)
    fps = float(np.mean(fps_buf))

    sharpness, brightness, contrast, overexpose = compute_metrics(frame)

    if recording and writer:
        writer.write(frame)

    draw_hud(frame, fps, sharpness, brightness, contrast, overexpose, recording)
    cv2.imshow("Borescope Stream Info", frame)

    key = cv2.waitKey(1) & 0xFF

    if key == ord('q'):
        break

    elif key == ord('s'):
        fname = os.path.join(OUTPUT_DIR,
                             f"snap_{datetime.now().strftime('%H%M%S')}_{snapshot_count:03d}.png")
        cv2.imwrite(fname, frame)
        print(f"Snapshot: {fname}")
        snapshot_count += 1

    elif key == ord('v'):
        if not recording:
            vname = os.path.join(OUTPUT_DIR,
                                 f"video_{datetime.now().strftime('%H%M%S')}.mp4")
            fourcc = cv2.VideoWriter_fourcc(*"mp4v")
            fps_write = cap.get(cv2.CAP_PROP_FPS)
            fps_write = fps_write if fps_write > 0 else 30.0
            writer = cv2.VideoWriter(vname, fourcc, fps_write, (w, h))
            recording = True
            print(f"Recording: {vname}")
        else:
            recording = False
            writer.release()
            writer = None
            print("Recording stopped.")

if recording and writer:
    writer.release()
cap.release()
cv2.destroyAllWindows()
