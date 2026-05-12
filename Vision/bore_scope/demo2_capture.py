"""
TESLONG NTG100 Borescope — capture demo
Controls: [s] snapshot  [v] toggle video recording  [q] / ESC  quit
Best focus distance (no 45° mirror): ~8 mm
"""

import cv2
import os
from datetime import datetime

# ── config ─────────────────────────────────────────────────────────────────
CAM_INDEX  = 1          # NTG100 on this machine (0 = laptop webcam)
RES_W      = 1280
RES_H      = 720
TARGET_FPS = 30.0
OUTPUT_DIR = os.path.join(os.path.dirname(__file__), "..", "vision_demo_test_res")
# ───────────────────────────────────────────────────────────────────────────

os.makedirs(OUTPUT_DIR, exist_ok=True)

cap = cv2.VideoCapture(CAM_INDEX, cv2.CAP_DSHOW)
if not cap.isOpened():
    print(f"[borescope] ERROR: cannot open camera at index {CAM_INDEX}")
    raise SystemExit(1)

cap.set(cv2.CAP_PROP_FRAME_WIDTH,  RES_W)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, RES_H)
cap.set(cv2.CAP_PROP_FPS, TARGET_FPS)

w   = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
h   = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
fps = cap.get(cv2.CAP_PROP_FPS) or TARGET_FPS
print(f"[borescope] opened index {CAM_INDEX}: {w}x{h} @ {fps:.1f} fps")
print("  [s] snapshot  [v] toggle video  [q]/ESC quit")

# Scale preview to fit screen
SCREEN_W, SCREEN_H = 1920, 1020
scale = min(SCREEN_W / w, SCREEN_H / h)
disp_w, disp_h = int(w * scale), int(h * scale)

cv2.namedWindow("NTG100 Borescope", cv2.WINDOW_NORMAL)
cv2.resizeWindow("NTG100 Borescope", disp_w, disp_h)

recording      = False
writer         = None
snapshot_count = 0


def ts() -> str:
    return datetime.now().strftime("%Y%m%d_%H%M%S")


while True:
    ret, frame = cap.read()
    if not ret:
        print("[borescope] frame read failed — camera disconnected?")
        break

    if recording and writer:
        writer.write(frame)

    # OSD overlay
    status = "REC" if recording else "LIVE"
    color  = (0, 0, 220) if recording else (0, 200, 0)
    cv2.putText(frame, status, (20, 50),
                cv2.FONT_HERSHEY_SIMPLEX, 1.4, color, 2)
    cv2.putText(frame, "[s]snapshot  [v]video  [q]quit",
                (20, h - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (200, 200, 200), 1)

    cv2.imshow("NTG100 Borescope", frame)
    key = cv2.waitKey(1) & 0xFF

    if key in (ord('q'), 27):
        break

    elif key == ord('s'):
        fname = os.path.join(OUTPUT_DIR, f"bore_snap_{ts()}_{snapshot_count:03d}.png")
        cv2.imwrite(fname, frame)
        print(f"[borescope] snapshot saved: {fname}")
        snapshot_count += 1

    elif key == ord('v'):
        if not recording:
            vname  = os.path.join(OUTPUT_DIR, f"bore_video_{ts()}.mp4")
            fourcc = cv2.VideoWriter_fourcc(*"mp4v")
            writer = cv2.VideoWriter(vname, fourcc, fps, (w, h))
            recording = True
            print(f"[borescope] recording started: {vname}")
        else:
            recording = False
            writer.release()
            writer = None
            print("[borescope] recording stopped.")

if recording and writer:
    writer.release()
cap.release()
cv2.destroyAllWindows()
