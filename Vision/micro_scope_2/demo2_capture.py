import cv2
import os
from datetime import datetime

CAM_INDEX = 1
OUTPUT_DIR = os.path.join(os.path.dirname(__file__), "..", "vision_demo_test_res")
os.makedirs(OUTPUT_DIR, exist_ok=True)

cap = cv2.VideoCapture(CAM_INDEX, cv2.CAP_DSHOW)
if not cap.isOpened():
    print(f"Failed to open camera at index {CAM_INDEX}")
    exit(1)

cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
print(f"Camera opened: {w}x{h}")
print("Controls: [s] snapshot  [v] toggle video  [q] quit")

# Fit preview to screen (leave room for taskbar)
SCREEN_W, SCREEN_H = 1920, 1020
scale = min(SCREEN_W / w, SCREEN_H / h)
disp_w, disp_h = int(w * scale), int(h * scale)

cv2.namedWindow("Borescope Preview", cv2.WINDOW_NORMAL)
cv2.resizeWindow("Borescope Preview", disp_w, disp_h)

recording = False
writer = None
snapshot_count = 0

def ts():
    return datetime.now().strftime("%Y%m%d_%H%M%S")

while True:
    ret, frame = cap.read()
    if not ret:
        print("Frame read failed.")
        break

    if recording and writer:
        writer.write(frame)

    # OSD
    status = "REC" if recording else "LIVE"
    color = (0, 0, 220) if recording else (0, 200, 0)
    cv2.putText(frame, status, (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.4, color, 2)
    cv2.putText(frame, "[s]snapshot  [v]video  [q]quit",
                (20, h - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (200, 200, 200), 1)

    cv2.imshow("Borescope Preview", frame)
    key = cv2.waitKey(1) & 0xFF

    if key == ord('q'):
        break

    elif key == ord('s'):
        fname = os.path.join(OUTPUT_DIR, f"snap_{ts()}_{snapshot_count:03d}.png")
        cv2.imwrite(fname, frame)
        print(f"Snapshot saved: {fname}")
        snapshot_count += 1

    elif key == ord('v'):
        if not recording:
            vname = os.path.join(OUTPUT_DIR, f"video_{ts()}.mp4")
            fourcc = cv2.VideoWriter_fourcc(*"mp4v")
            fps = cap.get(cv2.CAP_PROP_FPS)
            fps = fps if fps > 0 else 30.0
            writer = cv2.VideoWriter(vname, fourcc, fps, (w, h))
            recording = True
            print(f"Recording started: {vname}")
        else:
            recording = False
            writer.release()
            writer = None
            print("Recording stopped.")

if recording and writer:
    writer.release()
cap.release()
cv2.destroyAllWindows()
