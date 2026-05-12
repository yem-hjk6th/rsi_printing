"""
demo4_edge.py  —  Live edge detection with real-time parameter tuning
Builds on demo3 metrics HUD.

Layout: LEFT = original + HUD  |  RIGHT = edge result
Trackbar window: method & all params

Keys:
  [s]  snapshot (saves original + edge pair)
  [v]  toggle video recording
  [d]  cycle display mode: side-by-side | overlay | edge-only
  [q]  quit

Edge methods (trackbar "Method"):
  0 = Canny     params: Blur, T1, T2
  1 = Sobel     params: Blur, KSize(idx), Scale
  2 = Laplacian params: Blur, (no extra params)
"""

import cv2
import os
import time
import numpy as np
from datetime import datetime

CAM_INDEX = 1
BASE_OUTPUT = os.path.join(os.path.dirname(__file__), "..", "vision_demo_test_res")
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
print(f"Camera: {w}x{h}")

# ── Trackbar window ───────────────────────────────────────────
CTRL = "Edge Controls"
cv2.namedWindow(CTRL, cv2.WINDOW_NORMAL)
cv2.resizeWindow(CTRL, 480, 280)

#  Method:   0=Canny  1=Sobel  2=Laplacian
cv2.createTrackbar("Method  0=Canny 1=Sobel 2=Lap", CTRL, 0, 2, lambda x: None)
#  Pre-blur Gaussian radius (0=off, values are kernel sizes: 0→off, 1→3, 2→5 …)
cv2.createTrackbar("Blur (0=off  1=3  2=5  3=7)", CTRL, 1, 5, lambda x: None)
#  Canny T1 / (unused for Lap)
cv2.createTrackbar("Canny T1  / Sobel ksize-idx", CTRL, 30, 300, lambda x: None)
#  Canny T2 / Sobel scale (*0.1)
cv2.createTrackbar("Canny T2  / Sobel scale*10", CTRL, 80, 300, lambda x: None)
#  Overlay alpha (only used in overlay display mode)
cv2.createTrackbar("Overlay alpha *10 (0-10)", CTRL, 6, 10, lambda x: None)
# CLAHE: 0=off, 1-8 = clipLimit (x1)
cv2.createTrackbar("CLAHE clip (0=off  1-8)", CTRL, 3, 8, lambda x: None)
# CLAHE tile grid size index: 0=4, 1=8, 2=16
cv2.createTrackbar("CLAHE tile 0=4 1=8 2=16", CTRL, 1, 2, lambda x: None)

METHODS = ["Canny", "Sobel", "Laplacian"]
SOBEL_KSIZES = [1, 3, 5, 7]   # indexed by Canny-T1 trackbar (0-3)

# ── Main display window ───────────────────────────────────────
SCREEN_W, SCREEN_H = 1920, 1020
# Side-by-side: each panel is w/2 × h (half-width preview)
panel_w = SCREEN_W // 2
scale_p = min(panel_w / w, SCREEN_H / h)
p_w = int(w * scale_p)
p_h = int(h * scale_p)

WIN = "demo4 | LEFT=original  RIGHT=edge  [d]mode [s]snap [v]rec [q]quit"
cv2.namedWindow(WIN, cv2.WINDOW_NORMAL)
cv2.resizeWindow(WIN, p_w * 2, p_h)

# ── State ─────────────────────────────────────────────────────
recording = False
writer = None
snapshot_count = 0
display_mode = 0   # 0=side-by-side  1=overlay  2=edge-only
fps_buf = []
t_prev = time.perf_counter()

# ── Helpers ───────────────────────────────────────────────────
def metric_color(val, green, yellow):
    if green[0] <= val <= green[1]:   return (0, 210, 0)
    elif yellow[0] <= val <= yellow[1]: return (0, 200, 220)
    else: return (0, 60, 220)

def compute_metrics(frame):
    gray_u8 = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    gray    = gray_u8.astype(np.float32)
    sharpness  = float(cv2.Laplacian(gray_u8, cv2.CV_32F).var())
    brightness = float(gray.mean())
    contrast   = float(gray.std())
    overexpose = float((gray > 245).mean() * 100)
    return sharpness, brightness, contrast, overexpose

def apply_clahe(gray_u8, clip, tile_idx):
    if clip == 0:
        return gray_u8
    tile = [4, 8, 16][tile_idx]
    clahe = cv2.createCLAHE(clipLimit=float(clip), tileGridSize=(tile, tile))
    return clahe.apply(gray_u8)

def apply_blur(gray_u8, blur_idx):
    if blur_idx == 0:
        return gray_u8
    k = blur_idx * 2 + 1   # 1→3, 2→5, 3→7, 4→9, 5→11
    return cv2.GaussianBlur(gray_u8, (k, k), 0)

def detect_edges(gray_u8, method, blur_idx, p1, p2, clahe_clip, clahe_tile):
    enhanced = apply_clahe(gray_u8, clahe_clip, clahe_tile)
    blurred  = apply_blur(enhanced, blur_idx)
    if method == 0:   # Canny
        t1, t2 = max(1, p1), max(1, p2)
        edges = cv2.Canny(blurred, t1, t2)
    elif method == 1:  # Sobel
        ki = min(p1, 3)   # index 0-3 → ksize 1,3,5,7
        ksize = SOBEL_KSIZES[ki]
        scale = max(1, p2) * 0.1
        sx = cv2.Sobel(blurred, cv2.CV_32F, 1, 0, ksize=ksize, scale=scale)
        sy = cv2.Sobel(blurred, cv2.CV_32F, 0, 1, ksize=ksize, scale=scale)
        mag = cv2.magnitude(sx, sy)
        mag = np.clip(mag, 0, 255).astype(np.uint8)
        edges = mag
    else:              # Laplacian
        lap = cv2.Laplacian(blurred, cv2.CV_32F)
        lap = np.clip(np.abs(lap), 0, 255).astype(np.uint8)
        edges = lap
    return edges

def draw_hud(frame, fps, sharpness, brightness, contrast, overexpose,
             method_name, blur_idx, p1, p2, clahe_clip, clahe_tile, recording):
    ov = frame.copy()
    cv2.rectangle(ov, (8, 8), (470, 255), (20, 20, 20), -1)
    cv2.addWeighted(ov, 0.5, frame, 0.5, 0, frame)

    def put(text, y, color):
        cv2.putText(frame, text, (18, y), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (0,0,0), 3)
        cv2.putText(frame, text, (18, y), cv2.FONT_HERSHEY_SIMPLEX, 0.62, color, 1)

    put(f"FPS        {fps:5.1f}",        36,  metric_color(fps,        (25,999),   (15,25)))
    put(f"Sharpness  {sharpness:7.1f}",  62,  metric_color(sharpness,  (150,99999),(60,150)))
    put(f"Brightness {brightness:5.1f}", 88,  metric_color(brightness, (80,180),   (50,220)))
    put(f"Contrast   {contrast:5.1f}",   114, metric_color(contrast,   (40,999),   (20,40)))
    put(f"Overexpose {overexpose:4.1f}%", 140, metric_color(overexpose, (0,5),      (5,15)))

    blur_str = "off" if blur_idx == 0 else f"k={blur_idx*2+1}"
    clahe_str = "off" if clahe_clip == 0 else f"clip={clahe_clip} tile={[4,8,16][clahe_tile]}"
    if method_name == "Canny":
        param_str = f"blur={blur_str}  T1={p1}  T2={p2}"
    elif method_name == "Sobel":
        ki = min(p1, 3)
        param_str = f"blur={blur_str}  ksize={SOBEL_KSIZES[ki]}  scale={max(1,p2)*0.1:.1f}"
    else:
        param_str = f"blur={blur_str}"

    put(f"Method: {method_name}  {param_str}", 170, (220, 200, 80))
    put(f"CLAHE:  {clahe_str}", 196, (180, 180, 80))

    if recording:
        cv2.circle(frame, (w - 40, 30), 12, (0, 0, 220), -1)
        put("REC", 42, (0, 40, 220))

    cv2.putText(frame, "[s]snap [v]rec [d]mode [q]quit",
                (18, h - 14), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (150,150,150), 1)

def build_display(orig, edges_gray, mode, alpha):
    edge_bgr = cv2.cvtColor(edges_gray, cv2.COLOR_GRAY2BGR)
    if mode == 0:   # side-by-side
        combined = np.hstack([orig, edge_bgr])
        return cv2.resize(combined, (p_w * 2, p_h))
    elif mode == 1:  # overlay
        blended = cv2.addWeighted(orig, 1.0, edge_bgr, alpha, 0)
        return cv2.resize(blended, (p_w * 2, p_h))
    else:            # edge-only
        return cv2.resize(edge_bgr, (p_w * 2, p_h))


# ── Main loop ─────────────────────────────────────────────────
while True:
    ret, frame = cap.read()
    if not ret:
        print("Frame read failed.")
        break

    # FPS
    t_now = time.perf_counter()
    fps_buf.append(1.0 / max(t_now - t_prev, 1e-6))
    t_prev = t_now
    if len(fps_buf) > 30: fps_buf.pop(0)
    fps = float(np.mean(fps_buf))

    # Read trackbars
    method   = cv2.getTrackbarPos("Method  0=Canny 1=Sobel 2=Lap", CTRL)
    blur_idx = cv2.getTrackbarPos("Blur (0=off  1=3  2=5  3=7)", CTRL)
    p1       = cv2.getTrackbarPos("Canny T1  / Sobel ksize-idx", CTRL)
    p2       = cv2.getTrackbarPos("Canny T2  / Sobel scale*10", CTRL)
    alpha      = cv2.getTrackbarPos("Overlay alpha *10 (0-10)", CTRL) / 10.0
    clahe_clip = cv2.getTrackbarPos("CLAHE clip (0=off  1-8)", CTRL)
    clahe_tile = cv2.getTrackbarPos("CLAHE tile 0=4 1=8 2=16", CTRL)
    method_name = METHODS[method]

    # Metrics & edge
    sharpness, brightness, contrast, overexpose = compute_metrics(frame)
    gray_u8 = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    edges   = detect_edges(gray_u8, method, blur_idx, p1, p2, clahe_clip, clahe_tile)

    # Recording (original frame, no HUD)
    if recording and writer:
        writer.write(frame)

    # HUD drawn on a copy for display
    disp_frame = frame.copy()
    draw_hud(disp_frame, fps, sharpness, brightness, contrast, overexpose,
             method_name, blur_idx, p1, p2, clahe_clip, clahe_tile, recording)

    canvas = build_display(disp_frame, edges, display_mode, alpha)
    cv2.imshow(WIN, canvas)

    key = cv2.waitKey(1) & 0xFF

    if key == ord('q'):
        break

    elif key == ord('d'):
        display_mode = (display_mode + 1) % 3
        modes = ["side-by-side", "overlay", "edge-only"]
        print(f"Display mode: {modes[display_mode]}")

    elif key == ord('s'):
        ts = datetime.now().strftime('%H%M%S')
        orig_path = os.path.join(OUTPUT_DIR, f"snap_{ts}_{snapshot_count:03d}_orig.png")
        edge_path = os.path.join(OUTPUT_DIR, f"snap_{ts}_{snapshot_count:03d}_edge.png")
        cv2.imwrite(orig_path, frame)
        cv2.imwrite(edge_path, cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR))
        print(f"Saved: {orig_path}")
        print(f"Saved: {edge_path}")
        snapshot_count += 1

    elif key == ord('v'):
        if not recording:
            vname = os.path.join(OUTPUT_DIR,
                                 f"video_{datetime.now().strftime('%H%M%S')}.mp4")
            fourcc = cv2.VideoWriter_fourcc(*"mp4v")
            fps_w = cap.get(cv2.CAP_PROP_FPS)
            fps_w = fps_w if fps_w > 0 else 30.0
            writer = cv2.VideoWriter(vname, fourcc, fps_w, (w, h))
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
