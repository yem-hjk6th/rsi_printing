"""
demo4_stream_test.py — TESLONG NTG100 live HUD + parameter tuning + stability test

Built from demo3 findings: DSHOW backend, modes 640x480 / 1280x720,
fps locked at 30, 9 UVC controls actually writable.

Interactive HUD shows FPS / Sharpness / Brightness / Contrast / Overexpose.

Keys:
    [b]/[B]   BRIGHTNESS  -/+ 10
    [c]/[C]   CONTRAST    -/+ 10
    [r]       re-read driver state
    [1]       switch resolution to 640x480
    [2]       switch resolution to 1280x720
    [t]       run 30s no-write + 30s write stability test
    [s]       snapshot
    [v]       toggle continuous recording
    [q]/ESC   quit

DO NOT add an EXPOSURE key: cap.set(CAP_PROP_EXPOSURE, anything) bricks
NTG100's UVC pipeline into a black-frame state that only USB replug
recovers. Verified 2026-05-12.

Output: Vision/bore_scope/vision_demo_test_res/<session_ts>/
"""

import cv2
import json
import os
import time
import numpy as np
from datetime import datetime


CAM_INDEX = 1
BACKEND = cv2.CAP_DSHOW

RES_LOW  = (640, 480)
RES_HIGH = (1280, 720)
DEFAULT_RES = RES_HIGH

FPS_NOMINAL = 30.0
STABILITY_SECONDS = 30

DEFAULTS = {
    "BRIGHTNESS": 128,
    "CONTRAST":   38,
    "SATURATION": 64,
    "HUE":        0,
    "SHARPNESS":  2,
    "WB_TEMP":    4000,
    "AUTO_WB":    1,
    "BACKLIGHT":  0,
}

# EXPOSURE intentionally excluded — see top-of-file warning.
CONTROL_MAP = {
    "BRIGHTNESS": cv2.CAP_PROP_BRIGHTNESS,
    "CONTRAST":   cv2.CAP_PROP_CONTRAST,
    "SATURATION": cv2.CAP_PROP_SATURATION,
    "HUE":        cv2.CAP_PROP_HUE,
    "SHARPNESS":  cv2.CAP_PROP_SHARPNESS,
    "WB_TEMP":    cv2.CAP_PROP_WB_TEMPERATURE,
    "AUTO_WB":    cv2.CAP_PROP_AUTO_WB,
    "BACKLIGHT":  cv2.CAP_PROP_BACKLIGHT,
}

SCREEN_W, SCREEN_H = 1920, 1020
WINDOW_NAME = "NTG100 Stream Test"


session_ts = datetime.now().strftime("%Y%m%d_%H%M%S")
BASE_OUT = os.path.join(os.path.dirname(__file__), "..", "vision_demo_test_res")
SESSION_DIR = os.path.join(BASE_OUT, f"borescope_{session_ts}")
os.makedirs(SESSION_DIR, exist_ok=True)
print(f"Output dir: {SESSION_DIR}")


def metric_color(val, green, yellow):
    if green[0] <= val <= green[1]:
        return (0, 210, 0)
    if yellow[0] <= val <= yellow[1]:
        return (0, 200, 220)
    return (0, 60, 220)


def compute_metrics(frame):
    gray_u8 = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    gray = gray_u8.astype(np.float32)
    sharpness  = float(cv2.Laplacian(gray_u8, cv2.CV_32F).var())
    brightness = float(gray.mean())
    contrast   = float(gray.std())
    overexpose = float((gray > 245).mean() * 100)
    return sharpness, brightness, contrast, overexpose


def read_controls(cap):
    """Read current values from the camera, do NOT write — writing all 9
    at startup put NTG100 into a black-frame state requiring USB replug."""
    out = {}
    for name, prop in CONTROL_MAP.items():
        v = cap.get(prop)
        if v != -1.0:
            out[name] = int(v)
    return out


def open_cam(res):
    cap = cv2.VideoCapture(CAM_INDEX, BACKEND)
    if not cap.isOpened():
        return None
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, res[0])
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, res[1])
    return cap


def resize_window(w, h):
    scale = min(SCREEN_W / w, SCREEN_H / h)
    cv2.resizeWindow(WINDOW_NAME, int(w * scale), int(h * scale))


def draw_hud(frame, ctrl, fps, sharpness, brightness, contrast, overexpose,
             actual_w, actual_h, recording, test_status=""):
    h = frame.shape[0]
    overlay = frame.copy()
    cv2.rectangle(overlay, (10, 10), (520, 280), (20, 20, 20), -1)
    cv2.addWeighted(overlay, 0.55, frame, 0.45, 0, frame)

    def put(text, y, color):
        cv2.putText(frame, text, (20, y), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (0, 0, 0), 3)
        cv2.putText(frame, text, (20, y), cv2.FONT_HERSHEY_SIMPLEX, 0.62, color, 1)

    res_c = (0, 210, 0) if (actual_w, actual_h) == RES_HIGH else (0, 200, 220)
    put(f"Res        {actual_w}x{actual_h}", 38, res_c)

    fps_c = metric_color(fps, (25, 999), (15, 25))
    put(f"FPS        {fps:5.1f}", 64, fps_c)

    sh_c = metric_color(sharpness, (150, 99999), (60, 150))
    put(f"Sharpness  {sharpness:7.1f}  (Lap.Var)", 90, sh_c)

    br_c = metric_color(brightness, (80, 180), (50, 220))
    put(f"Brightness {brightness:5.1f} /255", 116, br_c)

    co_c = metric_color(contrast, (40, 999), (20, 40))
    put(f"Contrast   {contrast:5.1f}  (StdDev)", 142, co_c)

    ov_c = metric_color(overexpose, (0, 5), (5, 15))
    put(f"Overexpose {overexpose:5.1f} %", 168, ov_c)

    put(f"Ctrl  B={ctrl.get('BRIGHTNESS', '-'):>4}  C={ctrl.get('CONTRAST', '-'):>4}  "
        f"S={ctrl.get('SHARPNESS', '-'):>2}", 198, (200, 200, 200))

    if test_status:
        put(test_status, 228, (0, 220, 220))

    if recording:
        cv2.circle(frame, (frame.shape[1] - 40, 30), 12, (0, 0, 220), -1)
        cv2.putText(frame, "REC", (frame.shape[1] - 25, 38),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 220), 2)

    cv2.putText(frame,
                "[b/B][c/C] tune  [r] re-read  [1]/[2] res  [t] stability  [s] snap  [v] rec  [q] quit",
                (20, h - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (160, 160, 160), 1)

    return frame


def stability_test(cap, ctrl, res, session_dir):
    """30s read-only then 30s read+write; report fps drops, sharpness, write latency."""
    w, h = res
    print(f"\n[STABILITY] Phase 1/2: read-only for {STABILITY_SECONDS}s @ {w}x{h}")

    def run_phase(writer=None):
        per_sec_count = {}
        sharp_buf = []
        write_lat_ms = []
        n_ok = n_fail = 0
        t_start = time.perf_counter()
        last_print = t_start
        while True:
            t = time.perf_counter()
            if t - t_start >= STABILITY_SECONDS:
                break
            ok, frame = cap.read()
            if not ok:
                n_fail += 1
                continue
            n_ok += 1
            sec_bucket = int(t - t_start)
            per_sec_count[sec_bucket] = per_sec_count.get(sec_bucket, 0) + 1
            sharp_buf.append(float(cv2.Laplacian(
                cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY), cv2.CV_32F).var()))
            if writer is not None:
                t_w0 = time.perf_counter()
                writer.write(frame)
                write_lat_ms.append((time.perf_counter() - t_w0) * 1000.0)
            if t - last_print >= 1.0:
                inst = per_sec_count.get(sec_bucket, 0)
                print(f"  t={sec_bucket+1:02d}s  fps={inst:>3}  "
                      f"sharpness={np.mean(sharp_buf[-30:]):6.1f}")
                last_print = t

        elapsed = time.perf_counter() - t_start
        return {
            "elapsed_s": round(elapsed, 3),
            "frames_ok": n_ok,
            "frames_fail": n_fail,
            "avg_fps": round(n_ok / elapsed, 2),
            "min_fps_per_sec": min(per_sec_count.values()) if per_sec_count else 0,
            "max_fps_per_sec": max(per_sec_count.values()) if per_sec_count else 0,
            "avg_sharpness": round(float(np.mean(sharp_buf)), 2) if sharp_buf else 0,
            "write_lat_ms_avg": round(float(np.mean(write_lat_ms)), 2) if write_lat_ms else None,
            "write_lat_ms_p95": round(float(np.percentile(write_lat_ms, 95)), 2) if write_lat_ms else None,
        }

    phase1 = run_phase(writer=None)

    print(f"\n[STABILITY] Phase 2/2: read + mp4 write for {STABILITY_SECONDS}s")
    test_video = os.path.join(session_dir, f"stability_{w}x{h}.mp4")
    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    writer = cv2.VideoWriter(test_video, fourcc, FPS_NOMINAL, (w, h))
    phase2 = run_phase(writer=writer)
    writer.release()

    result = {
        "timestamp": datetime.now().isoformat(),
        "resolution": [w, h],
        "fps_nominal": FPS_NOMINAL,
        "test_seconds_each_phase": STABILITY_SECONDS,
        "controls_during_test": ctrl,
        "phase1_read_only": phase1,
        "phase2_read_write": phase2,
        "test_video": test_video,
    }
    json_path = os.path.join(session_dir,
                             f"stability_{w}x{h}_{datetime.now().strftime('%H%M%S')}.json")
    with open(json_path, "w") as f:
        json.dump(result, f, indent=2)

    print(f"\n[STABILITY SUMMARY @ {w}x{h}]")
    print(f"  phase1 (no write):  fps={phase1['avg_fps']}  min/max per-sec={phase1['min_fps_per_sec']}/{phase1['max_fps_per_sec']}  sharp={phase1['avg_sharpness']}")
    print(f"  phase2 (with mp4):  fps={phase2['avg_fps']}  min/max per-sec={phase2['min_fps_per_sec']}/{phase2['max_fps_per_sec']}  sharp={phase2['avg_sharpness']}")
    print(f"  write latency avg/p95 = {phase2['write_lat_ms_avg']} / {phase2['write_lat_ms_p95']} ms")
    print(f"  -> {json_path}")
    print(f"  -> {test_video}")


def main():
    res = DEFAULT_RES

    cap = open_cam(res)
    if cap is None:
        print(f"[err] cannot open camera idx {CAM_INDEX}")
        return

    ctrl = read_controls(cap)
    for name, default in DEFAULTS.items():
        ctrl.setdefault(name, default)

    actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"Opened: {actual_w}x{actual_h}  (requested {res})")
    print(f"Driver controls: {ctrl}")
    print("Controls: [b/B][c/C] tune  [r] re-read  [1]/[2] res  [t] stability  [s] snap  [v] rec  [q] quit")

    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
    resize_window(actual_w, actual_h)

    fps_buf = []
    t_prev = time.perf_counter()
    recording = False
    writer = None
    snap_count = 0
    test_status = ""

    while True:
        ok, frame = cap.read()
        if not ok:
            print("[err] frame read failed")
            break

        t_now = time.perf_counter()
        fps_buf.append(1.0 / max(t_now - t_prev, 1e-6))
        t_prev = t_now
        if len(fps_buf) > 30:
            fps_buf.pop(0)
        fps = float(np.mean(fps_buf))

        sharpness, brightness, contrast, overexpose = compute_metrics(frame)

        if recording and writer:
            writer.write(frame)

        draw_hud(frame, ctrl, fps, sharpness, brightness, contrast, overexpose,
                 actual_w, actual_h, recording, test_status)
        cv2.imshow(WINDOW_NAME, frame)
        test_status = ""

        key = cv2.waitKey(1) & 0xFF
        if key in (ord('q'), 27):
            break

        elif key == ord('b'):
            ctrl["BRIGHTNESS"] = max(0, ctrl["BRIGHTNESS"] - 10)
            cap.set(cv2.CAP_PROP_BRIGHTNESS, ctrl["BRIGHTNESS"])
        elif key == ord('B'):
            ctrl["BRIGHTNESS"] = min(255, ctrl["BRIGHTNESS"] + 10)
            cap.set(cv2.CAP_PROP_BRIGHTNESS, ctrl["BRIGHTNESS"])
        elif key == ord('c'):
            ctrl["CONTRAST"] = max(0, ctrl["CONTRAST"] - 10)
            cap.set(cv2.CAP_PROP_CONTRAST, ctrl["CONTRAST"])
        elif key == ord('C'):
            ctrl["CONTRAST"] = min(255, ctrl["CONTRAST"] + 10)
            cap.set(cv2.CAP_PROP_CONTRAST, ctrl["CONTRAST"])
        elif key == ord('r'):
            ctrl = read_controls(cap)
            for name, default in DEFAULTS.items():
                ctrl.setdefault(name, default)
            print(f"[reset] re-read driver state: {ctrl}")

        elif key == ord('1') and (actual_w, actual_h) != RES_LOW:
            if recording and writer:
                recording = False
                writer.release()
                writer = None
                print("[rec] stopped before resolution switch")
            cap.release()
            res = RES_LOW
            cap = open_cam(res)
            actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            resize_window(actual_w, actual_h)
            fps_buf.clear()
            print(f"[res] -> {actual_w}x{actual_h}")
        elif key == ord('2') and (actual_w, actual_h) != RES_HIGH:
            if recording and writer:
                recording = False
                writer.release()
                writer = None
                print("[rec] stopped before resolution switch")
            cap.release()
            res = RES_HIGH
            cap = open_cam(res)
            actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            resize_window(actual_w, actual_h)
            fps_buf.clear()
            print(f"[res] -> {actual_w}x{actual_h}")

        elif key == ord('t'):
            if recording and writer:
                recording = False
                writer.release()
                writer = None
                print("[rec] stopped before stability test")
            stability_test(cap, ctrl, (actual_w, actual_h), SESSION_DIR)
            fps_buf.clear()
            t_prev = time.perf_counter()

        elif key == ord('s'):
            fname = os.path.join(
                SESSION_DIR,
                f"snap_{datetime.now().strftime('%H%M%S')}_{snap_count:03d}.png")
            cv2.imwrite(fname, frame)
            print(f"[snap] {fname}")
            snap_count += 1

        elif key == ord('v'):
            if not recording:
                vname = os.path.join(
                    SESSION_DIR,
                    f"video_{actual_w}x{actual_h}_{datetime.now().strftime('%H%M%S')}.mp4")
                fourcc = cv2.VideoWriter_fourcc(*"mp4v")
                writer = cv2.VideoWriter(vname, fourcc, FPS_NOMINAL,
                                         (actual_w, actual_h))
                recording = True
                print(f"[rec] start {vname}")
            else:
                recording = False
                writer.release()
                writer = None
                print("[rec] stop")

    if recording and writer:
        writer.release()
    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
