"""
demo3_probe_params.py — TESLONG NTG100 UVC parameter capability probe (v2)

What changed vs v1:
  - Tries both DSHOW and MSMF backends; NTG100 may expose different UVC
    controls under each
  - Per-parameter test values (resolution uses real (W,H) tuples, exposure
    uses negative powers-of-2 per Windows log-scale, on/off uses 0/1)
  - FPS "writable" is verified by ACTUAL frame timing, not cached get()
  - Resolution check uses frame.shape (source of truth), not cap.get()
  - fourcc decoder handles negative int32 properly
  - Always re-open cap between mutating tests so a bad set() can't poison
    later readings

Usage:
    python demo3_probe_params.py            # auto-find first working index
    python demo3_probe_params.py 1          # NTG100 is at index 1 on this PC
"""

import cv2
import json
import os
import sys
import time
from datetime import datetime


BACKENDS = [("DSHOW", cv2.CAP_DSHOW), ("MSMF", cv2.CAP_MSMF)]


RES_MODES = [
    (640, 480),
    (800, 600),
    (1024, 768),
    (1280, 720),
    (1280, 960),
    (1920, 1080),
]


FPS_TARGETS = [5, 10, 15, 30, 60]


CONTROL_PROPS = [
    ("BRIGHTNESS",     cv2.CAP_PROP_BRIGHTNESS,     [0, 64, 128, 192]),
    ("CONTRAST",       cv2.CAP_PROP_CONTRAST,       [0, 64, 128, 192]),
    ("SATURATION",     cv2.CAP_PROP_SATURATION,     [0, 64, 128, 192]),
    ("HUE",            cv2.CAP_PROP_HUE,            [-90, 0, 90]),
    ("GAIN",           cv2.CAP_PROP_GAIN,           [0, 32, 64, 128]),
    ("EXPOSURE",       cv2.CAP_PROP_EXPOSURE,       [-1, -3, -5, -7, -9, -11, -13]),
    ("AUTO_EXPOSURE",  cv2.CAP_PROP_AUTO_EXPOSURE,  [0, 0.25, 0.75, 1, 3]),
    ("AUTO_WB",        cv2.CAP_PROP_AUTO_WB,        [0, 1]),
    ("WB_TEMPERATURE", cv2.CAP_PROP_WB_TEMPERATURE, [2800, 4000, 5500, 6500]),
    ("SHARPNESS",      cv2.CAP_PROP_SHARPNESS,      [0, 2, 4, 6]),
    ("GAMMA",          cv2.CAP_PROP_GAMMA,          [50, 100, 200]),
    ("BACKLIGHT",      cv2.CAP_PROP_BACKLIGHT,      [0, 1, 2]),
    ("FOCUS",          cv2.CAP_PROP_FOCUS,          [0, 50, 100]),
    ("AUTOFOCUS",      cv2.CAP_PROP_AUTOFOCUS,      [0, 1]),
    ("ZOOM",           cv2.CAP_PROP_ZOOM,           [100, 150, 200]),
]


def fourcc_to_str(value) -> str:
    """Decode FOURCC int (possibly negative due to int32 sign) into 4-char string."""
    v = int(value) & 0xFFFFFFFF
    if v == 0 or v == 0xFFFFFFFF:
        return ""
    chars = [chr((v >> (8 * i)) & 0xFF) for i in range(4)]
    if all(32 <= ord(c) < 127 for c in chars):
        return "".join(chars)
    return f"<raw:{v:08x}>"


def list_cameras(max_idx: int = 8) -> list:
    """Probe each index under DSHOW, return list of (idx, w, h, fourcc_str)."""
    found = []
    for idx in range(max_idx):
        cap = cv2.VideoCapture(idx, cv2.CAP_DSHOW)
        if not cap.isOpened():
            cap.release()
            continue
        ok, _ = cap.read()
        if ok:
            w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            fcc = fourcc_to_str(cap.get(cv2.CAP_PROP_FOURCC))
            found.append((idx, w, h, fcc))
        cap.release()
    return found


def probe_resolutions(idx: int, backend_id: int) -> list:
    """For each (w,h) mode, open fresh cap, request, read back via frame.shape."""
    achieved = []
    for w, h in RES_MODES:
        cap = cv2.VideoCapture(idx, backend_id)
        if not cap.isOpened():
            cap.release()
            continue
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
        ok, frame = cap.read()
        if ok and frame is not None:
            achieved.append({
                "requested": [w, h],
                "actual": [int(frame.shape[1]), int(frame.shape[0])],
                "match": frame.shape[1] == w and frame.shape[0] == h,
            })
        cap.release()
    return achieved


def measure_actual_fps(idx: int, backend_id: int, target_fps: int,
                       resolution=(1280, 720), n_frames: int = 30) -> dict:
    """Open cap, set FPS, read n_frames, time them."""
    cap = cv2.VideoCapture(idx, backend_id)
    if not cap.isOpened():
        return {"target": target_fps, "opened": False}
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, resolution[0])
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, resolution[1])
    cap.set(cv2.CAP_PROP_FPS, target_fps)

    for _ in range(5):
        cap.read()

    t0 = time.perf_counter()
    n_ok = 0
    for _ in range(n_frames):
        ok, _ = cap.read()
        if ok:
            n_ok += 1
    dt = time.perf_counter() - t0
    cap.release()

    return {
        "target": target_fps,
        "opened": True,
        "frames_ok": n_ok,
        "elapsed_s": round(dt, 3),
        "actual_fps": round(n_ok / dt, 2) if dt > 0 else 0,
    }


def probe_one_control(idx: int, backend_id: int, name: str, pid: int,
                      test_values: list) -> dict:
    """Open fresh cap, read initial, try each test value, see if get() reflects it."""
    cap = cv2.VideoCapture(idx, backend_id)
    if not cap.isOpened():
        cap.release()
        return {"name": name, "readable": False, "writable": False, "accepted": []}

    initial = cap.get(pid)
    readable = initial != -1.0

    accepted = []
    if readable:
        for v in test_values:
            ok_set = cap.set(pid, v)
            time.sleep(0.05)
            got = cap.get(pid)
            if ok_set and got != -1.0:
                tol = max(0.5, abs(v) * 0.1)
                if abs(got - v) <= tol:
                    accepted.append({"set": v, "got": round(got, 3)})

    cap.release()
    return {
        "name": name,
        "initial": initial,
        "readable": readable,
        "writable": len(accepted) > 0,
        "accepted": accepted,
    }


def probe_backend(idx: int, backend_name: str, backend_id: int) -> dict:
    """Run identity + resolution + fps + control probes for one backend."""
    print(f"\n{'=' * 80}")
    print(f"BACKEND: {backend_name}  (idx={idx})")
    print('=' * 80)

    cap = cv2.VideoCapture(idx, backend_id)
    if not cap.isOpened():
        cap.release()
        print(f"  [!] cannot open camera with {backend_name}")
        return {"backend": backend_name, "opened": False}

    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    default_fps = cap.get(cv2.CAP_PROP_FPS)
    fcc = fourcc_to_str(cap.get(cv2.CAP_PROP_FOURCC))
    cap.release()

    print(f"  identity: {w}x{h}  fourcc={fcc!r}  default_fps={default_fps:.1f}")

    print(f"\n  resolution probe:")
    print(f"    {'Requested':<12} {'Actual':<12} match")
    print(f"    " + "-" * 35)
    res_results = probe_resolutions(idx, backend_id)
    for r in res_results:
        req = f"{r['requested'][0]}x{r['requested'][1]}"
        act = f"{r['actual'][0]}x{r['actual'][1]}"
        print(f"    {req:<12} {act:<12} {r['match']}")

    print(f"\n  fps probe (target -> actual measured, @ 1280x720, 30 frames):")
    print(f"    {'Target':<8} {'Actual':<8} {'Frames OK':<10} elapsed")
    print(f"    " + "-" * 40)
    fps_results = []
    for t in FPS_TARGETS:
        r = measure_actual_fps(idx, backend_id, t)
        fps_results.append(r)
        if r.get("opened"):
            print(f"    {r['target']:<8} {r['actual_fps']:<8} {r['frames_ok']:<10} {r['elapsed_s']}s")

    print(f"\n  UVC control probe:")
    print(f"    {'Param':<16} {'Read':<5} {'Write':<6} {'Initial':>10}  {'Accepted (set->got)'}")
    print(f"    " + "-" * 90)
    ctrl_results = []
    for name, pid, vals in CONTROL_PROPS:
        r = probe_one_control(idx, backend_id, name, pid, vals)
        ctrl_results.append(r)
        init = f"{r['initial']:.2f}" if r["readable"] else "-1"
        acc = ", ".join(f"{a['set']}->{a['got']:.1f}" for a in r["accepted"][:5])
        if len(r["accepted"]) > 5:
            acc += f"  (+{len(r['accepted']) - 5})"
        print(f"    {r['name']:<16} {str(r['readable']):<5} {str(r['writable']):<6} "
              f"{init:>10}  {acc}")

    return {
        "backend": backend_name,
        "opened": True,
        "identity": {"width": w, "height": h, "fourcc": fcc, "default_fps": default_fps},
        "resolution_probe": res_results,
        "fps_probe": fps_results,
        "control_probe": ctrl_results,
    }


def main(camera_index: int = -1):
    print("[probe] enumerating cameras under DSHOW ...")
    cams = list_cameras()
    if not cams:
        print("[probe] ERROR: no cameras found")
        sys.exit(1)
    for i, w, h, fcc in cams:
        marker = "  <- requested" if i == camera_index else ""
        print(f"  idx={i:<3} {w}x{h:<5} fourcc={fcc!r}{marker}")

    if camera_index < 0:
        camera_index = cams[0][0]
        print(f"[probe] no index given, using idx={camera_index}")

    summary = {"timestamp": datetime.now().isoformat(), "camera_index": camera_index,
               "cameras_found": cams, "backends": []}

    for name, bid in BACKENDS:
        summary["backends"].append(probe_backend(camera_index, name, bid))

    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    out_path = os.path.join(os.path.dirname(__file__), f"demo3_probe_{ts}.json")
    with open(out_path, "w") as f:
        json.dump(summary, f, indent=2, default=str)
    print(f"\n[probe] full report -> {out_path}")

    print(f"\n[probe] takeaway:")
    for b in summary["backends"]:
        if not b.get("opened"):
            continue
        n_writable = sum(1 for r in b.get("control_probe", []) if r["writable"])
        n_resolutions = sum(1 for r in b.get("resolution_probe", []) if r["match"])
        print(f"  {b['backend']:<8} writable_controls={n_writable}  matched_resolutions={n_resolutions}")


if __name__ == "__main__":
    idx = int(sys.argv[1]) if len(sys.argv) > 1 else -1
    main(idx)
