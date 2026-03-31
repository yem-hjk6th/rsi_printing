"""
init_recorder_svo_csv.py — RSI-triggered ZED SVO2 recorder + CSV logger
Auto-starts recording when RSI communication is detected,
auto-stops when RSI communication is lost.
SVO2 is split into segments every SEGMENT_MINUTES to avoid large file corruption.
Output: rsi_printing/recorded_data/<session_ts>/recording_<session_ts>_<idx>_<end_ts>.svo2
        rsi_printing/rsi_data/rsi_data_<session_ts>.csv
"""

import csv
import ctypes
import json
import os
import socket
import threading
import time
import xml.etree.ElementTree as ET
from datetime import datetime

# ZED SDK DLL paths (must be added before importing pyzed)
if os.name == "nt":
    for p in [
        r"C:\Program Files (x86)\ZED SDK\bin",
        r"C:\Program Files (x86)\ZED SDK\dependencies\bin",
        r"C:\Program Files\NVIDIA GPU Computing Toolkit\CUDA\v12.8\bin",
    ]:
        if os.path.isdir(p):
            os.add_dll_directory(p)

import cv2
import numpy as np
import pyzed.sl as sl

# ─── Windows power management: prevent sleep & screen lock during recording ───

# SetThreadExecutionState flags
ES_CONTINUOUS       = 0x80000000
ES_SYSTEM_REQUIRED  = 0x00000001
ES_DISPLAY_REQUIRED = 0x00000002

def prevent_sleep():
    """Prevent Windows from sleeping or turning off display."""
    if os.name == "nt":
        ctypes.windll.kernel32.SetThreadExecutionState(
            ES_CONTINUOUS | ES_SYSTEM_REQUIRED | ES_DISPLAY_REQUIRED)
        print("[PWR] Screen lock & sleep prevention ENABLED")

def allow_sleep():
    """Re-allow normal Windows sleep/display behavior."""
    if os.name == "nt":
        ctypes.windll.kernel32.SetThreadExecutionState(ES_CONTINUOUS)
        print("[PWR] Screen lock & sleep prevention DISABLED")

# ─── Config ───────────────────────────────────────────────────────────────────

RSI_HOST = "0.0.0.0"
RSI_PORT = 59152

ZED_RESOLUTION = sl.RESOLUTION.HD2K
ZED_FPS = 15
ZED_DEPTH_MODE = sl.DEPTH_MODE.NEURAL
ZED_CODEC = sl.SVO_COMPRESSION_MODE.H264

CSV_LOG_HZ = 15
CSV_FLUSH_EVERY = 100

# SVO2 segment duration in minutes — file is split to avoid large-file corruption
SEGMENT_MINUTES = 15

# RSI disconnect timeout: if no packet received for this many seconds, consider RSI disconnected
RSI_DISCONNECT_TIMEOUT = 2.0

# ─── Output paths: rsi_printing/recorded_data/ and rsi_printing/rsi_data/ ─────
# Navigate from this file up to rsi_printing root
RSI_PRINTING_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
SVO_OUTPUT_DIR = os.path.join(RSI_PRINTING_ROOT, "recorded_data")
CSV_OUTPUT_DIR = os.path.join(RSI_PRINTING_ROOT, "rsi_data")

# ─── RSI ──────────────────────────────────────────────────────────────────────

def parse_rsi_xml(data: bytes):
    try:
        root = ET.fromstring(data)
        ipoc = root.findtext("IPOC", "0")

        rist = root.find("RIst")
        if rist is not None:
            x, y, z = float(rist.get("X", 0)), float(rist.get("Y", 0)), float(rist.get("Z", 0))
            a, b, c = float(rist.get("A", 0)), float(rist.get("B", 0)), float(rist.get("C", 0))
        else:
            x = y = z = a = b = c = 0.0

        ov = int(root.findtext("Override", "0") or 0)
        vel = int(root.findtext("Vel_Act", "0") or 0)
        return {"ipoc": ipoc, "x": x, "y": y, "z": z, "a": a, "b": b, "c": c,
                "override": ov, "vel": vel}
    except Exception:
        return None


def build_rsi_reply(ipoc: str):
    return (f'<Sen Type="ImFree">'
            f'<RKorr X="0.0000" Y="0.0000" Z="0.0000" '
            f'A="0.0000" B="0.0000" C="0.0000" />'
            f'<IPOC>{ipoc}</IPOC></Sen>').encode()


class RSIThread(threading.Thread):
    def __init__(self):
        super().__init__(daemon=True)
        self.latest = None
        self.lock = threading.Lock()
        self.running = True
        self.connected = False
        self.packet_count = 0
        self.last_packet_time = 0.0

    def run(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.settimeout(1.0)
        sock.bind((RSI_HOST, RSI_PORT))
        print(f"[RSI] Listening on {RSI_HOST}:{RSI_PORT}")

        while self.running:
            try:
                data, addr = sock.recvfrom(2048)
                self.packet_count += 1
                self.last_packet_time = time.time()
                pose = parse_rsi_xml(data)
                if pose:
                    with self.lock:
                        self.latest = pose
                    sock.sendto(build_rsi_reply(pose["ipoc"]), addr)
                    if not self.connected:
                        print(f"[RSI] Connected to {addr}")
                        self.connected = True
            except socket.timeout:
                continue
            except Exception as e:
                print(f"[RSI] Error: {e}")
        sock.close()

    def get_latest(self):
        with self.lock:
            return self.latest.copy() if self.latest else None

    def is_alive_rsi(self):
        """Return True if RSI packets were received recently."""
        if self.last_packet_time == 0.0:
            return False
        return (time.time() - self.last_packet_time) < RSI_DISCONNECT_TIMEOUT

    def stop(self):
        self.running = False


# ─── Main ─────────────────────────────────────────────────────────────────────

def main():
    # Verify output paths
    print(f"[PATH] SVO output dir: {SVO_OUTPUT_DIR}")
    print(f"[PATH] CSV output dir: {CSV_OUTPUT_DIR}")
    assert "rsi_printing" in SVO_OUTPUT_DIR, f"SVO path not under rsi_printing: {SVO_OUTPUT_DIR}"

    zed = sl.Camera()
    init_p = sl.InitParameters()
    init_p.camera_resolution = ZED_RESOLUTION
    init_p.camera_fps = ZED_FPS
    init_p.depth_mode = ZED_DEPTH_MODE
    status = zed.open(init_p)
    if status != sl.ERROR_CODE.SUCCESS:
        raise RuntimeError(f"ZED open failed: {status}")

    rsi = RSIThread()
    rsi.start()

    image = sl.Mat()
    runtime = sl.RuntimeParameters()

    # ─── Phase 1: Wait for RSI connection ─────────────────────────────────
    print("[WAIT] Waiting for RSI communication to start recording...")
    while True:
        if zed.grab(runtime) == sl.ERROR_CODE.SUCCESS:
            zed.retrieve_image(image, sl.VIEW.LEFT)
            frame = image.get_data()
            if frame.shape[2] == 4:
                frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
            cv2.putText(frame, "WAITING for RSI connection...", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
            cv2.imshow("RSI + ZED Recorder", frame)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            rsi.stop()
            zed.close()
            cv2.destroyAllWindows()
            print("[EXIT] Cancelled by user.")
            return

        if rsi.connected and rsi.is_alive_rsi():
            print("[RSI] Communication detected — starting recording.")
            break

    # ─── Phase 2: Start recording ─────────────────────────────────────────
    session_ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    svo_dir = os.path.join(SVO_OUTPUT_DIR, session_ts)
    os.makedirs(svo_dir, exist_ok=True)
    os.makedirs(CSV_OUTPUT_DIR, exist_ok=True)

    segment_idx = 1
    segment_duration = SEGMENT_MINUTES * 60

    def start_segment(idx):
        """Enable SVO2 recording for a new segment, return (path, start_time)."""
        seg_start_ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        svo_path = os.path.join(svo_dir,
                                f"recording_{session_ts}_{idx:03d}_{seg_start_ts}.svo2")
        rec_p = sl.RecordingParameters()
        rec_p.compression_mode = ZED_CODEC
        rec_p.video_filename = svo_path
        err = zed.enable_recording(rec_p)
        if err != sl.ERROR_CODE.SUCCESS:
            raise RuntimeError(f"ZED recording failed: {err}")
        print(f"[ZED] Segment {idx} → {svo_path}")
        return svo_path, time.time()

    svo_path, segment_start = start_segment(segment_idx)

    # ─── Save session metadata (precise epoch + intrinsics + baseline) ─────
    cam_info = zed.get_camera_information()
    cam_config = cam_info.camera_configuration
    cal_params = cam_config.calibration_parameters
    left_cam = cal_params.left_cam
    baseline_mm = abs(cal_params.get_camera_baseline())

    recording_start_epoch = time.time()
    session_meta = {
        "session_ts": session_ts,
        "recording_start_epoch": recording_start_epoch,
        "resolution": [cam_config.resolution.width, cam_config.resolution.height],
        "fps": int(cam_config.fps),
        "intrinsics": {
            "fx": left_cam.fx, "fy": left_cam.fy,
            "cx": left_cam.cx, "cy": left_cam.cy,
            "disto": list(left_cam.disto[:5]),
        },
        "baseline_m": baseline_mm / 1000.0,
        "depth_mode": str(ZED_DEPTH_MODE),
        "serial_number": cam_info.serial_number,
    }
    meta_path = os.path.join(svo_dir, "session_meta.json")
    with open(meta_path, "w") as f:
        json.dump(session_meta, f, indent=2)
    print(f"[META] Session metadata → {meta_path}")
    print(f"[META] Start epoch: {recording_start_epoch:.6f}")
    print(f"[META] K: fx={left_cam.fx:.1f} fy={left_cam.fy:.1f} "
          f"cx={left_cam.cx:.1f} cy={left_cam.cy:.1f} baseline={baseline_mm/1000:.4f}m")

    # Also save K.txt for direct FFS consumption
    k_path = os.path.join(svo_dir, "K.txt")
    with open(k_path, "w") as f:
        f.write(f"{left_cam.fx} 0.0 {left_cam.cx} 0.0 {left_cam.fy} {left_cam.cy} 0.0 0.0 1.0\n")
        f.write(f"{baseline_mm / 1000.0}\n")

    csv_path = os.path.join(CSV_OUTPUT_DIR, f"rsi_data_{session_ts}.csv")
    csv_file = open(csv_path, "w", newline="")
    writer = csv.writer(csv_file)
    writer.writerow(["timestamp", "ipoc", "x_mm", "y_mm", "z_mm",
                      "a_deg", "b_deg", "c_deg", "override", "vel",
                      "svo_frame_idx", "zed_timestamp_ns"])
    print(f"[CSV] Logging → {csv_path}")

    # Prevent Windows from sleeping or locking the screen during recording
    prevent_sleep()

    frame_count = 0
    csv_rows = 0
    grab_fail_count = 0
    start = time.time()
    last_csv_t = 0.0
    csv_interval = 1.0 / CSV_LOG_HZ

    print(f"\n[RUN] Recording started (segment every {SEGMENT_MINUTES} min, "
          f"auto-stops when RSI disconnects).\n")
    print(f"{'Time':>7}  {'Frames':>6}  {'CSV':>5}  {'RSI':>7}  {'Seg':>3}  {'Robot XYZ':>30}")
    print("-" * 75)

    try:
        while True:
            grab_status = zed.grab(runtime)
            if grab_status != sl.ERROR_CODE.SUCCESS:
                grab_fail_count += 1
                if grab_fail_count % 100 == 1:
                    print(f"[WARN] zed.grab() failed: {grab_status} (total fails: {grab_fail_count})")
                continue

            frame_count += 1
            now = time.time()
            elapsed = now - start

            zed.retrieve_image(image, sl.VIEW.LEFT)
            frame = image.get_data()
            if frame.shape[2] == 4:
                frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
            cv2.putText(frame, f"REC {elapsed:.1f}s | F{frame_count}",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

            pose = rsi.get_latest()
            # Get ZED frame timestamp (nanoseconds since epoch)
            zed_ts_ns = zed.get_timestamp(sl.TIME_REFERENCE.IMAGE).get_nanoseconds()

            if pose:
                cv2.putText(frame, f"RSI: ({pose['x']:.1f}, {pose['y']:.1f}, {pose['z']:.1f})",
                            (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

                if now - last_csv_t >= csv_interval:
                    writer.writerow([f"{now:.6f}", pose["ipoc"],
                                     f"{pose['x']:.2f}", f"{pose['y']:.2f}", f"{pose['z']:.2f}",
                                     f"{pose['a']:.2f}", f"{pose['b']:.2f}", f"{pose['c']:.2f}",
                                     pose["override"], pose["vel"],
                                     frame_count, zed_ts_ns])
                    csv_rows += 1
                    last_csv_t = now
                    if csv_rows % CSV_FLUSH_EVERY == 0:
                        csv_file.flush()
            else:
                cv2.putText(frame, "RSI: waiting...", (10, 60),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

            cv2.imshow("RSI + ZED Recorder", frame)

            # ─── Segment rotation ─────────────────────────────────────
            if now - segment_start >= segment_duration:
                zed.disable_recording()
                segment_idx += 1
                svo_path, segment_start = start_segment(segment_idx)

            if frame_count % ZED_FPS == 0 and pose:
                print(f"{elapsed:6.1f}s  {frame_count:6d}  {csv_rows:5d}  "
                      f"{rsi.packet_count:7d}  {segment_idx:3d}  "
                      f"({pose['x']:8.1f}, {pose['y']:8.1f}, {pose['z']:8.1f})")

            # Auto-stop: RSI communication lost
            if not rsi.is_alive_rsi():
                print(f"\n[RSI] Communication lost — stopping recording.")
                break

            if cv2.waitKey(1) & 0xFF == ord("q"):
                print(f"\n[USER] Manual stop requested.")
                break

    except KeyboardInterrupt:
        pass

    finally:
        allow_sleep()
        rsi.stop()
        zed.disable_recording()
        cv2.destroyAllWindows()
        zed.close()
        csv_file.close()

        total = time.time() - start
        print(f"\n{'=' * 75}")
        print(f"[DONE] {total:.1f}s | {frame_count} frames | {csv_rows} CSV rows | "
              f"{rsi.packet_count} RSI packets | {segment_idx} segments | {grab_fail_count} grab fails")
        print(f"  SVO dir: {svo_dir}")
        print(f"  CSV:     {csv_path}")
        print(f"{'=' * 75}")


if __name__ == "__main__":
    main()
