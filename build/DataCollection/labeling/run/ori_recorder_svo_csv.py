"""
ori_recorder_svo_csv.py — RSI listener + ZED SVO2 recorder + CSV logger
Output: recorded_data/<ts>/recording_<ts>.svo2 + rsi_data/rsi_data_<ts>.csv
"""

import csv
import os
import socket
import threading
import time
import xml.etree.ElementTree as ET
from datetime import datetime

import cv2
import pyzed.sl as sl

# ─── Config ───────────────────────────────────────────────────────────────────

RSI_HOST = "0.0.0.0"
RSI_PORT = 59152

ZED_RESOLUTION = sl.RESOLUTION.HD1080
ZED_FPS = 30
ZED_DEPTH_MODE = sl.DEPTH_MODE.ULTRA
ZED_CODEC = sl.SVO_COMPRESSION_MODE.H264

CSV_LOG_HZ = 30
CSV_FLUSH_EVERY = 100

WORKSPACE_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..", ".."))
SVO_OUTPUT_DIR = os.path.join(WORKSPACE_ROOT, "recorded_data")
CSV_OUTPUT_DIR = os.path.join(WORKSPACE_ROOT, "rsi_data")

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

    def stop(self):
        self.running = False


# ─── Main ─────────────────────────────────────────────────────────────────────

def main():
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
    print("[PREVIEW] press 's' to start, 'q' to quit")
    while True:
        if zed.grab(runtime) == sl.ERROR_CODE.SUCCESS:
            zed.retrieve_image(image, sl.VIEW.LEFT)
            frame = image.get_data()
            if frame.shape[2] == 4:
                frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
            cv2.putText(frame, "PREVIEW — press 's' to start", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
            cv2.imshow("RSI + ZED Recorder", frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('s'):
            break
        elif key == ord('q'):
            rsi.stop()
            zed.close()
            cv2.destroyAllWindows()
            print("[EXIT] Cancelled by user.")
            return

    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    svo_dir = os.path.join(SVO_OUTPUT_DIR, ts)
    os.makedirs(svo_dir, exist_ok=True)
    os.makedirs(CSV_OUTPUT_DIR, exist_ok=True)

    svo_path = os.path.join(svo_dir, f"recording_{ts}.svo2")
    csv_path = os.path.join(CSV_OUTPUT_DIR, f"rsi_data_{ts}.csv")

    rec_p = sl.RecordingParameters()
    rec_p.compression_mode = ZED_CODEC
    rec_p.video_filename = svo_path
    err = zed.enable_recording(rec_p)
    if err != sl.ERROR_CODE.SUCCESS:
        zed.close()
        raise RuntimeError(f"ZED recording failed: {err}")
    print(f"[ZED] Recording → {svo_path}")

    csv_file = open(csv_path, "w", newline="")
    writer = csv.writer(csv_file)
    writer.writerow(["timestamp", "ipoc", "x_mm", "y_mm", "z_mm",
                      "a_deg", "b_deg", "c_deg", "override", "vel"])
    print(f"[CSV] Logging → {csv_path}")

    frame_count = 0
    csv_rows = 0
    start = time.time()
    last_csv_t = 0.0
    csv_interval = 1.0 / CSV_LOG_HZ

    print(f"\n[RUN] Recording started. Press 'q' to stop.\n")
    print(f"{'Time':>7}  {'Frames':>6}  {'CSV':>5}  {'RSI':>7}  {'Robot XYZ':>30}")
    print("-" * 70)

    try:
        while True:
            if zed.grab(runtime) != sl.ERROR_CODE.SUCCESS:
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
            if pose:
                cv2.putText(frame, f"RSI: ({pose['x']:.1f}, {pose['y']:.1f}, {pose['z']:.1f})",
                            (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

                if now - last_csv_t >= csv_interval:
                    writer.writerow([f"{now:.6f}", pose["ipoc"],
                                     f"{pose['x']:.2f}", f"{pose['y']:.2f}", f"{pose['z']:.2f}",
                                     f"{pose['a']:.2f}", f"{pose['b']:.2f}", f"{pose['c']:.2f}",
                                     pose["override"], pose["vel"]])
                    csv_rows += 1
                    last_csv_t = now
                    if csv_rows % CSV_FLUSH_EVERY == 0:
                        csv_file.flush()
            else:
                cv2.putText(frame, "RSI: waiting...", (10, 60),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

            cv2.imshow("RSI + ZED Recorder", frame)

            if frame_count % ZED_FPS == 0 and pose:
                print(f"{elapsed:6.1f}s  {frame_count:6d}  {csv_rows:5d}  "
                      f"{rsi.packet_count:7d}  "
                      f"({pose['x']:8.1f}, {pose['y']:8.1f}, {pose['z']:8.1f})")

            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

    except KeyboardInterrupt:
        pass

    finally:
        rsi.stop()
        zed.disable_recording()
        cv2.destroyAllWindows()
        zed.close()
        csv_file.close()

        total = time.time() - start
        print(f"\n{'=' * 70}")
        print(f"[DONE] {total:.1f}s | {frame_count} frames | {csv_rows} CSV rows | {rsi.packet_count} RSI packets")
        print(f"  SVO: {svo_path}")
        print(f"  CSV: {csv_path}")
        print(f"{'=' * 70}")


if __name__ == "__main__":
    main()
