#!/usr/bin/env python3
"""
Minimal RSI + ArUco synchronized capture (ZED SDK).
Listens for KUKA RSI packets, detects ArUco from ZED left camera, writes CSV.

Usage:  python capture.py
        python capture.py --port 59152 --marker 1
"""

import argparse, csv, os, socket, threading, time
import xml.etree.ElementTree as ET
from pathlib import Path

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
import cv2.aruco as aruco
import numpy as np
import pyzed.sl as sl

DIR = Path(__file__).resolve().parent

# ── Defaults ─────────────────────────────────────────────────────────────────
HOST           = "0.0.0.0"
PORT           = 59152
# Marker physical size in mm.  Larger marker = more pixels on target = better
# corner localisation.  Recommended: ≥100 mm for distances around 0.5-1.5 m.
# NOTE: use ACTUAL measured size, not nominal. Nominal 125mm = actual 110mm.
MARKER_LEN_MM  = 88        # change after caliper calibration of printed marker
MARKER_ID      = 1            # aruco_gen2.py: 75mm=ID0, 100mm=ID1, 125mm=ID2, 150mm=ID3, 200mm=ID4
DICT_NAME      = "DICT_6X6_250"
STABLE_S       = 2.0          # how long robot must be motionless (s)
CAPTURE_GAP_S  = 3.0          # min gap between captures (s)

# ZED camera settings
ZED_RESOLUTION = "HD2K"       # 2208×1242, highest res, best ArUco corner accuracy
ZED_FPS        = 15            # HD2K max = 15fps (sufficient — robot waits 3s per pose)
ZED_DEPTH_MODE = "NONE"       # no depth needed for calibration capture

# Sub-pixel corner refinement (improves ArUco corner localisation)
SUBPIX_WIN_SIZE   = 5
SUBPIX_MAX_ITER   = 50
SUBPIX_MIN_ACC    = 0.01

# Stability gate — robot must be stationary before capture
STABLE_POS_MM  = 0.3          # max position delta between RSI packets (mm)
STABLE_ANG_DEG = 0.2          # max angle delta between RSI packets (deg)
CAM_LAG_MAX_S  = 0.12         # max allowed camera-to-RSI time lag (s)

# New-pose gate — robot must have moved enough since last capture
MIN_DISP_MM    = 3.0          # min position change from last capture (mm)
MIN_ROT_DEG    = 2.0          # min orientation change from last capture (deg)

# ── ArUco thread ─────────────────────────────────────────────────────────────
# Multi-frame averaging: collect N consecutive detections and return the median
# to reduce single-frame corner noise.  Only useful when robot is stationary.
AVG_FRAMES = 5

class ArucoState:
    def __init__(self):
        self.lock = threading.Lock()
        self.ts = 0.0
        self.rvec = None   # (3,)
        self.tvec = None   # (3,)
        self._rv_buf = []  # ring buffer for averaging
        self._tv_buf = []

    def push(self, ts, rvec, tvec):
        """Push a single-frame detection. Median of last AVG_FRAMES is used."""
        with self.lock:
            self._rv_buf.append(rvec)
            self._tv_buf.append(tvec)
            if len(self._rv_buf) > AVG_FRAMES:
                self._rv_buf.pop(0)
                self._tv_buf.pop(0)
            if len(self._rv_buf) >= AVG_FRAMES:
                self.rvec = np.median(self._rv_buf, axis=0)
                self.tvec = np.median(self._tv_buf, axis=0)
                self.ts = ts

    def clear_buf(self):
        with self.lock:
            self._rv_buf.clear()
            self._tv_buf.clear()

    def get(self):
        with self.lock:
            return self.ts, (self.rvec.copy() if self.rvec is not None else None),\
                   (self.tvec.copy() if self.tvec is not None else None)


def estimate_pose(corners, mlen, K, D):
    if hasattr(aruco, "estimatePoseSingleMarkers"):
        rv, tv, _ = aruco.estimatePoseSingleMarkers(corners, mlen, K, D)
        return rv, tv
    half = mlen / 2.0
    obj = np.array([[-half,half,0],[half,half,0],[half,-half,0],[-half,-half,0]], dtype=np.float32)
    rvs, tvs = [], []
    for c in corners:
        ok, rv, tv = cv2.solvePnP(obj, c.reshape(4,2).astype(np.float32), K, D,
                                   flags=cv2.SOLVEPNP_IPPE_SQUARE)
        if ok:
            rvs.append(rv.reshape(1,3)); tvs.append(tv.reshape(1,3))
    if not rvs:
        return None, None
    return np.vstack(rvs)[:,None,:], np.vstack(tvs)[:,None,:]


def cam_loop(state, stop, mid, mlen, show, intrinsics_out):
    # ── Open ZED ──
    zed = sl.Camera()
    init = sl.InitParameters()
    init.camera_resolution = getattr(sl.RESOLUTION, ZED_RESOLUTION)
    init.camera_fps = ZED_FPS
    init.depth_mode = getattr(sl.DEPTH_MODE, ZED_DEPTH_MODE)
    status = zed.open(init)
    if status != sl.ERROR_CODE.SUCCESS:
        print(f"[ERROR] ZED open failed: {status}"); return

    # ── Read intrinsics from SDK ──
    calib = zed.get_camera_information().camera_configuration.calibration_parameters
    K = np.array([[calib.left_cam.fx, 0, calib.left_cam.cx],
                  [0, calib.left_cam.fy, calib.left_cam.cy],
                  [0, 0, 1]], dtype=np.float64)
    d = calib.left_cam.disto
    D = np.array([d[0], d[1], d[2], d[3], d[4]], dtype=np.float64)
    print(f"[ZED] fx={calib.left_cam.fx:.1f} fy={calib.left_cam.fy:.1f} "
          f"cx={calib.left_cam.cx:.1f} cy={calib.left_cam.cy:.1f}")
    print(f"[ZED] resolution={ZED_RESOLUTION} fps={ZED_FPS}")

    # ── Save intrinsics alongside CSV ──
    if intrinsics_out is not None:
        with open(intrinsics_out, "w", encoding="utf-8") as kf:
            kf.write(f"# ZED {ZED_RESOLUTION} left camera intrinsics\n")
            kf.write(f"# fx fy cx cy\n")
            kf.write(f"{calib.left_cam.fx:.6f} {calib.left_cam.fy:.6f} "
                     f"{calib.left_cam.cx:.6f} {calib.left_cam.cy:.6f}\n")
            kf.write(f"# distortion k1 k2 p1 p2 k3\n")
            kf.write(" ".join(f"{v:.8f}" for v in D) + "\n")
        print(f"[ZED] intrinsics saved to {intrinsics_out}")

    aruco_dict = aruco.getPredefinedDictionary(getattr(aruco, DICT_NAME))

    # ── Sub-pixel refinement parameters ──
    if hasattr(aruco, 'DetectorParameters'):
        det_params = aruco.DetectorParameters()
    else:
        det_params = aruco.DetectorParameters_create()
    det_params.cornerRefinementMethod  = aruco.CORNER_REFINE_SUBPIX
    det_params.cornerRefinementWinSize = SUBPIX_WIN_SIZE
    det_params.cornerRefinementMaxIterations = SUBPIX_MAX_ITER
    det_params.cornerRefinementMinAccuracy   = SUBPIX_MIN_ACC

    zed_left = sl.Mat()
    zed_right = sl.Mat()
    runtime = sl.RuntimeParameters()

    try:
        while not stop.is_set():
            if zed.grab(runtime) != sl.ERROR_CODE.SUCCESS:
                continue
            zed.retrieve_image(zed_left, sl.VIEW.LEFT)
            frame = zed_left.get_data()
            if frame.shape[2] == 4:
                frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            if hasattr(aruco, "ArucoDetector"):
                detector = aruco.ArucoDetector(aruco_dict, det_params)
                corners, ids, _ = detector.detectMarkers(gray)
            else:
                corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=det_params)

            detected = False
            if ids is not None and mid in ids.flatten():
                idx = list(ids.flatten()).index(mid)
                rvs, tvs = estimate_pose(corners, mlen, K, D)
                if rvs is not None:
                    state.push(time.time(), rvs[idx].ravel(), tvs[idx].ravel())
                    detected = True
                    if show:
                        aruco.drawDetectedMarkers(frame, corners, ids)
                        cv2.drawFrameAxes(frame, K, D, rvs[idx], tvs[idx], mlen*0.5)

            if show:
                # ── Stereo side-by-side preview ──
                zed.retrieve_image(zed_right, sl.VIEW.RIGHT)
                rframe = zed_right.get_data()
                if rframe.shape[2] == 4:
                    rframe = cv2.cvtColor(rframe, cv2.COLOR_BGRA2BGR)
                # status bar on left image
                status_text = f"ID{mid} {'DETECTED' if detected else 'not found'}"
                cv2.putText(frame, status_text, (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                            (0, 255, 0) if detected else (0, 0, 255), 2)
                # resize both to half width for side-by-side
                h, w = frame.shape[:2]
                hw = w // 2
                hh = h // 2
                left_small = cv2.resize(frame, (hw, hh))
                right_small = cv2.resize(rframe, (hw, hh))
                cv2.putText(left_small, "LEFT", (10, 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                cv2.putText(right_small, "RIGHT", (10, 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                stereo = np.hstack([left_small, right_small])
                cv2.imshow("capture [L|R]", stereo)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    stop.set(); break
    finally:
        zed.close()
        if show: cv2.destroyAllWindows()

# ── RSI helpers ──────────────────────────────────────────────────────────────
def parse_rsi(data):
    root = ET.fromstring(data)
    ipoc = root.findtext("IPOC", "0")
    r = root.find("RIst")
    if r is None: return ipoc, None
    pose = [float(r.get(k,0)) for k in "XYZABC"]
    return ipoc, pose   # [x,y,z,a,b,c]

def reply(ipoc):
    return (f'<Sen Type="ImFree">'
            f'<RKorr X="0" Y="0" Z="0" A="0" B="0" C="0" />'
            f'<IPOC>{ipoc}</IPOC></Sen>')

# ── Main ─────────────────────────────────────────────────────────────────────
def main():
    p = argparse.ArgumentParser()
    p.add_argument("--port",   type=int,   default=PORT)
    p.add_argument("--marker", type=int,   default=MARKER_ID)
    p.add_argument("--no-show", action="store_true")
    a = p.parse_args()

    # ── Output directory: res/<timestamp>/ ──
    ts_str = time.strftime("%Y%m%d_%H%M%S")
    out_dir = DIR / "res" / ts_str
    out_dir.mkdir(parents=True, exist_ok=True)
    out = out_dir / "sync_robot_aruco.csv"
    hdr = ["pc_ts","ipoc","robot_x_mm","robot_y_mm","robot_z_mm",
           "robot_a_deg","robot_b_deg","robot_c_deg",
           "cam_ts","rvec_x","rvec_y","rvec_z","tvec_x_m","tvec_y_m","tvec_z_m"]
    fp = open(out, "w", newline="", encoding="utf-8")
    w = csv.writer(fp)
    w.writerow(hdr)

    # save intrinsics next to the CSV
    k_path = out_dir / "sync_robot_aruco.K.txt"

    state = ArucoState()
    stop = threading.Event()
    t = threading.Thread(target=cam_loop, daemon=True,
                         args=(state, stop, a.marker,
                               MARKER_LEN_MM/1000, not a.no_show, k_path))
    t.start()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.settimeout(1.0)
    sock.bind((HOST, a.port))

    print(f"Listening {HOST}:{a.port}  marker={a.marker}")
    print(f"CSV: {out}")

    prev = None; stable_t = None; last_cap = 0.0; n_saved = 0; n_pkt = 0
    last_cap_pose = None   # pose at last capture for displacement gate

    try:
        while not stop.is_set():
            try:
                data, addr = sock.recvfrom(4096)
            except socket.timeout:
                continue
            n_pkt += 1
            now = time.time()
            ipoc_str, pose = parse_rsi(data)
            sock.sendto(reply(ipoc_str).encode(), addr)
            if pose is None: continue

            # stability check
            if prev is not None:
                dp = sum((pose[i]-prev[i])**2 for i in range(3))**0.5
                da = sum((pose[i]-prev[i])**2 for i in range(3,6))**0.5
                if dp < STABLE_POS_MM and da < STABLE_ANG_DEG:
                    if stable_t is None: stable_t = now
                else:
                    stable_t = None
            prev = pose

            if stable_t is None or (now - stable_t) < STABLE_S:
                continue
            if (now - last_cap) < CAPTURE_GAP_S:
                continue

            # new-pose gate: robot must have moved since last capture
            if last_cap_pose is not None:
                dp_cap = sum((pose[i]-last_cap_pose[i])**2 for i in range(3))**0.5
                da_cap = sum((pose[i]-last_cap_pose[i])**2 for i in range(3,6))**0.5
                if dp_cap < MIN_DISP_MM and da_cap < MIN_ROT_DEG:
                    continue   # same pose as last capture, skip

            cam_ts, rv, tv = state.get()
            if rv is None or (now - cam_ts) > CAM_LAG_MAX_S:
                continue

            w.writerow([now, ipoc_str, *pose, cam_ts, *rv, *tv])
            fp.flush()
            n_saved += 1
            last_cap = now
            last_cap_pose = list(pose)
            state.clear_buf()  # reset averaging buffer for next pose
            print(f"[{n_saved:3d}] pos=({pose[0]:.0f},{pose[1]:.0f},{pose[2]:.0f}) "
                  f"ABC=({pose[3]:.1f},{pose[4]:.1f},{pose[5]:.1f}) "
                  f"tvec=({tv[0]:.3f},{tv[1]:.3f},{tv[2]:.3f})m")

            if n_pkt % 500 == 0:
                print(f"  pkts={n_pkt} saved={n_saved}")

    except KeyboardInterrupt:
        print("\nStopped")
    finally:
        stop.set(); t.join(2); sock.close(); fp.close()
        print(f"Total: {n_saved} samples saved to {out}")

if __name__ == "__main__":
    main()
