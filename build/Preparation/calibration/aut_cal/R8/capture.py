#!/usr/bin/env python3
"""
Minimal RSI + ArUco synchronized capture (ZED SDK).
Listens for KUKA RSI packets, detects ArUco from ZED left camera, writes CSV.

Usage:  python capture.py
        python capture.py --port 59152 --marker 0
"""

import argparse, csv, socket, threading, time
import xml.etree.ElementTree as ET
from pathlib import Path
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
MARKER_LEN_MM  = 100.0
MARKER_ID      = 1            # aruco_gen2.py: 75mm=ID0, 100mm=ID1, 125mm=ID2, 150mm=ID3, 200mm=ID4
DICT_NAME      = "DICT_6X6_250"
STABLE_S       = 0.8
CAPTURE_GAP_S  = 2.6

# ── ArUco thread ─────────────────────────────────────────────────────────────
class ArucoState:
    def __init__(self):
        self.lock = threading.Lock()
        self.ts = 0.0
        self.rvec = None   # (3,)
        self.tvec = None   # (3,)

    def set(self, ts, rvec, tvec):
        with self.lock:
            self.ts, self.rvec, self.tvec = ts, rvec, tvec

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


def cam_loop(state, stop, mid, mlen, show):
    # ── Open ZED ──
    zed = sl.Camera()
    init = sl.InitParameters()
    init.camera_resolution = sl.RESOLUTION.HD1080       # changed from HD720
    init.camera_fps = 30
    init.depth_mode = sl.DEPTH_MODE.NONE
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

    aruco_dict = aruco.getPredefinedDictionary(getattr(aruco, DICT_NAME))

    # ── Sub-pixel refinement parameters ──
    if hasattr(aruco, 'DetectorParameters'):
        det_params = aruco.DetectorParameters()
    else:
        det_params = aruco.DetectorParameters_create()
    det_params.cornerRefinementMethod  = aruco.CORNER_REFINE_SUBPIX
    det_params.cornerRefinementWinSize = 5
    det_params.cornerRefinementMaxIterations = 50
    det_params.cornerRefinementMinAccuracy   = 0.01

    zed_image = sl.Mat()
    runtime = sl.RuntimeParameters()

    try:
        while not stop.is_set():
            if zed.grab(runtime) != sl.ERROR_CODE.SUCCESS:
                continue
            zed.retrieve_image(zed_image, sl.VIEW.LEFT)
            frame = zed_image.get_data()
            if frame.shape[2] == 4:
                frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            if hasattr(aruco, "ArucoDetector"):
                detector = aruco.ArucoDetector(aruco_dict, det_params)
                corners, ids, _ = detector.detectMarkers(gray)
            else:
                corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=det_params)

            if ids is not None and mid in ids.flatten():
                idx = list(ids.flatten()).index(mid)
                rvs, tvs = estimate_pose(corners, mlen, K, D)
                if rvs is not None:
                    state.set(time.time(), rvs[idx].ravel(), tvs[idx].ravel())
                    if show:
                        aruco.drawDetectedMarkers(frame, corners, ids)
                        cv2.drawFrameAxes(frame, K, D, rvs[idx], tvs[idx], mlen*0.5)
            if show:
                cv2.imshow("capture", frame)
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

    out = DIR / "sync_robot_aruco.csv"
    # auto-increment: _1.csv, _2.csv, ...
    if out.exists():
        i = 1
        while (DIR / f"sync_robot_aruco_{i}.csv").exists():
            i += 1
        out = DIR / f"sync_robot_aruco_{i}.csv"
    hdr = ["pc_ts","ipoc","robot_x_mm","robot_y_mm","robot_z_mm",
           "robot_a_deg","robot_b_deg","robot_c_deg",
           "cam_ts","rvec_x","rvec_y","rvec_z","tvec_x_m","tvec_y_m","tvec_z_m"]
    fp = open(out, "w", newline="", encoding="utf-8")
    w = csv.writer(fp)
    w.writerow(hdr)

    state = ArucoState()
    stop = threading.Event()
    t = threading.Thread(target=cam_loop, daemon=True,
                         args=(state, stop, a.marker,
                               MARKER_LEN_MM/1000, not a.no_show))
    t.start()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.settimeout(1.0)
    sock.bind((HOST, a.port))

    print(f"Listening {HOST}:{a.port}  marker={a.marker}")
    print(f"CSV: {out}")

    prev = None; stable_t = None; last_cap = 0.0; n_saved = 0; n_pkt = 0

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
                if dp < 0.3 and da < 0.2:
                    if stable_t is None: stable_t = now
                else:
                    stable_t = None
            prev = pose

            if stable_t is None or (now - stable_t) < STABLE_S:
                continue
            if (now - last_cap) < CAPTURE_GAP_S:
                continue

            cam_ts, rv, tv = state.get()
            if rv is None or (now - cam_ts) > 0.12:
                continue

            w.writerow([now, ipoc_str, *pose, cam_ts, *rv, *tv])
            fp.flush()
            n_saved += 1
            last_cap = now
            print(f"[{n_saved:3d}] pos=({pose[0]:.0f},{pose[1]:.0f},{pose[2]:.0f}) "
                  f"ABC=({pose[3]:.1f},{pose[4]:.1f},{pose[5]:.1f})")

            if n_pkt % 500 == 0:
                print(f"  pkts={n_pkt} saved={n_saved}")

    except KeyboardInterrupt:
        print("\nStopped")
    finally:
        stop.set(); t.join(2); sock.close(); fp.close()
        print(f"Total: {n_saved} samples saved to {out}")

if __name__ == "__main__":
    main()
