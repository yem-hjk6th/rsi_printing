#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
RSI + ArUco synchronized capture

- Receives RSI UDP XML packets from KUKA controller
- Detects ArUco markers from camera stream continuously
- Writes synchronized robot pose + marker poses into one CSV row
- Uses laptop timestamp as reference and checks camera lag

Default behavior:
- Keep RSI alive with zero RKorr
- Record only when robot pose is stable for a short dwell time
- Record only when >= min_markers are visible
"""

import argparse
import csv
import socket
import threading
import time
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, TextIO, Tuple

import cv2
import cv2.aruco as aruco
import numpy as np

try:
    import pyzed.sl as sl
except Exception:
    sl = None

SCRIPT_DIR = Path(__file__).resolve().parent


# =============================
# Tunable defaults (edit here)
# =============================
DEFAULT_HOST = "0.0.0.0"
DEFAULT_PORT = 59152
DEFAULT_OUT = str(SCRIPT_DIR / "sync_robot_aruco.csv")

DEFAULT_CAMERA_SOURCE = 1
DEFAULT_USE_ZED = False
DEFAULT_ZED_RESOLUTION = "HD1080"
DEFAULT_ZED_FPS = 30
DEFAULT_ZED_DEPTH_MODE = "NONE"
DEFAULT_ZED_UNITS = "MILLIMETER"
DEFAULT_CAM_PREVIEW = True

DEFAULT_ARUCO_DICT = "DICT_6X6_250"
DEFAULT_MARKER_LENGTH_MM = 50.0
DEFAULT_TRACKED_MARKER_IDS = "0,1,2,3,4"
DEFAULT_MIN_MARKERS = 1

DEFAULT_MAX_CAM_LAG_MS = 120.0
DEFAULT_CAPTURE_INTERVAL_S = 2.6
DEFAULT_STABLE_TIME_S = 0.8
DEFAULT_STABLE_POS_MM = 0.3
DEFAULT_STABLE_ANG_DEG = 0.2


@dataclass
class RobotPose:
    x: float
    y: float
    z: float
    a: float
    b: float
    c: float


@dataclass
class MarkerPose:
    rvec: np.ndarray
    tvec: np.ndarray


class SharedArucoState:
    def __init__(self) -> None:
        self.lock = threading.Lock()
        self.timestamp_s: float = 0.0
        self.markers: Dict[int, MarkerPose] = {}
        self.frame_index: int = 0

    def update(self, ts: float, markers: Dict[int, MarkerPose], frame_index: int) -> None:
        with self.lock:
            self.timestamp_s = ts
            self.markers = markers
            self.frame_index = frame_index

    def snapshot(self) -> Tuple[float, Dict[int, MarkerPose], int]:
        with self.lock:
            return self.timestamp_s, dict(self.markers), self.frame_index


def parse_rsi_xml(data: bytes) -> Tuple[bool, str, Optional[RobotPose], int, int, int]:
    try:
        root = ET.fromstring(data)

        ipoc = "0"
        ipoc_elem = root.find("IPOC")
        if ipoc_elem is not None and ipoc_elem.text is not None:
            ipoc = ipoc_elem.text

        pose = None
        rist = root.find("RIst")
        if rist is not None:
            pose = RobotPose(
                x=float(rist.get("X", 0.0)),
                y=float(rist.get("Y", 0.0)),
                z=float(rist.get("Z", 0.0)),
                a=float(rist.get("A", 0.0)),
                b=float(rist.get("B", 0.0)),
                c=float(rist.get("C", 0.0)),
            )

        override = 0
        ov_elem = root.find("Override")
        if ov_elem is not None and ov_elem.text is not None:
            override = int(ov_elem.text)

        vel_act = 0
        vel_elem = root.find("Vel_Act")
        if vel_elem is not None and vel_elem.text is not None:
            vel_act = int(vel_elem.text)

        rpm_ext = 0
        rpm_elem = root.find("RPM_Ext")
        if rpm_elem is not None and rpm_elem.text is not None:
            rpm_ext = int(rpm_elem.text)

        return True, ipoc, pose, override, vel_act, rpm_ext
    except Exception:
        return False, "0", None, 0, 0, 0


def build_zero_reply(ipoc: str) -> str:
    return (
        f'<Sen Type="ImFree">'
        f'<RKorr X="0.0000" Y="0.0000" Z="0.0000" '
        f'A="0.0000" B="0.0000" C="0.0000" />'
        f"<IPOC>{ipoc}</IPOC>"
        f"</Sen>"
    )


def pose_delta(curr: RobotPose, prev: RobotPose) -> Tuple[float, float]:
    dp = np.linalg.norm(np.array([curr.x - prev.x, curr.y - prev.y, curr.z - prev.z], dtype=np.float64))
    da = np.linalg.norm(np.array([curr.a - prev.a, curr.b - prev.b, curr.c - prev.c], dtype=np.float64))
    return float(dp), float(da)


def parse_marker_ids(text: str) -> List[int]:
    if not text.strip():
        return []
    return [int(item.strip()) for item in text.split(",") if item.strip()]


def _estimate_pose(corners, marker_length_m: float, camera_matrix: np.ndarray, dist_coeffs: np.ndarray):
    if hasattr(aruco, "estimatePoseSingleMarkers"):
        rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, marker_length_m, camera_matrix, dist_coeffs)
        return rvecs, tvecs

    half = marker_length_m / 2.0
    obj_points = np.array(
        [
            [-half, half, 0.0],
            [half, half, 0.0],
            [half, -half, 0.0],
            [-half, -half, 0.0],
        ],
        dtype=np.float32,
    )

    rvecs = []
    tvecs = []
    for c in corners:
        img_points = c.reshape(4, 2).astype(np.float32)
        ok, rvec, tvec = cv2.solvePnP(
            obj_points,
            img_points,
            camera_matrix,
            dist_coeffs,
            flags=cv2.SOLVEPNP_IPPE_SQUARE,
        )
        if ok:
            rvecs.append(rvec.reshape(1, 3))
            tvecs.append(tvec.reshape(1, 3))

    if not rvecs:
        return None, None

    return np.vstack(rvecs)[:, None, :], np.vstack(tvecs)[:, None, :]


def _open_zed(resolution: str, fps: int, depth_mode: str, units: str):
    if sl is None:
        raise RuntimeError("pyzed.sl not available")
    zed = sl.Camera()
    init_params = sl.InitParameters()
    init_params.camera_resolution = getattr(sl.RESOLUTION, resolution)
    init_params.camera_fps = fps
    init_params.depth_mode = getattr(sl.DEPTH_MODE, depth_mode)
    init_params.coordinate_units = getattr(sl.UNIT, units)
    status = zed.open(init_params)
    if status != sl.ERROR_CODE.SUCCESS:
        raise RuntimeError(f"Failed to open ZED camera: {status}")
    return zed


def aruco_worker(
    shared: SharedArucoState,
    stop_event: threading.Event,
    use_zed: bool,
    source: int,
    dict_name: str,
    marker_length_m: float,
    camera_matrix: np.ndarray,
    dist_coeffs: np.ndarray,
    show: bool,
    zed_resolution: str,
    zed_fps: int,
    zed_depth_mode: str,
    zed_units: str,
) -> None:
    aruco_dict = aruco.getPredefinedDictionary(getattr(aruco, dict_name))

    cap = None
    zed = None
    zed_image = None
    zed_runtime = None

    if use_zed:
        zed = _open_zed(zed_resolution, zed_fps, zed_depth_mode, zed_units)
        zed_image = sl.Mat()
        zed_runtime = sl.RuntimeParameters()
    else:
        cap = cv2.VideoCapture(source)
        if not cap.isOpened():
            raise RuntimeError(f"Failed to open camera source {source}")

    frame_idx = 0
    try:
        while not stop_event.is_set():
            if zed is not None:
                if zed.grab(zed_runtime) != sl.ERROR_CODE.SUCCESS:
                    continue
                zed.retrieve_image(zed_image, sl.VIEW.LEFT)
                frame = zed_image.get_data()
                if frame.shape[2] == 4:
                    frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
            else:
                ok, frame = cap.read()
                if not ok:
                    continue

            frame_idx += 1
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            if hasattr(aruco, "ArucoDetector"):
                detector = aruco.ArucoDetector(aruco_dict)
                corners, ids, _ = detector.detectMarkers(gray)
            else:
                corners, ids, _ = aruco.detectMarkers(gray, aruco_dict)

            markers: Dict[int, MarkerPose] = {}
            if ids is not None and len(ids) > 0:
                rvecs, tvecs = _estimate_pose(corners, marker_length_m, camera_matrix, dist_coeffs)
                if rvecs is not None:
                    for marker_id, rvec, tvec in zip(ids.flatten(), rvecs, tvecs):
                        markers[int(marker_id)] = MarkerPose(rvec=rvec.reshape(3), tvec=tvec.reshape(3))

                if show:
                    aruco.drawDetectedMarkers(frame, corners, ids)
                    if rvecs is not None:
                        for rvec, tvec in zip(rvecs, tvecs):
                            cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvec, tvec, marker_length_m * 0.5)

            ts = time.time()
            shared.update(ts, markers, frame_idx)

            if show:
                cv2.putText(
                    frame,
                    f"markers={len(markers)} frame={frame_idx}",
                    (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 255, 0),
                    2,
                )
                cv2.imshow("ArUco Sync Capture", frame)
                key = cv2.waitKey(1) & 0xFF
                if key == ord("q"):
                    stop_event.set()
                    break
    finally:
        if cap is not None:
            cap.release()
        if zed is not None:
            zed.close()
        if show:
            cv2.destroyAllWindows()


def build_csv_header(marker_ids: List[int]) -> List[str]:
    header = [
        "pc_timestamp_s",
        "ipoc",
        "override",
        "vel_act",
        "rpm_ext",
        "robot_x_mm",
        "robot_y_mm",
        "robot_z_mm",
        "robot_a_deg",
        "robot_b_deg",
        "robot_c_deg",
        "aruco_timestamp_s",
        "camera_lag_ms",
        "visible_count",
        "visible_ids",
    ]
    for marker_id in marker_ids:
        header.extend(
            [
                f"m{marker_id}_rvec_x",
                f"m{marker_id}_rvec_y",
                f"m{marker_id}_rvec_z",
                f"m{marker_id}_tvec_x_m",
                f"m{marker_id}_tvec_y_m",
                f"m{marker_id}_tvec_z_m",
            ]
        )
    return header


def ensure_csv(path: Path, marker_ids: List[int]) -> Tuple[TextIO, csv.writer]:
    path.parent.mkdir(parents=True, exist_ok=True)
    is_new = not path.exists()
    fp = open(path, "a", encoding="utf-8", newline="")
    writer = csv.writer(fp)
    if is_new:
        writer.writerow(build_csv_header(marker_ids))
        fp.flush()
    return fp, writer


def resolve_in_script_dir(path_text: str) -> Path:
    path = Path(path_text)
    if path.is_absolute():
        return path
    return SCRIPT_DIR / path


def main() -> None:
    parser = argparse.ArgumentParser(description="Synchronized RSI + ArUco capture")

    parser.add_argument("--host", type=str, default=DEFAULT_HOST)
    parser.add_argument("--port", type=int, default=DEFAULT_PORT)

    parser.add_argument("--out", type=str, default=DEFAULT_OUT)

    parser.add_argument("--camera-source", type=int, default=DEFAULT_CAMERA_SOURCE)
    parser.add_argument("--use-zed", action="store_true", default=DEFAULT_USE_ZED)
    parser.add_argument("--zed-resolution", type=str, default=DEFAULT_ZED_RESOLUTION)
    parser.add_argument("--zed-fps", type=int, default=DEFAULT_ZED_FPS)
    parser.add_argument("--zed-depth-mode", type=str, default=DEFAULT_ZED_DEPTH_MODE)
    parser.add_argument("--zed-units", type=str, default=DEFAULT_ZED_UNITS)

    parser.add_argument("--dict", type=str, default=DEFAULT_ARUCO_DICT)
    parser.add_argument("--marker-length-mm", type=float, default=DEFAULT_MARKER_LENGTH_MM)
    parser.add_argument("--camera-matrix", type=str, default="")

    parser.add_argument("--tracked-marker-ids", type=str, default=DEFAULT_TRACKED_MARKER_IDS)
    parser.add_argument("--min-markers", type=int, default=DEFAULT_MIN_MARKERS)

    parser.add_argument("--max-cam-lag-ms", type=float, default=DEFAULT_MAX_CAM_LAG_MS)
    parser.add_argument("--capture-interval-s", type=float, default=DEFAULT_CAPTURE_INTERVAL_S)
    parser.add_argument("--stable-time-s", type=float, default=DEFAULT_STABLE_TIME_S)
    parser.add_argument("--stable-pos-mm", type=float, default=DEFAULT_STABLE_POS_MM)
    parser.add_argument("--stable-ang-deg", type=float, default=DEFAULT_STABLE_ANG_DEG)

    parser.add_argument("--cam-preview", dest="cam_preview", action="store_true", help="show camera preview window")
    parser.add_argument("--no-cam-preview", dest="cam_preview", action="store_false", help="disable camera preview window")
    parser.add_argument("--show", dest="cam_preview", action="store_true", help=argparse.SUPPRESS)
    parser.set_defaults(cam_preview=DEFAULT_CAM_PREVIEW)
    args = parser.parse_args()

    if args.camera_matrix:
        data = np.load(args.camera_matrix)
        camera_matrix = data["camera_matrix"]
        dist_coeffs = data["dist_coeffs"]
    else:
        camera_matrix = np.array(
            [[951.72229004, 0.0, 638.30792236], [0.0, 951.72229004, 352.57427979], [0.0, 0.0, 1.0]],
            dtype=np.float64,
        )
        dist_coeffs = np.zeros((5,), dtype=np.float64)

    tracked_marker_ids = parse_marker_ids(args.tracked_marker_ids)
    if not tracked_marker_ids:
        raise RuntimeError("tracked-marker-ids cannot be empty")

    out_path = resolve_in_script_dir(args.out)
    csv_fp, writer = ensure_csv(out_path, tracked_marker_ids)

    shared = SharedArucoState()
    stop_event = threading.Event()

    worker = threading.Thread(
        target=aruco_worker,
        kwargs=dict(
            shared=shared,
            stop_event=stop_event,
            use_zed=args.use_zed,
            source=args.camera_source,
            dict_name=args.dict,
            marker_length_m=args.marker_length_mm / 1000.0,
            camera_matrix=camera_matrix,
            dist_coeffs=dist_coeffs,
            show=args.cam_preview,
            zed_resolution=args.zed_resolution,
            zed_fps=args.zed_fps,
            zed_depth_mode=args.zed_depth_mode,
            zed_units=args.zed_units,
        ),
        daemon=True,
    )
    worker.start()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.settimeout(1.0)
    sock.bind((args.host, args.port))

    print("=" * 100)
    print(f"RSI sync capture listening on {args.host}:{args.port}")
    print(f"CSV: {out_path.resolve()}")
    print(f"Camera preview: {'ON' if args.cam_preview else 'OFF'}")
    print(f"Marker gate: >= {args.min_markers} from set {tracked_marker_ids}")
    print(
        f"Tunables: dict={args.dict} marker_len={args.marker_length_mm:.1f}mm "
        f"stable=({args.stable_time_s:.2f}s,{args.stable_pos_mm:.3f}mm,{args.stable_ang_deg:.3f}deg) "
        f"lag<={args.max_cam_lag_ms:.1f}ms interval>={args.capture_interval_s:.2f}s"
    )
    print("Press Ctrl+C to stop")
    print("=" * 100)

    is_connected = False
    stable_since: Optional[float] = None
    prev_pose: Optional[RobotPose] = None
    last_capture_time = 0.0
    packet_count = 0
    saved_count = 0
    ipoc = "0"

    try:
        while True:
            try:
                data, addr = sock.recvfrom(4096)
                packet_count += 1
                now = time.time()

                ok, ipoc, pose, override, vel_act, rpm_ext = parse_rsi_xml(data)

                if not is_connected:
                    print(f"[CONNECTED] {addr}")
                    is_connected = True

                reply = build_zero_reply(ipoc)
                sock.sendto(reply.encode(), addr)

                if not ok or pose is None:
                    continue

                if prev_pose is None:
                    prev_pose = pose
                    stable_since = now
                    continue

                dp_mm, da_deg = pose_delta(pose, prev_pose)
                prev_pose = pose

                if dp_mm <= args.stable_pos_mm and da_deg <= args.stable_ang_deg:
                    if stable_since is None:
                        stable_since = now
                else:
                    stable_since = None

                is_stable = stable_since is not None and (now - stable_since) >= args.stable_time_s
                is_interval_ok = (now - last_capture_time) >= args.capture_interval_s

                cam_ts, marker_map, frame_idx = shared.snapshot()
                lag_ms = (now - cam_ts) * 1000.0 if cam_ts > 0 else 999999.0
                raw_count = len(marker_map)
                visible_ids = sorted([mid for mid in marker_map.keys() if mid in tracked_marker_ids])
                visible_count = len(visible_ids)

                gate_markers = visible_count >= args.min_markers
                gate_lag = lag_ms <= args.max_cam_lag_ms

                if is_stable and is_interval_ok and gate_markers and gate_lag:
                    row = [
                        now,
                        ipoc,
                        override,
                        vel_act,
                        rpm_ext,
                        pose.x,
                        pose.y,
                        pose.z,
                        pose.a,
                        pose.b,
                        pose.c,
                        cam_ts,
                        lag_ms,
                        visible_count,
                        "|".join(map(str, visible_ids)),
                    ]

                    for marker_id in tracked_marker_ids:
                        mp = marker_map.get(marker_id)
                        if mp is None:
                            row.extend(["", "", "", "", "", ""])
                        else:
                            row.extend(
                                [
                                    float(mp.rvec[0]),
                                    float(mp.rvec[1]),
                                    float(mp.rvec[2]),
                                    float(mp.tvec[0]),
                                    float(mp.tvec[1]),
                                    float(mp.tvec[2]),
                                ]
                            )

                    writer.writerow(row)
                    csv_fp.flush()
                    saved_count += 1
                    last_capture_time = now
                    print(
                        f"[SAVE #{saved_count}] ipoc={ipoc} stable=Y frame={frame_idx} "
                        f"visible={visible_ids} lag={lag_ms:.1f}ms pos=({pose.x:.1f},{pose.y:.1f},{pose.z:.1f})"
                    )

                if packet_count % 500 == 0:
                    print(
                        f"[STATUS] packets={packet_count} saves={saved_count} stable={'Y' if is_stable else 'N'} "
                        f"markers={visible_count} raw={raw_count} lag={lag_ms:.1f}ms dp={dp_mm:.3f}mm da={da_deg:.3f}deg"
                    )

            except socket.timeout:
                continue

    except KeyboardInterrupt:
        print("\n[STOP] Keyboard interrupt")
    finally:
        stop_event.set()
        worker.join(timeout=2.0)
        sock.close()
        csv_fp.close()
        print(f"[SUMMARY] packets={packet_count} saves={saved_count}")


if __name__ == "__main__":
    main()
