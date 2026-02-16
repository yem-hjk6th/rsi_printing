import argparse
import csv
import time
from pathlib import Path

import cv2
import cv2.aruco as aruco
import numpy as np

try:
    import pyzed.sl as sl
except Exception:
    sl = None

# ZED camera settings (match ArUco_rec defaults)
USE_ZED = True
ZED_RESOLUTION = "HD1080"
ZED_FPS = 30
ZED_ENABLE_DEPTH = False
ZED_DEPTH_MODE = "QUALITY"
ZED_COORDINATE_UNITS = "MILLIMETER"


def _ensure_writer(csv_path: Path) -> csv.writer:
    csv_path.parent.mkdir(parents=True, exist_ok=True)
    is_new = not csv_path.exists()
    f = open(csv_path, "a", encoding="utf-8", newline="")
    writer = csv.writer(f)
    if is_new:
        writer.writerow(
            [
                "timestamp_s",
                "marker_id",
                "rvec_x",
                "rvec_y",
                "rvec_z",
                "tvec_x",
                "tvec_y",
                "tvec_z",
            ]
        )
    return writer


def _estimate_pose(
    corners,
    marker_length_m: float,
    camera_matrix: np.ndarray,
    dist_coeffs: np.ndarray,
):
    if hasattr(aruco, "estimatePoseSingleMarkers"):
        rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
            corners, marker_length_m, camera_matrix, dist_coeffs
        )
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
            obj_points, img_points, camera_matrix, dist_coeffs, flags=cv2.SOLVEPNP_IPPE_SQUARE
        )
        if ok:
            rvecs.append(rvec.reshape(1, 3))
            tvecs.append(tvec.reshape(1, 3))
    if not rvecs:
        return None, None
    return np.vstack(rvecs)[:, None, :], np.vstack(tvecs)[:, None, :]


def _get_zed_resolution(name: str):
    return getattr(sl.RESOLUTION, name)


def _get_zed_depth_mode(name: str):
    return getattr(sl.DEPTH_MODE, name)


def _get_zed_units(name: str):
    return getattr(sl.UNIT, name)


def _open_zed_camera():
    if sl is None:
        raise RuntimeError("pyzed.sl not available")
    zed = sl.Camera()
    init_params = sl.InitParameters()
    init_params.camera_resolution = _get_zed_resolution(ZED_RESOLUTION)
    init_params.camera_fps = ZED_FPS
    init_params.depth_mode = _get_zed_depth_mode(ZED_DEPTH_MODE) if ZED_ENABLE_DEPTH else sl.DEPTH_MODE.NONE
    init_params.coordinate_units = _get_zed_units(ZED_COORDINATE_UNITS)
    if zed.open(init_params) != sl.ERROR_CODE.SUCCESS:
        raise RuntimeError("Failed to open ZED camera")
    return zed


def main() -> None:
    parser = argparse.ArgumentParser(description="Manual ArUco capture (press s to save)")
    parser.add_argument("--source", type=int, default=0, help="Camera index")
    parser.add_argument("--dict", type=str, default="DICT_6X6_250", help="ArUco dictionary")
    parser.add_argument("--marker-length-mm", type=float, default=50.0, help="Marker side length (mm)")
    parser.add_argument("--out", type=str, default="manual_aruco_poses.csv", help="Output CSV path")
    parser.add_argument("--use-zed", action="store_true", help="Use ZED camera instead of OpenCV source")
    parser.add_argument(
        "--camera-matrix",
        type=str,
        default="",
        help="Optional npz file with camera_matrix/dist_coeffs",
    )
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
        dist_coeffs = np.array([0.0, 0.0, 0.0, 0.0, 0.0], dtype=np.float64)

    aruco_dict = aruco.getPredefinedDictionary(getattr(aruco, args.dict))
    marker_length_m = args.marker_length_mm / 1000.0

    cap = None
    zed = None
    zed_image = None
    zed_runtime = None
    use_zed = args.use_zed or USE_ZED
    if use_zed:
        zed = _open_zed_camera()
        zed_image = sl.Mat()
        zed_runtime = sl.RuntimeParameters()
    else:
        cap = cv2.VideoCapture(args.source)
        if not cap.isOpened():
            raise RuntimeError("Failed to open camera source")

    csv_path = Path(args.out)
    writer = _ensure_writer(csv_path)
    save_count = 0

    while True:
        if zed is not None:
            if zed.grab(zed_runtime) != sl.ERROR_CODE.SUCCESS:
                break
            zed.retrieve_image(zed_image, sl.VIEW.LEFT)
            frame = zed_image.get_data()
            if frame.shape[2] == 4:
                frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
        else:
            ret, frame = cap.read()
            if not ret:
                break

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        if hasattr(aruco, "ArucoDetector"):
            detector = aruco.ArucoDetector(aruco_dict)
            corners, ids, _ = detector.detectMarkers(gray)
        else:
            corners, ids, _ = aruco.detectMarkers(gray, aruco_dict)

        count = 0 if ids is None else len(ids)
        cv2.putText(
            frame,
            f"ArUco count: {count} | saved: {save_count}",
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 0),
            2,
        )

        if ids is not None and len(ids) > 0:
            aruco.drawDetectedMarkers(frame, corners, ids)
            rvecs, tvecs = _estimate_pose(corners, marker_length_m, camera_matrix, dist_coeffs)
            if rvecs is not None:
                for rvec, tvec in zip(rvecs, tvecs):
                    if hasattr(aruco, "drawAxis"):
                        aruco.drawAxis(frame, camera_matrix, dist_coeffs, rvec, tvec, marker_length_m * 0.5)
                    else:
                        cv2.drawFrameAxes(
                            frame, camera_matrix, dist_coeffs, rvec, tvec, marker_length_m * 0.5
                        )

        cv2.imshow("Manual ArUco Capture", frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break
        if key == ord("s") and ids is not None and len(ids) > 0:
            rvecs, tvecs = _estimate_pose(corners, marker_length_m, camera_matrix, dist_coeffs)
            if rvecs is None:
                continue
            ts = time.time()
            for marker_id, rvec, tvec in zip(ids.flatten(), rvecs, tvecs):
                r = rvec.reshape(-1).tolist()
                t = tvec.reshape(-1).tolist()
                writer.writerow([ts, int(marker_id), r[0], r[1], r[2], t[0], t[1], t[2]])
            save_count += 1
            print(f"Saved {len(ids)} marker(s) at {ts}")

    if cap is not None:
        cap.release()
    if zed is not None:
        zed.close()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
