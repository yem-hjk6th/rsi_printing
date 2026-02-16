import argparse
from dataclasses import dataclass
from typing import Optional, Tuple

import cv2
import cv2.aruco as aruco
import numpy as np

try:
    import pyzed.sl as sl
except Exception:
    sl = None

# HD1080
CAMERA_MATRIX = np.array(
    [[951.72229004, 0.0, 638.30792236], [0.0, 951.72229004, 352.57427979], [0.0, 0.0, 1.0]],
    dtype=np.float64,
)
DIST_COEFFS = np.array([0.0, 0.0, 0.0, 0.0, 0.0], dtype=np.float64)
MARKER_LENGTH_MM = 50.0

# ZED camera settings (edit here)
USE_ZED = True
ZED_RESOLUTION = "HD1080"  # HD2K, HD1080, HD720, VGA
ZED_FPS = 30
ZED_ENABLE_DEPTH = True
ZED_DEPTH_MODE = "QUALITY"  # NONE, PERFORMANCE, QUALITY, ULTRA
ZED_COORDINATE_UNITS = "MILLIMETER"

# IMU_HARDCODED_START
IMU_SAMPLE_TIMESTAMP_NS = 1770760939393317200
IMU_LINEAR_ACCEL = np.array([0.10819728672504425, 6.609189510345459, -7.237473011016846], dtype=np.float64)
IMU_ANGULAR_VELOCITY = np.array([-0.381019651889801, 0.11768340319395065, -0.03277638554573059], dtype=np.float64)
IMU_ORIENTATION_QUAT = np.array([0.9143733382225037, -0.004001015797257423, 0.002460542833432555, -0.404844731092453], dtype=np.float64)
# IMU_HARDCODED_END



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


@dataclass
class PoseResult:
    rvec: np.ndarray
    tvec: np.ndarray


def load_camera_intrinsics() -> Tuple[np.ndarray, np.ndarray]:
    return CAMERA_MATRIX, DIST_COEFFS


def detect_single_marker_pose(
    frame: np.ndarray,
    aruco_dict: cv2.aruco_Dictionary,
    marker_length_m: float,
    camera_matrix: np.ndarray,
    dist_coeffs: np.ndarray,
) -> Tuple[np.ndarray, Optional[PoseResult]]:
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    if hasattr(aruco, "ArucoDetector"):
        detector = aruco.ArucoDetector(aruco_dict)
        corners, ids, _ = detector.detectMarkers(gray)
    else:
        corners, ids, _ = aruco.detectMarkers(gray, aruco_dict)
    if ids is None or len(ids) == 0:
        return frame, None

    aruco.drawDetectedMarkers(frame, corners, ids)

    if hasattr(aruco, "estimatePoseSingleMarkers"):
        rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
            corners, marker_length_m, camera_matrix, dist_coeffs
        )
        rvec = rvecs[0].reshape(3, 1)
        tvec = tvecs[0].reshape(3, 1)
    else:
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
        img_points = corners[0].reshape(4, 2).astype(np.float32)
        ok, rvec, tvec = cv2.solvePnP(
            obj_points, img_points, camera_matrix, dist_coeffs, flags=cv2.SOLVEPNP_IPPE_SQUARE
        )
        if not ok:
            return frame, None

    if hasattr(aruco, "drawAxis"):
        aruco.drawAxis(frame, camera_matrix, dist_coeffs, rvec, tvec, marker_length_m * 0.5)
    else:
        cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvec, tvec, marker_length_m * 0.5)
    return frame, PoseResult(rvec=rvec, tvec=tvec)


def save_pose_csv(out_path: str, pose: PoseResult) -> None:
    r = pose.rvec.reshape(-1).tolist()
    t = pose.tvec.reshape(-1).tolist()
    with open(out_path, "w", encoding="utf-8") as f:
        f.write("rvec_x,rvec_y,rvec_z,tvec_x,tvec_y,tvec_z\n")
        f.write(f"{r[0]},{r[1]},{r[2]},{t[0]},{t[1]},{t[2]}\n")


def run_aruco_pose_estimation(
    source: int,
    dict_name: str,
    marker_length_mm: float,
    intrinsics_path: str,
    out_pose_path: str,
    show: bool,
) -> None:
    aruco_dict = aruco.getPredefinedDictionary(getattr(aruco, dict_name))
    marker_length_m = marker_length_mm / 1000.0
    camera_matrix, dist_coeffs = load_camera_intrinsics()

    cap = None
    zed = None
    if USE_ZED:
        zed = _open_zed_camera()
        zed_image = sl.Mat()
        zed_runtime = sl.RuntimeParameters()
    else:
        cap = cv2.VideoCapture(source)
        if not cap.isOpened():
            raise RuntimeError("Failed to open camera source.")

    last_pose = None
    while True:
        if USE_ZED:
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

        frame, pose = detect_single_marker_pose(
            frame, aruco_dict, marker_length_m, camera_matrix, dist_coeffs
        )
        if pose is not None:
            last_pose = pose

        if show:
            cv2.imshow("ArUco Pose", frame)
            key = cv2.waitKey(1)
            if key == ord("q"):
                break
            if key == ord("s") and last_pose is not None:
                save_pose_csv(out_pose_path, last_pose)
                print(f"Saved pose: {out_pose_path}")

    if cap is not None:
        cap.release()
    if zed is not None:
        zed.close()
    cv2.destroyAllWindows()

    if last_pose is not None and not show:
        save_pose_csv(out_pose_path, last_pose)
        print(f"Saved pose: {out_pose_path}")


def main() -> None:
    parser = argparse.ArgumentParser(description="ArUco detection + pose estimation")
    parser.add_argument("--source", type=int, default=0, help="Camera index")
    parser.add_argument(
        "--dict",
        type=str,
        default="DICT_6X6_250",
        help="ArUco dictionary name, e.g. DICT_6X6_250",
    )
    parser.add_argument(
        "--marker-length-mm",
        type=float,
        required=False,
        default=MARKER_LENGTH_MM,
        help="Marker side length in mm (physical size)",
    )
    parser.add_argument(
        "--intrinsics",
        type=str,
        required=False,
        default=None,
        help="Unused. Intrinsics are hardcoded from zed_left_intrinsics.npz",
    )
    parser.add_argument(
        "--out-pose",
        type=str,
        default="aruco_pose.csv",
        help="Output pose CSV path",
    )
    parser.add_argument("--no-show", action="store_true", help="Disable preview window")

    args = parser.parse_args()

    run_aruco_pose_estimation(
        source=args.source,
        dict_name=args.dict,
        marker_length_mm=args.marker_length_mm,
        intrinsics_path=args.intrinsics,
        out_pose_path=args.out_pose,
        show=not args.no_show,
    )


if __name__ == "__main__":
    main()
