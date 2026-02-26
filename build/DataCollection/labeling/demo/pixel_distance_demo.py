import time
from pathlib import Path

import cv2
import numpy as np
import pyzed.sl as sl


class DemoConfig:
    # Preview resolution (choose one):
    # sl.RESOLUTION.HD2K   -> 2208x1242
    # sl.RESOLUTION.HD1080 -> 1920x1080
    # sl.RESOLUTION.HD720  -> 1280x720
    # sl.RESOLUTION.VGA    -> 672x376
    RESOLUTION = sl.RESOLUTION.HD720
    FPS = 30
    DEPTH_MODE = sl.DEPTH_MODE.ULTRA
    COORDINATE_UNITS = sl.UNIT.METER

    PIXEL_1 = (520, 408)
    PIXEL_2 = (720, 408)

    DEPTH_PATCH_RADIUS = 1
    PRINT_EVERY_SEC = 1.0
    WINDOW_NAME = "ZED Two-Pixel Distance Demo"

    EXTRINSICS_PATH = Path("../../Preparation/calibration/aut_cal/extrinsics_res/20260219_174710/extrinsics.txt")


def init_camera(config=DemoConfig):
    zed = sl.Camera()
    init_params = sl.InitParameters()
    init_params.camera_resolution = config.RESOLUTION
    init_params.camera_fps = config.FPS
    init_params.depth_mode = config.DEPTH_MODE
    init_params.coordinate_units = config.COORDINATE_UNITS

    status = zed.open(init_params)
    if status != sl.ERROR_CODE.SUCCESS:
        raise RuntimeError(f"Failed to open ZED: {status}")
    return zed


def resolution_to_text(resolution):
    mapping = {
        sl.RESOLUTION.HD2K: "2208x1242",
        sl.RESOLUTION.HD1080: "1920x1080",
        sl.RESOLUTION.HD720: "1280x720",
        sl.RESOLUTION.VGA: "672x376",
    }
    return mapping.get(resolution, "unknown")


def parse_extrinsics_txt(file_path):
    file_path = Path(file_path)
    if not file_path.exists():
        return None

    rows = []
    for line in file_path.read_text(encoding="utf-8").splitlines():
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        vals = [float(x) for x in line.split()]
        if len(vals) == 4:
            rows.append(vals)

    if len(rows) != 4:
        return None
    return np.array(rows, dtype=np.float64)


def robust_xyz(point_cloud, u, v, patch_radius=1):
    h = point_cloud.get_height()
    w = point_cloud.get_width()

    xs, ys, zs = [], [], []
    for dv in range(-patch_radius, patch_radius + 1):
        for du in range(-patch_radius, patch_radius + 1):
            uu = int(np.clip(u + du, 0, w - 1))
            vv = int(np.clip(v + dv, 0, h - 1))
            err, value = point_cloud.get_value(uu, vv)
            if err != sl.ERROR_CODE.SUCCESS:
                continue
            x, y, z = float(value[0]), float(value[1]), float(value[2])
            if np.isfinite(x) and np.isfinite(y) and np.isfinite(z) and z > 0:
                xs.append(x)
                ys.append(y)
                zs.append(z)

    if not xs:
        return None
    return np.array([np.median(xs), np.median(ys), np.median(zs)], dtype=np.float64)


def to_homo(p3):
    return np.array([p3[0], p3[1], p3[2], 1.0], dtype=np.float64)


def draw_red_cross(frame, pt, size=8, thickness=2):
    u, v = int(pt[0]), int(pt[1])
    color = (0, 0, 255)
    cv2.line(frame, (u - size, v - size), (u + size, v + size), color, thickness)
    cv2.line(frame, (u - size, v + size), (u + size, v - size), color, thickness)


def run_demo(config=DemoConfig):
    zed = init_camera(config)

    image = sl.Mat()
    point_cloud = sl.Mat()
    runtime = sl.RuntimeParameters()

    t_cam2gripper = parse_extrinsics_txt(config.EXTRINSICS_PATH)
    if t_cam2gripper is None:
        print("[WARN] Extrinsics not loaded. Distance will still be valid in camera frame.")
    else:
        print(f"[INFO] Loaded extrinsics: {config.EXTRINSICS_PATH}")

    print(f"[INFO] Resolution = {resolution_to_text(config.RESOLUTION)}")
    print(f"[INFO] Pixel 1 = {config.PIXEL_1}, Pixel 2 = {config.PIXEL_2}")
    print("[INFO] Press 'q' to quit.")

    cv2.namedWindow(config.WINDOW_NAME, cv2.WINDOW_NORMAL)

    last_print = 0.0

    try:
        while True:
            if zed.grab(runtime) != sl.ERROR_CODE.SUCCESS:
                continue

            zed.retrieve_image(image, sl.VIEW.LEFT)
            zed.retrieve_measure(point_cloud, sl.MEASURE.XYZ)

            frame = image.get_data()
            if frame.shape[2] == 4:
                frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)

            (u1, v1), (u2, v2) = config.PIXEL_1, config.PIXEL_2
            p1_cam = robust_xyz(point_cloud, u1, v1, config.DEPTH_PATCH_RADIUS)
            p2_cam = robust_xyz(point_cloud, u2, v2, config.DEPTH_PATCH_RADIUS)

            draw_red_cross(frame, (u1, v1), size=8, thickness=2)
            draw_red_cross(frame, (u2, v2), size=8, thickness=2)
            cv2.line(frame, (u1, v1), (u2, v2), (0, 0, 255), 2)

            text = "invalid depth"
            if p1_cam is not None and p2_cam is not None:
                dist_cam_m = float(np.linalg.norm(p1_cam - p2_cam))
                text = f"cam_dist = {dist_cam_m * 1000.0:.1f} mm"

                now = time.time()
                if now - last_print >= config.PRINT_EVERY_SEC:
                    msg = (
                        f"cam: p1={p1_cam.round(4).tolist()} m, "
                        f"p2={p2_cam.round(4).tolist()} m, "
                        f"dist={dist_cam_m * 1000.0:.1f} mm"
                    )

                    if t_cam2gripper is not None:
                        p1_g = (t_cam2gripper @ to_homo(p1_cam))[:3]
                        p2_g = (t_cam2gripper @ to_homo(p2_cam))[:3]
                        dist_g_m = float(np.linalg.norm(p1_g - p2_g))
                        msg += f" | gripper_dist={dist_g_m * 1000.0:.1f} mm"

                    print(msg)
                    last_print = now

            cv2.putText(frame, text, (25, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (20, 20, 255), 2)
            cv2.imshow(config.WINDOW_NAME, frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    finally:
        cv2.destroyAllWindows()
        zed.close()


if __name__ == "__main__":
    run_demo()
