import time

import cv2
import numpy as np
import pyzed.sl as sl


class Config:
    # ===== User Settings (edit here) =====
    # Resolution options:
    #   sl.RESOLUTION.HD2K   -> 2208x1242
    #   sl.RESOLUTION.HD1080 -> 1920x1080
    #   sl.RESOLUTION.HD720  -> 1280x720
    #   sl.RESOLUTION.VGA    -> 672x376
    RESOLUTION = sl.RESOLUTION.HD1080
    FPS = 30
    DEPTH_MODE = sl.DEPTH_MODE.NEURAL
    COORDINATE_UNITS = sl.UNIT.METER
    DEPTH_MIN_M = 0.20
    DEPTH_MAX_M = 3.00

    PIXEL_1 = (780, 520)
    PIXEL_2 = (1120, 520)

    DEPTH_PATCH_RADIUS = 1
    MIN_VALID_SAMPLES = 3
    PRINT_EVERY_SEC = 1.0

    WINDOW_NAME = "ZED Pixel Distance"
    CROSS_SIZE = 8
    CROSS_THICKNESS = 2
    LINE_THICKNESS = 2

    # Hard-coded camera-to-gripper extrinsics (T_cam2gripper).
    # Replace this matrix when a higher-accuracy calibration is available.
    # Required format:
    #   - 4x4 homogeneous matrix
    #   - row-major order
    #   - translation unit: meter
    #   - last row must be [0, 0, 0, 1]
    # Example source file format can be copied from extrinsics.txt and pasted here.
    T_CAM2GRIPPER = np.array(
        [
            [0.066052, 0.823625, 0.563276, -0.322889],
            [0.997690, -0.063505, -0.024136, -0.066070],
            [0.015892, 0.563569, -0.825916, 0.419890],
            [0.0, 0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )


def resolution_to_text(resolution):
    mapping = {
        sl.RESOLUTION.HD2K: "2208x1242",
        sl.RESOLUTION.HD1080: "1920x1080",
        sl.RESOLUTION.HD720: "1280x720",
        sl.RESOLUTION.VGA: "672x376",
    }
    return mapping.get(resolution, "unknown")


def init_camera(config=Config):
    zed = sl.Camera()
    init_params = sl.InitParameters()
    init_params.camera_resolution = config.RESOLUTION
    init_params.camera_fps = config.FPS
    init_params.depth_mode = config.DEPTH_MODE
    init_params.coordinate_units = config.COORDINATE_UNITS
    init_params.depth_minimum_distance = config.DEPTH_MIN_M
    init_params.depth_maximum_distance = config.DEPTH_MAX_M

    status = zed.open(init_params)
    if status != sl.ERROR_CODE.SUCCESS:
        raise RuntimeError(f"Failed to open ZED: {status}")
    return zed


def get_left_intrinsics(zed):
    calib = zed.get_camera_information().camera_configuration.calibration_parameters
    left = calib.left_cam
    return float(left.fx), float(left.fy), float(left.cx), float(left.cy)


def robust_xyz(point_cloud, u, v, patch_radius=1, min_valid_samples=1):
    h = point_cloud.get_height()
    w = point_cloud.get_width()

    xs, ys, zs = [], [], []
    total_samples = (2 * patch_radius + 1) ** 2
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

    valid_samples = len(xs)
    if valid_samples < min_valid_samples:
        return None, valid_samples, total_samples
    point = np.array([np.median(xs), np.median(ys), np.median(zs)], dtype=np.float64)
    return point, valid_samples, total_samples


def to_homo(point3):
    return np.array([point3[0], point3[1], point3[2], 1.0], dtype=np.float64)


def draw_red_cross(frame, pt, size=8, thickness=2):
    u, v = int(pt[0]), int(pt[1])
    color = (0, 0, 255)
    cv2.line(frame, (u - size, v - size), (u + size, v + size), color, thickness)
    cv2.line(frame, (u - size, v + size), (u + size, v - size), color, thickness)


def run_demo(config=Config):
    zed = init_camera(config)

    image = sl.Mat()
    point_cloud = sl.Mat()
    runtime = sl.RuntimeParameters()
    runtime.enable_fill_mode = True

    fx, fy, cx, cy = get_left_intrinsics(zed)

    t_cam2gripper = config.T_CAM2GRIPPER
    if t_cam2gripper.shape != (4, 4):
        raise ValueError("T_CAM2GRIPPER must be a 4x4 matrix")
    if not np.allclose(t_cam2gripper[3], np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float64)):
        raise ValueError("T_CAM2GRIPPER last row must be [0, 0, 0, 1]")
    print("[INFO] Using hard-coded T_cam2gripper from Config.T_CAM2GRIPPER")

    print(f"[INFO] Resolution = {resolution_to_text(config.RESOLUTION)}")
    print(f"[INFO] Intrinsics (left): fx={fx:.2f}, fy={fy:.2f}, cx={cx:.2f}, cy={cy:.2f}")
    print(f"[INFO] Pixel 1 = {config.PIXEL_1}, Pixel 2 = {config.PIXEL_2}")
    print("[INFO] Press 'q' to quit.")

    cv2.namedWindow(config.WINDOW_NAME, cv2.WINDOW_NORMAL)

    last_print = 0.0
    last_valid_dist_mm = None

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
            p1_cam, p1_valid, p1_total = robust_xyz(
                point_cloud,
                u1,
                v1,
                config.DEPTH_PATCH_RADIUS,
                min_valid_samples=config.MIN_VALID_SAMPLES,
            )
            p2_cam, p2_valid, p2_total = robust_xyz(
                point_cloud,
                u2,
                v2,
                config.DEPTH_PATCH_RADIUS,
                min_valid_samples=config.MIN_VALID_SAMPLES,
            )

            draw_red_cross(frame, (u1, v1), size=config.CROSS_SIZE, thickness=config.CROSS_THICKNESS)
            draw_red_cross(frame, (u2, v2), size=config.CROSS_SIZE, thickness=config.CROSS_THICKNESS)
            cv2.line(frame, (u1, v1), (u2, v2), (0, 0, 255), config.LINE_THICKNESS)

            text = f"invalid depth | p1 {p1_valid}/{p1_total}, p2 {p2_valid}/{p2_total}"
            if p1_cam is not None and p2_cam is not None:
                dist_cam_m = float(np.linalg.norm(p1_cam - p2_cam))
                text = f"cam_dist = {dist_cam_m * 1000.0:.1f} mm"
                last_valid_dist_mm = dist_cam_m * 1000.0

                now = time.time()
                if now - last_print >= config.PRINT_EVERY_SEC:
                    message = (
                        f"cam: p1={p1_cam.round(4).tolist()} m, "
                        f"p2={p2_cam.round(4).tolist()} m, "
                        f"dist={dist_cam_m * 1000.0:.1f} mm"
                    )
                    if t_cam2gripper is not None:
                        p1_g = (t_cam2gripper @ to_homo(p1_cam))[:3]
                        p2_g = (t_cam2gripper @ to_homo(p2_cam))[:3]
                        dist_g_m = float(np.linalg.norm(p1_g - p2_g))
                        message += f" | gripper_dist={dist_g_m * 1000.0:.1f} mm"

                    print(message)
                    last_print = now

            if last_valid_dist_mm is not None and (p1_cam is None or p2_cam is None):
                text = (
                    f"invalid depth | p1 {p1_valid}/{p1_total}, p2 {p2_valid}/{p2_total} "
                    f"| last={last_valid_dist_mm:.1f} mm"
                )

            cv2.putText(frame, text, (25, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (20, 20, 255), 2)
            cv2.imshow(config.WINDOW_NAME, frame)

            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

    finally:
        cv2.destroyAllWindows()
        zed.close()


if __name__ == "__main__":
    run_demo()
