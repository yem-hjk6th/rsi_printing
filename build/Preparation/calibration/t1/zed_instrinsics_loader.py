import os
import numpy as np
import pyzed.sl as sl

OUTPUT_DIR = os.path.join(
    os.path.dirname(__file__),
    "camera",
)


def main() -> None:
    os.makedirs(OUTPUT_DIR, exist_ok=True)

    zed = sl.Camera()
    init_params = sl.InitParameters()
    init_params.camera_resolution = sl.RESOLUTION.HD1080
    init_params.depth_mode = sl.DEPTH_MODE.QUALITY

    if zed.open(init_params) != sl.ERROR_CODE.SUCCESS:
        raise RuntimeError("Failed to open ZED camera")

    info = zed.get_camera_information()
    calib = info.camera_configuration.calibration_parameters

    def build_intrinsics(cam) -> tuple[np.ndarray, np.ndarray]:
        fx, fy = cam.fx, cam.fy
        cx, cy = cam.cx, cam.cy
        disto = cam.disto
        camera_matrix = np.array(
            [[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]], dtype=np.float64
        )
        dist_coeffs = np.array(
            [disto[0], disto[1], disto[2], disto[3], disto[4]], dtype=np.float64
        )
        return camera_matrix, dist_coeffs

    left_camera_matrix, left_dist_coeffs = build_intrinsics(calib.left_cam)

    left_out_path = os.path.join(OUTPUT_DIR, "zed_left_intrinsics.npz")
    np.savez(
        left_out_path,
        camera_matrix=left_camera_matrix,
        dist_coeffs=left_dist_coeffs,
        resolution=np.array(
            [info.camera_configuration.resolution.width, info.camera_configuration.resolution.height],
            dtype=np.int32,
        ),
        fps=np.array([info.camera_configuration.fps], dtype=np.int32),
        serial_number=np.array([info.serial_number], dtype=np.int64),
    )

    if hasattr(calib, "depth_cam"):
        depth_camera_matrix, depth_dist_coeffs = build_intrinsics(calib.depth_cam)
        depth_out_path = os.path.join(OUTPUT_DIR, "zed_depth_intrinsics.npz")
        np.savez(
            depth_out_path,
            camera_matrix=depth_camera_matrix,
            dist_coeffs=depth_dist_coeffs,
            resolution=np.array(
                [info.camera_configuration.resolution.width, info.camera_configuration.resolution.height],
                dtype=np.int32,
            ),
            fps=np.array([info.camera_configuration.fps], dtype=np.int32),
            serial_number=np.array([info.serial_number], dtype=np.int64),
        )

    zed.close()
    print(f"Saved intrinsics: {left_out_path}")
    if hasattr(calib, "depth_cam"):
        print(f"Saved intrinsics: {depth_out_path}")


if __name__ == "__main__":
    main()
