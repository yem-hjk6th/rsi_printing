"""
demo3_stitch_L_R.py — Show only the stereo overlap region from left & right views.

For rectified stereo (ZED SDK output):
  - Left overlap:  columns [d_max, width-1]   → same scene as
  - Right overlap: columns [0, width-1-d_max]

Press +/- to adjust Z_min, 'q' to quit.
"""
import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
import zed_setup  # noqa: E402
import pyzed.sl as sl
import cv2
import numpy as np


def main():
    zed = sl.Camera()
    init = sl.InitParameters()
    init.camera_resolution = sl.RESOLUTION.HD2K
    init.camera_fps = 15
    init.depth_mode = sl.DEPTH_MODE.NONE
    init.coordinate_units = sl.UNIT.MILLIMETER

    if zed.open(init) != sl.ERROR_CODE.SUCCESS:
        print("Failed to open camera")
        return

    info = zed.get_camera_information()
    calib = info.camera_configuration.calibration_parameters
    res = info.camera_configuration.resolution
    w, h = res.width, res.height
    fx = calib.left_cam.fx
    baseline_mm = calib.get_camera_baseline()

    z_min = 300.0  # mm, adjustable

    left_mat = sl.Mat()
    right_mat = sl.Mat()
    runtime = sl.RuntimeParameters()

    cv2.namedWindow("Stereo Overlap", cv2.WINDOW_NORMAL)

    while True:
        if zed.grab(runtime) != sl.ERROR_CODE.SUCCESS:
            continue

        zed.retrieve_image(left_mat, sl.VIEW.LEFT)
        zed.retrieve_image(right_mat, sl.VIEW.RIGHT)
        left = left_mat.get_data()[:, :, :3].copy()
        right = right_mat.get_data()[:, :, :3].copy()

        d_max = min(int(fx * baseline_mm / z_min), w - 1)
        overlap_w = w - d_max

        # Crop overlap regions (same physical scene)
        left_crop = left[:, d_max:]        # [d_max .. w-1]
        right_crop = right[:, :overlap_w]  # [0 .. w-1-d_max]

        # Labels
        info_txt = f"Z_min={z_min:.0f}mm  d_max={d_max}px  overlap={overlap_w}px/{w}px ({100*overlap_w/w:.0f}%)"
        for crop, label in [(left_crop, "LEFT overlap"), (right_crop, "RIGHT overlap")]:
            cv2.putText(crop, label, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            cv2.putText(crop, info_txt, (10, h - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (200, 200, 200), 1)

        # Separator line
        sep = np.full((h, 3, 3), (0, 0, 255), dtype=np.uint8)
        canvas = np.hstack([left_crop, sep, right_crop])
        cv2.imshow("Stereo Overlap", canvas)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key in (ord('+'), ord('=')):
            z_min = min(z_min + 50, 5000)
        elif key in (ord('-'), ord('_')):
            z_min = max(z_min - 50, 100)

    zed.close()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
