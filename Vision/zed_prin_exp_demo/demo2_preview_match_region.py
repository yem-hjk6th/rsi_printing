"""
demo2_preview_dist.py — Preview left/right rectified images with disparity ROI overlay.

Red box = region where full disparity range is available for depth computation.
  - LEFT image:  x ∈ [d_max, width-1]  (left border loses d_max cols — no right match exists)
  - RIGHT image: x ∈ [0, width-1-d_max] (right border loses d_max cols — no left query exists)

Where d_max = fx * baseline / Z_min  (max disparity at closest valid depth).

Press 'q' to quit. Press '+'/'-' to adjust Z_min live and see ROI change.
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
    init.depth_mode = sl.DEPTH_MODE.ULTRA
    init.coordinate_units = sl.UNIT.MILLIMETER
    init.depth_minimum_distance = 200  # mm

    if zed.open(init) != sl.ERROR_CODE.SUCCESS:
        print("Failed to open camera")
        return

    info = zed.get_camera_information()
    calib = info.camera_configuration.calibration_parameters
    res = info.camera_configuration.resolution
    w, h = res.width, res.height
    fx = calib.left_cam.fx
    baseline_mm = calib.get_camera_baseline()  # mm

    print(f"Resolution: {w}x{h}, fx={fx:.1f}, baseline={baseline_mm:.1f}mm")

    z_min = init.depth_minimum_distance  # mm, adjustable

    left_mat = sl.Mat()
    right_mat = sl.Mat()
    runtime = sl.RuntimeParameters()

    cv2.namedWindow("Stereo Preview", cv2.WINDOW_NORMAL)

    while True:
        if zed.grab(runtime) != sl.ERROR_CODE.SUCCESS:
            continue

        zed.retrieve_image(left_mat, sl.VIEW.LEFT)
        zed.retrieve_image(right_mat, sl.VIEW.RIGHT)
        left = left_mat.get_data()[:, :, :3].copy()
        right = right_mat.get_data()[:, :, :3].copy()

        # max disparity at current z_min
        d_max = int(fx * baseline_mm / z_min)
        d_max = min(d_max, w - 1)

        # LEFT: valid region starts at d_max (left cols have no right-image match)
        cv2.rectangle(left, (d_max, 0), (w - 1, h - 1), (0, 0, 255), 2)
        # shade the dead zone
        overlay = left.copy()
        cv2.rectangle(overlay, (0, 0), (d_max, h - 1), (0, 0, 180), -1)
        cv2.addWeighted(overlay, 0.3, left, 0.7, 0, left)

        # RIGHT: valid region ends at w-1-d_max (right cols not queried by left)
        cv2.rectangle(right, (0, 0), (w - 1 - d_max, h - 1), (0, 0, 255), 2)
        overlay = right.copy()
        cv2.rectangle(overlay, (w - d_max, 0), (w - 1, h - 1), (0, 0, 180), -1)
        cv2.addWeighted(overlay, 0.3, right, 0.7, 0, right)

        # info text
        txt = f"Z_min={z_min:.0f}mm  d_max={d_max}px  valid={w-d_max}px/{w}px ({100*(w-d_max)/w:.0f}%)"
        for img in [left, right]:
            cv2.putText(img, txt, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 255), 2)

        cv2.putText(left, "LEFT (query)", (10, h - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.putText(right, "RIGHT (search)", (10, h - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

        canvas = np.hstack([left, right])
        cv2.imshow("Stereo Preview", canvas)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('+') or key == ord('='):
            z_min = min(z_min + 50, 5000)
            print(f"Z_min → {z_min:.0f}mm, d_max → {int(fx * baseline_mm / z_min)}px")
        elif key == ord('-') or key == ord('_'):
            z_min = max(z_min - 50, 100)
            print(f"Z_min → {z_min:.0f}mm, d_max → {int(fx * baseline_mm / z_min)}px")

    zed.close()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
