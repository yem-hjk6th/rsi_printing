"""
demo4_blend_overlap.py — Blend left & right overlap regions to inspect pixel matching.

Three display modes (press 'm' to cycle):
  1. BLEND   — 50/50 alpha blend (misaligned features show as ghosting)
  2. DIFF    — absolute difference (bright = mismatch, dark = matched)
  3. ANAGLYPH — left=red, right=cyan (use for disparity-aware visual check)

Horizontal epipolar lines drawn every N rows (toggle with 'e').
Press +/- to adjust Z_min, 'q' to quit.
"""
import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
import zed_setup  # noqa: E402
import pyzed.sl as sl
import cv2
import numpy as np


MODE_NAMES = ["BLEND", "DIFF", "ANAGLYPH"]


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

    z_min = 300.0
    mode = 0        # 0=blend, 1=diff, 2=anaglyph
    show_epi = True
    epi_step = 40   # pixels between epipolar lines

    left_mat = sl.Mat()
    right_mat = sl.Mat()
    runtime = sl.RuntimeParameters()

    cv2.namedWindow("Overlap Blend", cv2.WINDOW_NORMAL)

    while True:
        if zed.grab(runtime) != sl.ERROR_CODE.SUCCESS:
            continue

        zed.retrieve_image(left_mat, sl.VIEW.LEFT)
        zed.retrieve_image(right_mat, sl.VIEW.RIGHT)
        left = left_mat.get_data()[:, :, :3]
        right = right_mat.get_data()[:, :, :3]

        d_max = min(int(fx * baseline_mm / z_min), w - 1)
        overlap_w = w - d_max

        left_crop = left[:, d_max:].copy()
        right_crop = right[:, :overlap_w].copy()

        # Generate composite
        if mode == 0:  # BLEND
            canvas = cv2.addWeighted(left_crop, 0.5, right_crop, 0.5, 0)
        elif mode == 1:  # DIFF
            diff = cv2.absdiff(left_crop, right_crop)
            canvas = cv2.applyColorMap(cv2.cvtColor(diff, cv2.COLOR_BGR2GRAY), cv2.COLORMAP_JET)
        else:  # ANAGLYPH
            lg = cv2.cvtColor(left_crop, cv2.COLOR_BGR2GRAY)
            rg = cv2.cvtColor(right_crop, cv2.COLOR_BGR2GRAY)
            canvas = np.stack([rg, rg, lg], axis=-1)  # R=left, GB=right

        # Epipolar lines
        if show_epi:
            for y in range(0, h, epi_step):
                cv2.line(canvas, (0, y), (overlap_w - 1, y), (0, 255, 0), 1)

        # Info
        txt = f"[{MODE_NAMES[mode]}] Z_min={z_min:.0f}mm  d_max={d_max}px  overlap={overlap_w}px"
        cv2.putText(canvas, txt, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 255), 2)
        cv2.putText(canvas, "m:mode  e:epipolar  +/-:Zmin  q:quit", (10, h - 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)

        cv2.imshow("Overlap Blend", canvas)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('m'):
            mode = (mode + 1) % 3
        elif key == ord('e'):
            show_epi = not show_epi
        elif key in (ord('+'), ord('=')):
            z_min = min(z_min + 50, 5000)
        elif key in (ord('-'), ord('_')):
            z_min = max(z_min - 50, 100)

    zed.close()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
