"""
demo5_resol_preview.py — Sharpness / edge-acuity heatmap on the stereo overlap region.

Pipeline:
  1. Grab left rectified image, crop to overlap region (same as demo3).
  2. Convert to gray, compute Laplacian variance in a sliding window → sharpness map.
  3. Overlay colour heatmap on the live image (blue=blurry → red=sharp).
  4. Run Canny edge detection; show edge density per grid cell as numeric overlay.

Keys:
  +/- : adjust Z_min (changes overlap crop)
  w/s : adjust analysis block size (default 64 px)
  q   : quit
"""
import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
import zed_setup  # noqa: E402
import pyzed.sl as sl
import cv2
import numpy as np


def laplacian_sharpness_map(gray: np.ndarray, blk: int) -> np.ndarray:
    """Return per-block Laplacian-variance sharpness map (float32, same size as gray)."""
    lap = cv2.Laplacian(gray, cv2.CV_32F)
    lap_sq = lap * lap
    # integral image for fast block variance
    integral = cv2.integral(lap_sq.astype(np.float64))
    h, w = gray.shape
    out = np.zeros((h, w), dtype=np.float32)
    half = blk // 2
    for y in range(0, h, blk):
        for x in range(0, w, blk):
            y0 = max(y, 0)
            x0 = max(x, 0)
            y1 = min(y + blk, h)
            x1 = min(x + blk, w)
            area = (y1 - y0) * (x1 - x0)
            s = integral[y1, x1] - integral[y0, x1] - integral[y1, x0] + integral[y0, x0]
            val = s / area
            out[y0:y1, x0:x1] = val
    return out


def edge_density_grid(gray: np.ndarray, blk: int):
    """Return Canny edge image and per-cell edge-pixel ratio array (rows x cols)."""
    edges = cv2.Canny(gray, 50, 150)
    h, w = gray.shape
    rows = (h + blk - 1) // blk
    cols = (w + blk - 1) // blk
    density = np.zeros((rows, cols), dtype=np.float32)
    for r in range(rows):
        for c in range(cols):
            y0, y1 = r * blk, min((r + 1) * blk, h)
            x0, x1 = c * blk, min((c + 1) * blk, w)
            area = (y1 - y0) * (x1 - x0)
            density[r, c] = edges[y0:y1, x0:x1].sum() / 255.0 / area
    return edges, density


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
    blk = 64  # analysis block size in pixels

    left_mat = sl.Mat()
    right_mat = sl.Mat()
    runtime = sl.RuntimeParameters()

    cv2.namedWindow("Sharpness Preview", cv2.WINDOW_NORMAL)

    while True:
        if zed.grab(runtime) != sl.ERROR_CODE.SUCCESS:
            continue

        zed.retrieve_image(left_mat, sl.VIEW.LEFT)
        zed.retrieve_image(right_mat, sl.VIEW.RIGHT)
        left = left_mat.get_data()[:, :, :3].copy()
        right = right_mat.get_data()[:, :, :3].copy()

        d_max = min(int(fx * baseline_mm / z_min), w - 1)
        overlap_w = w - d_max
        left_crop = left[:, d_max:]          # left overlap
        right_crop = right[:, :overlap_w]    # right overlap

        panels = []
        for crop, label in [(left_crop, "LEFT"), (right_crop, "RIGHT")]:
            gray = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)

            # --- sharpness heatmap ---
            smap = laplacian_sharpness_map(gray, blk)
            smap_log = np.log1p(smap)
            smax = smap_log.max() if smap_log.max() > 0 else 1.0
            smap_u8 = np.clip(smap_log / smax * 255, 0, 255).astype(np.uint8)
            heatmap = cv2.applyColorMap(smap_u8, cv2.COLORMAP_JET)
            blended = cv2.addWeighted(crop, 0.55, heatmap, 0.45, 0)

            # --- edge density numbers ---
            edges, density = edge_density_grid(gray, blk)
            ch, cw = crop.shape[:2]
            rows = (ch + blk - 1) // blk
            cols = (cw + blk - 1) // blk
            for r in range(rows):
                for c in range(cols):
                    cx = c * blk + blk // 2
                    cy = r * blk + blk // 2
                    if cx >= cw or cy >= ch:
                        continue
                    pct = density[r, c] * 100
                    if pct > 1.0:
                        cv2.putText(blended, f"{pct:.0f}", (cx - 12, cy + 5),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.35, (255, 255, 255), 1,
                                    cv2.LINE_AA)

            # draw grid
            for r in range(1, rows):
                cv2.line(blended, (0, r * blk), (cw, r * blk), (80, 80, 80), 1)
            for c in range(1, cols):
                cv2.line(blended, (c * blk, 0), (c * blk, ch), (80, 80, 80), 1)

            # label & info
            mean_sharp = float(smap.mean())
            info_txt = (f"{label}  blk={blk}  Z_min={z_min:.0f}mm  overlap={overlap_w}px  "
                        f"sharp={mean_sharp:.1f}  [+/-] Z  [w/s] blk  [q] quit")
            cv2.putText(blended, info_txt, (10, 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)

            # small edge overlay in corner
            edge_small = cv2.resize(edges, (cw // 4, ch // 4))
            edge_bgr = cv2.cvtColor(edge_small, cv2.COLOR_GRAY2BGR)
            eh, ew = edge_bgr.shape[:2]
            blended[ch - eh:ch, cw - ew:cw] = edge_bgr

            panels.append(blended)

        sep = np.full((h, 3, 3), (0, 0, 255), dtype=np.uint8)
        canvas = np.hstack([panels[0], sep, panels[1]])
        cv2.imshow("Sharpness Preview", canvas)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key in (ord('+'), ord('=')):
            z_min = min(z_min + 50, 5000)
        elif key in (ord('-'), ord('_')):
            z_min = max(z_min - 50, 100)
        elif key == ord('w'):
            blk = min(blk * 2, 256)
            print(f"block size → {blk}")
        elif key == ord('s'):
            blk = max(blk // 2, 16)
            print(f"block size → {blk}")

    zed.close()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
