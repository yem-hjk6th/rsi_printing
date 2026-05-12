"""
demo9_distance_sweep.py — ZED2i point cloud quality vs. capture distance.

Fixed single viewpoint, fixed target object.  User manually repositions camera
(or object) to each target distance, then presses [SPACE] to capture.
After all distances are done (or [q] to stop early), a CSV + summary plot are
written automatically.

Metrics computed per distance:
  valid_ratio   — fraction of pixels with valid depth (in ROI)
  depth_noise   — stddev of depth in a user-drawn flat-surface ROI (mm)
  point_density — valid 3D points per cm² of projected area
  plane_rmse    — RMSE after fitting a plane to the point cloud (mm)
                  (good proxy for absolute accuracy on a flat target)

Controls:
  Mouse drag    : draw ROI rectangle (use on the flat reference surface)
  r             : reset ROI
  SPACE         : capture current distance → compute metrics → advance
  s             : skip current distance (mark as NaN)
  q             : quit / finish early → write results

Usage:
  1. Set DISTANCES_MM below to the distances you want to test.
  2. Run the script.
  3. For each distance, position the camera/object, draw ROI on the flat surface,
     press SPACE.  The script prints metrics and moves to the next distance.
  4. Results written to vision_demo_test_res/distance_sweep_<timestamp>/

Notes:
  - Uses ZED SDK ULTRA depth (same as demo8), no FFS subprocess needed.
  - Distances are nominal; script also records the measured median depth
    from the captured frames as "actual_z_mm".
  - Keep the target object still; the script captures N frames and median-fuses.
"""

import sys, os, time, csv
from pathlib import Path

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
import zed_setup  # noqa: E402
import pyzed.sl as sl
import cv2
import numpy as np

# ── Config ────────────────────────────────────────────────────────────────────
DISTANCES_MM    = [500, 800, 1200, 1500]  # nominal target distances in mm
N_FRAMES        = 15                       # frames to median-fuse per position
SETTLE_SKIP     = 8                        # frames to discard (auto-exposure)

# Auto-lock parameters
LOCK_WINDOW     = 20     # rolling window size (frames) for stability check
LOCK_TOL_MM     = 40     # ±mm around target to be considered "in range"
LOCK_STD_MM     = 15     # max std within window to be considered stable
LOCK_MIN_VALID  = 0.30   # min valid_ratio in patch to trust the reading
DEPTH_MIN_M     = 0.15                     # ignore closer than this (m)
DEPTH_MAX_M     = 2.50                     # ignore farther than this (m)
DISC_THRESH     = 0.05
DISPLAY_SCALE   = 0.5

SCRIPT_DIR  = Path(__file__).resolve().parent
VISION_DIR  = SCRIPT_DIR.parent
OUTPUT_ROOT = VISION_DIR / "vision_demo_test_res"


# ══════════════════════════════════════════════════════════════════════════════
#  ROI selector (same as demo8)
# ══════════════════════════════════════════════════════════════════════════════

class ROISelector:
    def __init__(self):
        self.drawing = False
        self.start = self.end = None
        self.roi_set = False

    def callback(self, event, x, y, flags, param):
        scale = param.get("scale", 1.0)
        inv = 1.0 / scale
        if event == cv2.EVENT_LBUTTONDOWN:
            self.drawing = True
            self.start = (int(x * inv), int(y * inv))
            self.end = self.start
            self.roi_set = False
        elif event == cv2.EVENT_MOUSEMOVE and self.drawing:
            self.end = (int(x * inv), int(y * inv))
        elif event == cv2.EVENT_LBUTTONUP:
            self.drawing = False
            self.end = (int(x * inv), int(y * inv))
            self.roi_set = True

    def get_rect(self, W, H):
        if not self.roi_set or self.start is None:
            return None
        x0 = max(0, min(self.start[0], self.end[0]))
        y0 = max(0, min(self.start[1], self.end[1]))
        x1 = min(W - 1, max(self.start[0], self.end[0]))
        y1 = min(H - 1, max(self.start[1], self.end[1]))
        if x1 - x0 < 10 or y1 - y0 < 10:
            return None
        return (x0, y0, x1, y1)

    def reset(self):
        self.drawing = False
        self.start = self.end = None
        self.roi_set = False


# ══════════════════════════════════════════════════════════════════════════════
#  Capture helpers
# ══════════════════════════════════════════════════════════════════════════════

def grab_frames(zed, n_frames, settle_skip=SETTLE_SKIP):
    left_mat  = sl.Mat()
    depth_mat = sl.Mat()
    runtime   = sl.RuntimeParameters()

    for _ in range(settle_skip):
        zed.grab(runtime)

    depths = []
    color_bgr = None
    for i in range(n_frames):
        if zed.grab(runtime) != sl.ERROR_CODE.SUCCESS:
            print(f"  grab failed frame {i}"); continue
        zed.retrieve_image(left_mat, sl.VIEW.LEFT)
        zed.retrieve_measure(depth_mat, sl.MEASURE.DEPTH)
        raw = depth_mat.get_data().astype(np.float32) / 1000.0  # mm → m
        raw[~np.isfinite(raw)] = np.nan
        depths.append(raw)
        color_bgr = left_mat.get_data()[:, :, :3].copy()

    if not depths:
        return None, color_bgr

    stack = np.stack(depths, axis=0)
    valid_mask = np.isfinite(stack)
    valid_count = valid_mask.sum(axis=0)
    safe_stack = np.where(valid_mask, stack, 0.0)
    fused = (safe_stack.sum(axis=0) / np.maximum(valid_count, 1)).astype(np.float32)
    fused[valid_count == 0] = np.nan
    return fused, color_bgr


# ══════════════════════════════════════════════════════════════════════════════
#  Metrics
# ══════════════════════════════════════════════════════════════════════════════

def compute_metrics(depth_m, color_bgr, roi_rect, fx, fy, cx, cy):
    """
    Returns dict with:
      valid_ratio, depth_noise_mm, actual_z_mm,
      point_density_per_cm2, plane_rmse_mm
    and saves a pointcloud.ply to out_dir.
    """
    H, W = depth_m.shape

    # crop to ROI
    if roi_rect is not None:
        x0, y0, x1, y1 = roi_rect
        depth_roi = depth_m[y0:y1+1, x0:x1+1]
        cx_r, cy_r = cx - x0, cy - y0
    else:
        depth_roi = depth_m
        cx_r, cy_r = cx, cy

    Hr, Wr = depth_roi.shape
    valid_mask = np.isfinite(depth_roi) & (depth_roi > DEPTH_MIN_M) & (depth_roi < DEPTH_MAX_M)

    valid_ratio = valid_mask.mean()
    actual_z_mm = float(np.nanmedian(depth_roi[valid_mask]) * 1000) if valid_mask.any() else float('nan')
    depth_noise_mm = float(np.nanstd(depth_roi[valid_mask]) * 1000) if valid_mask.sum() > 10 else float('nan')

    # back-project valid pixels → 3D
    uu, vv = np.meshgrid(np.arange(Wr), np.arange(Hr))
    zz = depth_roi[valid_mask]
    pts = np.stack([(uu[valid_mask] - cx_r) * zz / fx,
                    (vv[valid_mask] - cy_r) * zz / fy,
                    zz], axis=-1).astype(np.float64)  # (N, 3) in metres

    # point density: points / cm² of image plane projected area
    # ROI projected area at median depth
    if valid_mask.any() and actual_z_mm > 0:
        z_med = actual_z_mm / 1000.0  # m
        roi_w_m = Wr * z_med / fx
        roi_h_m = Hr * z_med / fy
        proj_area_cm2 = roi_w_m * roi_h_m * 1e4
        point_density = len(pts) / proj_area_cm2 if proj_area_cm2 > 0 else float('nan')
    else:
        point_density = float('nan')

    # plane RMSE via SVD (least-squares plane fit)
    plane_rmse_mm = float('nan')
    if len(pts) >= 10:
        centroid = pts.mean(axis=0)
        _, _, Vt = np.linalg.svd(pts - centroid, full_matrices=False)
        normal = Vt[-1]  # normal to best-fit plane
        dists = np.abs((pts - centroid) @ normal)  # perpendicular distances (m)
        plane_rmse_mm = float(np.sqrt((dists ** 2).mean()) * 1000)

    return {
        "valid_ratio":          round(float(valid_ratio), 4),
        "actual_z_mm":          round(actual_z_mm, 1),
        "depth_noise_mm":       round(depth_noise_mm, 3),
        "point_density_per_cm2": round(point_density, 1),
        "plane_rmse_mm":        round(plane_rmse_mm, 3),
        "n_valid_pts":          int(valid_mask.sum()),
    }, pts


# ══════════════════════════════════════════════════════════════════════════════
#  PLY writer (no external deps)
# ══════════════════════════════════════════════════════════════════════════════

def write_ply(path, pts_m, color_bgr, roi_rect, H, W):
    """Write colored point cloud for the ROI."""
    if roi_rect:
        x0, y0, x1, y1 = roi_rect
        color_roi = color_bgr[y0:y1+1, x0:x1+1]
    else:
        color_roi = color_bgr
    Hr, Wr = color_roi.shape[:2]
    color_rgb = cv2.cvtColor(color_roi, cv2.COLOR_BGR2RGB)

    N = len(pts_m)
    if N == 0:
        return

    # pts_m already filtered — we need matching colors
    # We reconstruct which pixels were valid from depth (already computed above)
    # Instead: just use white color as fallback since we'd need the valid_mask here
    # (caller passes pts_m which are back-projected, colors need re-sampling)
    # Simple approach: sample color at reprojected pixel
    fx_approx = 951.9  # rough; not used for accuracy
    colors = np.full((N, 3), 200, dtype=np.uint8)

    dtype = np.dtype([('x','<f4'),('y','<f4'),('z','<f4'),
                      ('r','u1'),('g','u1'),('b','u1')])
    arr = np.empty(N, dtype=dtype)
    arr['x'] = pts_m[:, 0]; arr['y'] = pts_m[:, 1]; arr['z'] = pts_m[:, 2]
    arr['r'] = colors[:, 0]; arr['g'] = colors[:, 1]; arr['b'] = colors[:, 2]
    header = (f"ply\nformat binary_little_endian 1.0\n"
              f"element vertex {N}\n"
              "property float x\nproperty float y\nproperty float z\n"
              "property uchar red\nproperty uchar green\nproperty uchar blue\n"
              "end_header\n")
    with open(path, 'wb') as f:
        f.write(header.encode('ascii'))
        f.write(arr.tobytes())


# ══════════════════════════════════════════════════════════════════════════════
#  Results writer
# ══════════════════════════════════════════════════════════════════════════════

FIELDS = ["nominal_z_mm", "actual_z_mm", "valid_ratio", "depth_noise_mm",
          "point_density_per_cm2", "plane_rmse_mm", "n_valid_pts"]


def write_results(out_dir, rows):
    csv_path = out_dir / "metrics.csv"
    with open(csv_path, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=FIELDS)
        w.writeheader()
        w.writerows(rows)
    print(f"\nCSV → {csv_path}")

    # summary plot
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt

        valid_rows = [r for r in rows if r["actual_z_mm"] == r["actual_z_mm"]]  # not NaN
        if not valid_rows:
            return

        zs     = [r["actual_z_mm"]          for r in valid_rows]
        vr     = [r["valid_ratio"] * 100     for r in valid_rows]
        noise  = [r["depth_noise_mm"]        for r in valid_rows]
        dens   = [r["point_density_per_cm2"] for r in valid_rows]
        rmse   = [r["plane_rmse_mm"]         for r in valid_rows]

        fig, axes = plt.subplots(2, 2, figsize=(10, 8))
        fig.suptitle("ZED2i Point Cloud Quality vs. Capture Distance", fontsize=13)

        axes[0, 0].plot(zs, vr, 'o-', color='tab:blue')
        axes[0, 0].set_xlabel("Distance (mm)"); axes[0, 0].set_ylabel("Valid ratio (%)")
        axes[0, 0].set_title("Valid depth coverage"); axes[0, 0].grid(True)

        axes[0, 1].plot(zs, noise, 's-', color='tab:orange')
        axes[0, 1].set_xlabel("Distance (mm)"); axes[0, 1].set_ylabel("Depth noise σ (mm)")
        axes[0, 1].set_title("Depth noise (flat region)"); axes[0, 1].grid(True)

        axes[1, 0].plot(zs, dens, '^-', color='tab:green')
        axes[1, 0].set_xlabel("Distance (mm)"); axes[1, 0].set_ylabel("pts / cm²")
        axes[1, 0].set_title("Point density"); axes[1, 0].grid(True)

        axes[1, 1].plot(zs, rmse, 'D-', color='tab:red')
        axes[1, 1].set_xlabel("Distance (mm)"); axes[1, 1].set_ylabel("Plane RMSE (mm)")
        axes[1, 1].set_title("Plane fit RMSE (accuracy proxy)"); axes[1, 1].grid(True)

        plt.tight_layout()
        plot_path = out_dir / "metrics_plot.png"
        plt.savefig(str(plot_path), dpi=150)
        plt.close()
        print(f"Plot → {plot_path}")
    except ImportError:
        print("  matplotlib not available — plot skipped")


# ══════════════════════════════════════════════════════════════════════════════
#  Main
# ══════════════════════════════════════════════════════════════════════════════

def main():
    zed  = sl.Camera()
    init = sl.InitParameters()
    init.camera_resolution      = sl.RESOLUTION.HD2K
    init.camera_fps             = 15
    init.depth_mode             = sl.DEPTH_MODE.ULTRA
    init.coordinate_units       = sl.UNIT.MILLIMETER
    init.depth_minimum_distance = 150  # mm

    if zed.open(init) != sl.ERROR_CODE.SUCCESS:
        print("Failed to open ZED camera"); return

    info  = zed.get_camera_information()
    calib = info.camera_configuration.calibration_parameters
    res   = info.camera_configuration.resolution
    W, H  = res.width, res.height
    fx    = calib.left_cam.fx
    fy    = calib.left_cam.fy
    cx    = calib.left_cam.cx
    cy    = calib.left_cam.cy
    B_mm  = calib.get_camera_baseline()

    print(f"ZED 2i  {W}×{H}  fx={fx:.1f}  B={B_mm:.1f}mm")
    print(f"Distances to capture: {DISTANCES_MM} mm")
    print("Controls: drag=ROI  r=resetROI  SPACE=capture  s=skip  q=quit\n")

    out_dir = OUTPUT_ROOT / f"distance_sweep_{time.strftime('%Y%m%d_%H%M%S')}"
    out_dir.mkdir(parents=True, exist_ok=True)

    roi_sel   = ROISelector()
    left_mat  = sl.Mat()
    depth_mat = sl.Mat()
    runtime   = sl.RuntimeParameters()
    WIN      = "demo9 — distance sweep"
    cv2.namedWindow(WIN, cv2.WINDOW_NORMAL)
    cv2.setMouseCallback(WIN, roi_sel.callback, {"scale": DISPLAY_SCALE})

    rows          = []
    dist_idx      = 0
    distances     = list(DISTANCES_MM)

    # Auto-lock state
    z_history      = []       # rolling live_z readings
    locked         = False    # True = stable in range, waiting for Y confirmation
    lock_announced = False    # suppress repeated lock prints for same target
    osd_msg        = ""       # temporary OSD message
    osd_msg_until  = 0.0      # time.time() when to clear it

    while dist_idx < len(distances):
        target_z = distances[dist_idx]

        if zed.grab(runtime) != sl.ERROR_CODE.SUCCESS:
            continue
        zed.retrieve_image(left_mat, sl.VIEW.LEFT)
        zed.retrieve_measure(depth_mat, sl.MEASURE.DEPTH)
        frame     = left_mat.get_data()[:, :, :3].copy()
        raw_depth = depth_mat.get_data().astype(np.float32)  # mm (SDK units)

        display = cv2.resize(frame, (int(W * DISPLAY_SCALE), int(H * DISPLAY_SCALE)))
        rect    = roi_sel.get_rect(W, H)

        # ── draw ROI ────────────────────────────────────────────────────────
        if roi_sel.drawing and roi_sel.start and roi_sel.end:
            ds = (int(roi_sel.start[0] * DISPLAY_SCALE), int(roi_sel.start[1] * DISPLAY_SCALE))
            de = (int(roi_sel.end[0]   * DISPLAY_SCALE), int(roi_sel.end[1]   * DISPLAY_SCALE))
            cv2.rectangle(display, ds, de, (0, 200, 255), 1)
        elif rect:
            x0, y0, x1, y1 = rect
            cv2.rectangle(display,
                          (int(x0 * DISPLAY_SCALE), int(y0 * DISPLAY_SCALE)),
                          (int(x1 * DISPLAY_SCALE), int(y1 * DISPLAY_SCALE)),
                          (0, 255, 0), 2)
        else:
            # no ROI: show crosshair at frame center so user knows what's being sampled
            cdisp = (int(cx * DISPLAY_SCALE), int(cy * DISPLAY_SCALE))
            cv2.drawMarker(display, cdisp, (0, 200, 255),
                           cv2.MARKER_CROSS, 30, 1, cv2.LINE_AA)

        # ── live depth ──────────────────────────────────────────────────────
        if rect:
            rx0, ry0, rx1, ry1 = rect
            patch = raw_depth[ry0:ry1+1, rx0:rx1+1]
            src_label = "ROI-median"
        else:
            cxp, cyp = int(cx), int(cy)
            patch = raw_depth[max(0, cyp-20):cyp+20, max(0, cxp-20):cxp+20]
            src_label = "center-patch"

        valid_d = patch[np.isfinite(patch) & (patch > 0)]
        valid_ratio_live = len(valid_d) / max(patch.size, 1)
        if len(valid_d) > 0 and valid_ratio_live >= LOCK_MIN_VALID:
            live_z_mm = float(np.median(valid_d))
        else:
            live_z_mm = float('nan')

        # ── rolling stability buffer ─────────────────────────────────────────
        if np.isfinite(live_z_mm):
            z_history.append(live_z_mm)
        if len(z_history) > LOCK_WINDOW:
            z_history.pop(0)

        # stability criteria
        in_range  = False
        stable    = False
        lock_pct  = 0
        if len(z_history) >= LOCK_WINDOW // 2:
            win_arr  = np.array(z_history)
            in_range = bool(abs(np.median(win_arr) - target_z) < LOCK_TOL_MM)
            stable   = bool(np.std(win_arr) < LOCK_STD_MM)
            lock_pct = int(min(len(z_history) / LOCK_WINDOW * 100, 100))

        locked = in_range and stable and (len(z_history) >= LOCK_WINDOW)
        if locked and not lock_announced:
            osd_msg       = f"LOCKED at ~{np.median(z_history):.0f}mm — press [Y] to capture"
            osd_msg_until = time.time() + 9999  # stays until dismissed
            print(f"  [AUTO-LOCK] stable at {np.median(z_history):.0f}mm for target {target_z}mm")
            lock_announced = True

        # ── OSD ─────────────────────────────────────────────────────────────
        live_z_str = f"{live_z_mm:.0f}mm" if np.isfinite(live_z_mm) else "---"
        ref_z  = live_z_mm if np.isfinite(live_z_mm) and live_z_mm > 0 else target_z
        d_max_ = int(fx * B_mm / max(ref_z, 1))
        eff_w_ = max(W - d_max_, 0)
        overlap_str = f"{100*eff_w_/W:.0f}%" if eff_w_ > 0 else "NO OVERLAP"

        line1 = (f"[{dist_idx+1}/{len(distances)}] target={target_z}mm  "
                 f"live_z={live_z_str} ({src_label})  overlap~{overlap_str}")
        # stability bar:  [####......] 40%  in_range=T stable=T
        bar_len = 16
        filled  = int(lock_pct / 100 * bar_len)
        bar_str = '[' + '#'*filled + '.'*(bar_len-filled) + f"] {lock_pct}%"
        lock_color = (0, 255, 80) if locked else ((0, 200, 255) if in_range else (80, 80, 255))
        line2 = f"stability {bar_str}  in_range={'Y' if in_range else 'N'}  stable={'Y' if stable else 'N'}"

        roi_hint = ("ROI=set (flat-surface metrics)" if rect
                    else "ROI=none (drag to select flat surface for metrics)")
        line3 = f"{roi_hint}   [r]=clear ROI"
        line4 = "[Y/SPACE]=capture  [s]=skip  [q]=finish"

        cv2.putText(display, line1, (8, 22),  cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1)
        cv2.putText(display, line2, (8, 42),  cv2.FONT_HERSHEY_SIMPLEX, 0.45, lock_color,    1)
        cv2.putText(display, line3, (8, 62),  cv2.FONT_HERSHEY_SIMPLEX, 0.40, (180, 180, 180), 1)
        cv2.putText(display, line4, (8, 80),  cv2.FONT_HERSHEY_SIMPLEX, 0.40, (200, 200, 200), 1)

        # temporary OSD message (ROI cleared / locked alert)
        if osd_msg and time.time() < osd_msg_until:
            cv2.putText(display, osd_msg, (8, int(H * DISPLAY_SCALE) - 12),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                        (0, 255, 80) if locked else (0, 220, 255), 2)

        cv2.imshow(WIN, display)
        key = cv2.waitKey(1) & 0xFF

        if key == ord('q'):
            print("Quit — writing results so far.")
            break
        elif key == ord('r'):
            roi_sel.reset()
            osd_msg       = "ROI cleared — drag to set a new one"
            osd_msg_until = time.time() + 2.5
        elif key == ord('s'):
            print(f"  Skipped {target_z}mm")
            row = {f: float('nan') for f in FIELDS}
            row["nominal_z_mm"] = target_z
            rows.append(row)
            dist_idx += 1
            z_history.clear()
            locked = False
            lock_announced = False
            osd_msg = ""
            roi_sel.reset()
        elif key in (ord('y'), ord('Y'), ord(' ')):
            if key == ord(' ') and not locked:
                # force capture even if not locked
                pass
            elif key in (ord('y'), ord('Y')) and not locked:
                osd_msg       = "Not locked yet — move closer to target distance"
                osd_msg_until = time.time() + 2.0
                continue

            print(f"\n{'='*60}")
            print(f"Capturing  target={target_z}mm  live={live_z_str}  ({N_FRAMES} frames)")
            depth_fused, color_bgr = grab_frames(zed, N_FRAMES)

            if depth_fused is None:
                print("  ERROR: capture failed"); continue

            cur_rect = roi_sel.get_rect(W, H)
            metrics, pts = compute_metrics(depth_fused, color_bgr, cur_rect, fx, fy, cx, cy)
            metrics["nominal_z_mm"] = target_z

            print(f"  actual_z      = {metrics['actual_z_mm']:.1f} mm")
            print(f"  valid_ratio   = {metrics['valid_ratio']*100:.1f}%")
            print(f"  depth_noise   = {metrics['depth_noise_mm']:.3f} mm")
            print(f"  point_density = {metrics['point_density_per_cm2']:.1f} pts/cm²")
            print(f"  plane_RMSE    = {metrics['plane_rmse_mm']:.3f} mm")
            print(f"  n_valid_pts   = {metrics['n_valid_pts']:,}")

            tag = f"z{target_z:04d}mm"
            np.save(str(out_dir / f"{tag}_depth.npy"), depth_fused)
            cv2.imwrite(str(out_dir / f"{tag}_color.png"), color_bgr)
            ply_path = out_dir / f"{tag}_cloud.ply"
            write_ply(str(ply_path), pts, color_bgr, cur_rect, H, W)
            print(f"  saved → {tag}_depth.npy / _color.png / _cloud.ply")

            row = {f: metrics.get(f, float('nan')) for f in FIELDS}
            rows.append(row)
            dist_idx += 1
            z_history.clear()
            locked = False
            lock_announced = False
            osd_msg = ""
            roi_sel.reset()

    cv2.destroyAllWindows()
    zed.close()

    if rows:
        write_results(out_dir, rows)
        print(f"\n{'='*60}")
        print("Summary:")
        print(f"  {'Nominal Z':>10}  {'Actual Z':>10}  {'Valid%':>8}  "
              f"{'Noise mm':>10}  {'pts/cm²':>10}  {'RMSE mm':>10}")
        print(f"  {'-'*62}")
        for r in rows:
            vr  = f"{r['valid_ratio']*100:.1f}" if r['valid_ratio'] == r['valid_ratio'] else "NaN"
            nz  = f"{r['actual_z_mm']:.0f}"     if r['actual_z_mm'] == r['actual_z_mm'] else "NaN"
            nm  = f"{r['depth_noise_mm']:.3f}"  if r['depth_noise_mm'] == r['depth_noise_mm'] else "NaN"
            dc  = f"{r['point_density_per_cm2']:.1f}" if r['point_density_per_cm2'] == r['point_density_per_cm2'] else "NaN"
            rm  = f"{r['plane_rmse_mm']:.3f}"   if r['plane_rmse_mm'] == r['plane_rmse_mm'] else "NaN"
            print(f"  {int(r['nominal_z_mm']):>10}  {nz:>10}  {vr:>8}  {nm:>10}  {dc:>10}  {rm:>10}")
        print(f"\nOutput dir: {out_dir}")


if __name__ == "__main__":
    main()
