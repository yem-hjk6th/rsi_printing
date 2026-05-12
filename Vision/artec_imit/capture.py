"""
capture.py — Keyframe capture module for artec_imit pipeline.

Supports two modes (--capture-mode auto|manual), both producing the same
output format consumed by register.py / fuse.py downstream.

Manual mode:  [SPACE] triggers keyframe + ROI selectROI dialog
Auto mode:    frames are captured when camera motion exceeds translation/rotation thresholds
              (requires tracking — falls back to manual if ZED positional tracking unavailable)

Output per keyframe (saved to <out_dir>/view_NNN_*):
  view_NNN.ply              ROI-cropped point cloud with RGB (for registration)
  view_NNN_depth.npy        float32 depth map, meters, NaN outside ROI
  view_NNN_color.png        BGR color image, full frame
  capture_meta.json         intrinsics + capture params

Usage (standalone):
    python capture.py [--mode manual|auto] [--out <dir>]
"""

import sys, os, time, json, argparse
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
import zed_setup  # noqa
import pyzed.sl as sl

from config import (
    OUTPUT_ROOT, N_FRAMES_FUSE, SETTLE_SKIP,
    DEPTH_MIN_M, DEPTH_MAX_M, MAX_VIEWS, DISPLAY_SCALE,
    AUTO_TRANS_M, AUTO_ROT_DEG,
)


# ══════════════════════════════════════════════════════════════════════════════
#  ZED helpers
# ══════════════════════════════════════════════════════════════════════════════

def open_zed():
    zed  = sl.Camera()
    init = sl.InitParameters()
    init.camera_resolution      = sl.RESOLUTION.HD2K
    init.camera_fps             = 15
    init.depth_mode             = sl.DEPTH_MODE.ULTRA
    init.coordinate_units       = sl.UNIT.MILLIMETER
    init.depth_minimum_distance = 300
    if zed.open(init) != sl.ERROR_CODE.SUCCESS:
        raise RuntimeError("Failed to open ZED camera")
    info  = zed.get_camera_information()
    calib = info.camera_configuration.calibration_parameters
    res   = info.camera_configuration.resolution
    intr  = {
        "fx": calib.left_cam.fx, "fy": calib.left_cam.fy,
        "cx": calib.left_cam.cx, "cy": calib.left_cam.cy,
        "W":  res.width,         "H":  res.height,
        "baseline_mm": calib.get_camera_baseline(),
    }
    print(f"ZED 2i  {intr['W']}×{intr['H']}  fx={intr['fx']:.1f}  B={intr['baseline_mm']:.1f}mm")
    return zed, intr


def grab_fused(zed, n_frames=N_FRAMES_FUSE, settle_skip=SETTLE_SKIP):
    """Mean-fuse N frames → (depth_m float32, color_bgr uint8)."""
    left_mat  = sl.Mat()
    depth_mat = sl.Mat()
    runtime   = sl.RuntimeParameters()
    for _ in range(settle_skip):
        zed.grab(runtime)
    depths, color_bgr = [], None
    for _ in range(n_frames):
        if zed.grab(runtime) != sl.ERROR_CODE.SUCCESS:
            continue
        zed.retrieve_image(left_mat,  sl.VIEW.LEFT)
        zed.retrieve_measure(depth_mat, sl.MEASURE.DEPTH)
        raw = depth_mat.get_data().astype(np.float32) / 1000.0
        raw[~np.isfinite(raw)] = np.nan
        depths.append(raw)
        color_bgr = left_mat.get_data()[:, :, :3].copy()
    if not depths:
        return None, color_bgr
    stack = np.stack(depths, axis=0)
    valid = np.isfinite(stack)
    count = valid.sum(axis=0)
    fused = (np.where(valid, stack, 0.0).sum(axis=0) / np.maximum(count, 1)).astype(np.float32)
    fused[count == 0] = np.nan
    return fused, color_bgr


def get_live_frame(zed):
    """Single grab for display / auto-trigger tracking."""
    left_mat  = sl.Mat()
    depth_mat = sl.Mat()
    runtime   = sl.RuntimeParameters()
    if zed.grab(runtime) != sl.ERROR_CODE.SUCCESS:
        return None, None
    zed.retrieve_image(left_mat, sl.VIEW.LEFT)
    zed.retrieve_measure(depth_mat, sl.MEASURE.DEPTH)
    color = left_mat.get_data()[:, :, :3].copy()
    depth = depth_mat.get_data().astype(np.float32) / 1000.0
    depth[~np.isfinite(depth)] = np.nan
    return color, depth


# ══════════════════════════════════════════════════════════════════════════════
#  ROI & point cloud helpers
# ══════════════════════════════════════════════════════════════════════════════

def select_roi_manual(color_bgr, view_idx, scale=DISPLAY_SCALE):
    """Show ROI dialog, return pixel-precise boolean mask (H×W)."""
    H, W = color_bgr.shape[:2]
    disp = cv2.resize(color_bgr, (int(W * scale), int(H * scale)))
    cv2.putText(disp, f"View {view_idx}: drag ROI — ENTER=confirm, C=full frame",
                (8, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.50, (0, 255, 255), 1)
    win = "Select ROI"
    r = cv2.selectROI(win, disp, fromCenter=False, showCrosshair=True)
    cv2.destroyWindow(win)
    if r[2] < 10 or r[3] < 10:
        return None  # full frame
    s = 1.0 / scale
    x1, y1 = int(r[0] * s), int(r[1] * s)
    x2, y2 = int((r[0] + r[2]) * s), int((r[1] + r[3]) * s)
    mask = np.zeros((H, W), dtype=bool)
    mask[max(0, y1):min(H, y2), max(0, x1):min(W, x2)] = True
    return mask


def auto_roi_depth(depth_m):
    """
    Automatic ROI via depth-range segmentation.
    Assumes object is at DEPTH_MIN_M..DEPTH_MAX_M and table/background is farther.
    Returns boolean mask (H×W).
    """
    valid = np.isfinite(depth_m) & (depth_m > DEPTH_MIN_M) & (depth_m < DEPTH_MAX_M)
    if valid.sum() < 1000:
        return None
    # Morphological clean-up: remove isolated noise pixels
    valid_u8 = valid.astype(np.uint8) * 255
    kernel   = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (15, 15))
    cleaned  = cv2.morphologyEx(valid_u8, cv2.MORPH_CLOSE, kernel)
    cleaned  = cv2.morphologyEx(cleaned,  cv2.MORPH_OPEN,  kernel)
    # Keep only largest connected component
    n_labels, labels, stats, _ = cv2.connectedComponentsWithStats(cleaned, connectivity=8)
    if n_labels < 2:
        return valid
    largest = 1 + np.argmax(stats[1:, cv2.CC_STAT_AREA])
    mask = (labels == largest)
    return mask


def depth_to_pcd(depth_m, color_bgr, intr, roi_mask=None):
    """Back-project depth → open3d PointCloud with RGB, using ROI mask."""
    import open3d as o3d
    H, W  = depth_m.shape
    fx, fy, cx, cy = intr["fx"], intr["fy"], intr["cx"], intr["cy"]
    valid = np.isfinite(depth_m) & (depth_m > DEPTH_MIN_M) & (depth_m < DEPTH_MAX_M)
    if roi_mask is not None:
        valid = valid & roi_mask
    uu, vv = np.meshgrid(np.arange(W), np.arange(H))
    zz  = depth_m[valid]
    pts = np.stack([(uu[valid] - cx) * zz / fx,
                    (vv[valid] - cy) * zz / fy,
                    zz], axis=-1).astype(np.float64)
    cols = cv2.cvtColor(color_bgr, cv2.COLOR_BGR2RGB)[valid].astype(np.float64) / 255.0
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pts)
    pcd.colors = o3d.utility.Vector3dVector(cols)
    return pcd


def apply_roi_to_depth(depth_m, roi_mask):
    """Return a copy of depth_m with pixels outside ROI set to NaN."""
    if roi_mask is None:
        return depth_m.copy()
    masked = depth_m.copy()
    masked[~roi_mask] = np.nan
    return masked


# ══════════════════════════════════════════════════════════════════════════════
#  Save one keyframe
# ══════════════════════════════════════════════════════════════════════════════

def save_keyframe(out_dir, idx, depth_m, color_bgr, roi_mask):
    """
    Save depth.npy (ROI-masked), color.png (full frame), PLY (ROI points).
    Returns (depth_path, color_path, ply_path).
    """
    import open3d as o3d
    depth_masked = apply_roi_to_depth(depth_m, roi_mask)
    np.save(str(out_dir / f"view_{idx:03d}_depth.npy"), depth_masked)
    cv2.imwrite(str(out_dir / f"view_{idx:03d}_color.png"), color_bgr)
    pcd = depth_to_pcd(depth_m, color_bgr, _INTR_CACHE, roi_mask)
    o3d.io.write_point_cloud(str(out_dir / f"view_{idx:03d}.ply"), pcd)
    return pcd


_INTR_CACHE = None  # set by capture_session()


# ══════════════════════════════════════════════════════════════════════════════
#  Auto-trigger: motion estimation from consecutive frames
# ══════════════════════════════════════════════════════════════════════════════

def _estimate_motion(pcd_prev, pcd_curr):
    """
    Rough frame-to-frame motion estimate via Geometric ICP.
    Returns (trans_m, rot_deg) or (0, 0) on failure.
    """
    import open3d as o3d
    if pcd_prev is None or len(pcd_prev.points) < 500:
        return 999.0, 999.0
    vox = 0.010
    s = pcd_prev.voxel_down_sample(vox)
    t = pcd_curr.voxel_down_sample(vox)
    for pc in [s, t]:
        pc.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=0.02, max_nn=20))
    res = o3d.pipelines.registration.registration_icp(
        s, t, max_correspondence_distance=0.05,
        estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPlane(),
        criteria=o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=30),
    )
    T = res.transformation
    trans = np.linalg.norm(T[:3, 3])
    cos_a = (np.trace(T[:3, :3]) - 1) / 2
    rot   = np.degrees(np.arccos(float(np.clip(cos_a, -1, 1))))
    return trans, rot


# ══════════════════════════════════════════════════════════════════════════════
#  Capture session
# ══════════════════════════════════════════════════════════════════════════════

def capture_session(mode="manual", out_dir=None):
    """
    Run a full capture session.

    mode: "manual" — press SPACE to trigger, ROI dialog per keyframe
          "auto"   — auto-trigger on motion threshold, auto depth-range ROI

    Returns out_dir (Path) containing all keyframe files + capture_meta.json.
    """
    global _INTR_CACHE

    zed, intr = open_zed()
    _INTR_CACHE = intr
    W, H = intr["W"], intr["H"]
    fx, fy, cx, cy = intr["fx"], intr["fy"], intr["cx"], intr["cy"]

    if out_dir is None:
        out_dir = OUTPUT_ROOT / f"imit_{time.strftime('%Y%m%d_%H%M%S')}"
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    WIN = f"artec_imit capture ({mode})"
    cv2.namedWindow(WIN, cv2.WINDOW_NORMAL)

    runtime   = sl.RuntimeParameters()
    left_mat  = sl.Mat()
    depth_mat = sl.Mat()

    view_idx       = 0
    prev_pcd_auto  = None   # for auto-trigger motion estimation
    saved_files    = []

    print(f"Capture mode: {mode}")
    if mode == "manual":
        print("Controls: [SPACE]=capture  [u]=undo  [q]=finish")
    else:
        print(f"Auto trigger: trans>{AUTO_TRANS_M*1000:.0f}mm or rot>{AUTO_ROT_DEG:.0f}°")
        print("Controls: [u]=undo  [q]=finish  [SPACE]=force capture")

    while view_idx < MAX_VIEWS:
        if zed.grab(runtime) != sl.ERROR_CODE.SUCCESS:
            continue

        zed.retrieve_image(left_mat, sl.VIEW.LEFT)
        zed.retrieve_measure(depth_mat, sl.MEASURE.DEPTH)
        frame     = left_mat.get_data()[:, :, :3].copy()
        raw_depth = depth_mat.get_data().astype(np.float32) / 1000.0
        raw_depth[~np.isfinite(raw_depth)] = np.nan

        disp = cv2.resize(frame, (int(W * DISPLAY_SCALE), int(H * DISPLAY_SCALE)))

        # live center depth OSD
        patch  = raw_depth[max(0, int(cy)-20):int(cy)+20, max(0, int(cx)-20):int(cx)+20]
        valid  = patch[np.isfinite(patch) & (patch > 0)]
        live_z = f"{float(np.median(valid))*1000:.0f}mm" if len(valid) > 0 else "---"

        cv2.drawMarker(disp, (int(cx * DISPLAY_SCALE), int(cy * DISPLAY_SCALE)),
                       (0, 200, 255), cv2.MARKER_CROSS, 30, 1, cv2.LINE_AA)
        cv2.putText(disp, f"views:{view_idx}  z={live_z}  mode={mode}",
                    (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (0, 255, 255), 1)
        cv2.putText(disp, "[SPACE]=capture  [u]=undo  [q]=finish",
                    (8, 42), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (200, 200, 200), 1)
        bar = "[" + "#" * view_idx + "." * max(0, 8 - view_idx) + f"] {view_idx}"
        cv2.putText(disp, bar, (8, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (0, 255, 80), 1)
        cv2.imshow(WIN, disp)
        key = cv2.waitKey(1) & 0xFF

        if key == ord('q'):
            print("Finishing."); break

        elif key == ord('u') and view_idx > 0:
            view_idx -= 1
            for suffix in [".ply", "_depth.npy", "_color.png"]:
                p = out_dir / f"view_{view_idx:03d}{suffix}"
                if p.exists(): p.unlink()
            print(f"  Undid view {view_idx}")
            saved_files = [f for f in saved_files if f"view_{view_idx:03d}" not in str(f)]
            prev_pcd_auto = None
            continue

        # ── Decide whether to trigger ─────────────────────────────────────
        trigger = (key == ord(' '))

        if mode == "auto" and not trigger:
            curr_pcd = depth_to_pcd(raw_depth, frame, intr,
                                    roi_mask=auto_roi_depth(raw_depth))
            trans, rot = _estimate_motion(prev_pcd_auto, curr_pcd)
            if trans > AUTO_TRANS_M or rot > AUTO_ROT_DEG:
                trigger = True
                print(f"  Auto-trigger: trans={trans*1000:.0f}mm rot={rot:.1f}°")

        if not trigger:
            continue

        # ── Capture keyframe ──────────────────────────────────────────────
        print(f"\n--- Capturing view {view_idx} ---")
        depth_fused, color_bgr = grab_fused(zed)
        if depth_fused is None:
            print("  ERROR: capture failed"); continue

        # ROI
        if mode == "manual":
            roi_mask = select_roi_manual(color_bgr, view_idx)
        else:
            roi_mask = auto_roi_depth(depth_fused)

        if roi_mask is not None:
            n_roi = roi_mask.sum()
            print(f"  ROI pixels: {n_roi:,}")
        else:
            print("  No ROI — full frame")

        pcd = save_keyframe(out_dir, view_idx, depth_fused, color_bgr, roi_mask)
        n_pts = len(pcd.points)
        if n_pts < 500:
            print(f"  Too few points ({n_pts}) — skip"); continue

        print(f"  view_{view_idx:03d}  {n_pts:,} pts")
        prev_pcd_auto = pcd
        view_idx += 1

    cv2.destroyAllWindows()
    zed.close()

    if view_idx == 0:
        print("No views captured."); return out_dir

    meta = {
        "timestamp":      time.strftime('%Y%m%d_%H%M%S'),
        "capture_mode":   mode,
        "n_views":        view_idx,
        "camera":         "ZED 2i",
        "resolution":     [W, H],
        "intrinsics":     {"fx": intr["fx"], "fy": intr["fy"],
                           "cx": intr["cx"], "cy": intr["cy"]},
        "baseline_mm":    intr["baseline_mm"],
        "depth_min_m":    DEPTH_MIN_M,
        "depth_max_m":    DEPTH_MAX_M,
        "n_frames_fused": N_FRAMES_FUSE,
        "depth_unit":     "meters",
        "roi_in_depth":   True,   # depth.npy has NaN outside ROI
    }
    (out_dir / "capture_meta.json").write_text(json.dumps(meta, indent=2))
    print(f"\nCapture complete: {view_idx} views → {out_dir}")
    return out_dir


# ══════════════════════════════════════════════════════════════════════════════
#  Standalone entry
# ══════════════════════════════════════════════════════════════════════════════

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="artec_imit capture module")
    parser.add_argument("--mode", choices=["manual", "auto"], default="manual")
    parser.add_argument("--out",  type=str, default=None,
                        help="Output directory (default: auto-timestamped)")
    args = parser.parse_args()
    capture_session(mode=args.mode, out_dir=args.out)
