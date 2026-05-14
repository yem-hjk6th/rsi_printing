"""
capture.py — Keyframe capture module for artec_ffs pipeline.

Same as artec_imit/capture.py with one addition:
  - grab_fused() also retrieves the right image
  - save_keyframe() saves view_NNN_right.png alongside left/depth/ply
    so ffs_depth.py can later replace ZED depth with FFS depth

Output per keyframe:
  view_NNN.ply              ROI-cropped point cloud (from ZED depth, for registration init)
  view_NNN_depth.npy        float32 depth, meters, NaN outside ROI  (ZED ULTRA)
  view_NNN_color.png        left BGR image (full frame)
  view_NNN_right.png        right BGR image (full frame) ← NEW for FFS
  capture_meta.json         intrinsics + capture params

Usage:
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
    """
    Mean-fuse N frames.
    Returns (depth_m float32, color_bgr uint8, right_bgr uint8).
    right_bgr is the last grabbed right frame (not fused — used for FFS stereo matching).
    """
    left_mat  = sl.Mat()
    right_mat = sl.Mat()
    depth_mat = sl.Mat()
    runtime   = sl.RuntimeParameters()
    for _ in range(settle_skip):
        zed.grab(runtime)
    depths, color_bgr, right_bgr = [], None, None
    for _ in range(n_frames):
        if zed.grab(runtime) != sl.ERROR_CODE.SUCCESS:
            continue
        zed.retrieve_image(left_mat,  sl.VIEW.LEFT)
        zed.retrieve_image(right_mat, sl.VIEW.RIGHT)
        zed.retrieve_measure(depth_mat, sl.MEASURE.DEPTH)
        raw = depth_mat.get_data().astype(np.float32) / 1000.0
        raw[~np.isfinite(raw)] = np.nan
        depths.append(raw)
        color_bgr = left_mat.get_data()[:, :, :3].copy()
        right_bgr = right_mat.get_data()[:, :, :3].copy()
    if not depths:
        return None, color_bgr, right_bgr
    stack = np.stack(depths, axis=0)
    valid = np.isfinite(stack)
    count = valid.sum(axis=0)
    fused = (np.where(valid, stack, 0.0).sum(axis=0) / np.maximum(count, 1)).astype(np.float32)
    fused[count == 0] = np.nan
    return fused, color_bgr, right_bgr


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
    H, W = color_bgr.shape[:2]
    disp = cv2.resize(color_bgr, (int(W * scale), int(H * scale)))
    cv2.putText(disp, f"View {view_idx}: drag ROI — ENTER=confirm, C=full frame",
                (8, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.50, (0, 255, 255), 1)
    win = "Select ROI"
    r = cv2.selectROI(win, disp, fromCenter=False, showCrosshair=True)
    cv2.destroyWindow(win)
    if r[2] < 10 or r[3] < 10:
        return None
    s = 1.0 / scale
    x1, y1 = int(r[0] * s), int(r[1] * s)
    x2, y2 = int((r[0] + r[2]) * s), int((r[1] + r[3]) * s)
    mask = np.zeros((H, W), dtype=bool)
    mask[max(0, y1):min(H, y2), max(0, x1):min(W, x2)] = True
    return mask


def auto_roi_depth(depth_m):
    valid = np.isfinite(depth_m) & (depth_m > DEPTH_MIN_M) & (depth_m < DEPTH_MAX_M)
    if valid.sum() < 1000:
        return None
    valid_u8 = valid.astype(np.uint8) * 255
    kernel   = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (15, 15))
    cleaned  = cv2.morphologyEx(valid_u8, cv2.MORPH_CLOSE, kernel)
    cleaned  = cv2.morphologyEx(cleaned,  cv2.MORPH_OPEN,  kernel)
    n_labels, labels, stats, _ = cv2.connectedComponentsWithStats(cleaned, connectivity=8)
    if n_labels < 2:
        return valid
    largest = 1 + np.argmax(stats[1:, cv2.CC_STAT_AREA])
    return (labels == largest)


def depth_to_pcd(depth_m, color_bgr, intr, roi_mask=None):
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
    if roi_mask is None:
        return depth_m.copy()
    masked = depth_m.copy()
    masked[~roi_mask] = np.nan
    return masked


# ══════════════════════════════════════════════════════════════════════════════
#  Save one keyframe
# ══════════════════════════════════════════════════════════════════════════════

def save_keyframe(out_dir, idx, depth_m, color_bgr, roi_mask, right_bgr=None):
    """
    Save depth.npy (ROI-masked), color.png, right.png (for FFS), PLY.
    right.png is the raw right frame saved for offline FFS depth refinement.
    Returns pcd.
    """
    import open3d as o3d
    depth_masked = apply_roi_to_depth(depth_m, roi_mask)
    np.save(str(out_dir / f"view_{idx:03d}_depth.npy"), depth_masked)
    cv2.imwrite(str(out_dir / f"view_{idx:03d}_color.png"), color_bgr)
    if right_bgr is not None:
        cv2.imwrite(str(out_dir / f"view_{idx:03d}_right.png"), right_bgr)
    pcd = depth_to_pcd(depth_m, color_bgr, _INTR_CACHE, roi_mask)
    o3d.io.write_point_cloud(str(out_dir / f"view_{idx:03d}.ply"), pcd)
    return pcd


_INTR_CACHE = None


# ══════════════════════════════════════════════════════════════════════════════
#  Auto-trigger: motion estimation
# ══════════════════════════════════════════════════════════════════════════════

def _estimate_motion(pcd_prev, pcd_curr):
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
    global _INTR_CACHE

    zed, intr = open_zed()
    _INTR_CACHE = intr
    W, H = intr["W"], intr["H"]
    fx, fy, cx, cy = intr["fx"], intr["fy"], intr["cx"], intr["cy"]

    if out_dir is None:
        out_dir = OUTPUT_ROOT / f"ffs_{time.strftime('%Y%m%d_%H%M%S')}"
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    WIN = f"artec_ffs capture ({mode})"
    cv2.namedWindow(WIN, cv2.WINDOW_NORMAL)

    runtime   = sl.RuntimeParameters()
    left_mat  = sl.Mat()
    depth_mat = sl.Mat()

    view_idx      = 0
    prev_pcd_auto = None
    saved_files   = []

    print(f"Capture mode: {mode}")
    if mode == "manual":
        print("Controls: [SPACE]=capture  [u]=undo  [q]=finish")
    else:
        print(f"Auto trigger: trans>{AUTO_TRANS_M*1000:.0f}mm or rot>{AUTO_ROT_DEG:.0f}°")
        print("Controls: [SPACE]=confirm first frame  [u]=undo  [q]=finish")

    while view_idx < MAX_VIEWS:
        if zed.grab(runtime) != sl.ERROR_CODE.SUCCESS:
            continue

        zed.retrieve_image(left_mat, sl.VIEW.LEFT)
        zed.retrieve_measure(depth_mat, sl.MEASURE.DEPTH)
        frame     = left_mat.get_data()[:, :, :3].copy()
        raw_depth = depth_mat.get_data().astype(np.float32) / 1000.0
        raw_depth[~np.isfinite(raw_depth)] = np.nan

        disp = cv2.resize(frame, (int(W * DISPLAY_SCALE), int(H * DISPLAY_SCALE)))
        patch  = raw_depth[max(0, int(cy)-20):int(cy)+20, max(0, int(cx)-20):int(cx)+20]
        valid  = patch[np.isfinite(patch) & (patch > 0)]
        live_z = f"{float(np.median(valid))*1000:.0f}mm" if len(valid) > 0 else "---"

        cv2.drawMarker(disp, (int(cx * DISPLAY_SCALE), int(cy * DISPLAY_SCALE)),
                       (0, 200, 255), cv2.MARKER_CROSS, 30, 1, cv2.LINE_AA)
        if mode == "auto" and view_idx == 0:
            hint = "[SPACE]=confirm first frame (aim at target first!)"
            hint_col = (0, 100, 255)
        elif mode == "auto":
            hint = f"auto  trans>{AUTO_TRANS_M*1000:.0f}mm|rot>{AUTO_ROT_DEG:.0f}deg  [SPACE]=force  [u]=undo"
            hint_col = (200, 200, 200)
        else:
            hint = "[SPACE]=capture  [u]=undo  [q]=finish"
            hint_col = (200, 200, 200)
        cv2.putText(disp, f"views:{view_idx}  z={live_z}  mode={mode}  [ffs]",
                    (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (0, 200, 255), 1)
        cv2.putText(disp, hint,
                    (8, 42), cv2.FONT_HERSHEY_SIMPLEX, 0.38, hint_col, 1)
        bar = "[" + "#" * view_idx + "." * max(0, 8 - view_idx) + f"] {view_idx}"
        cv2.putText(disp, bar, (8, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (0, 255, 80), 1)
        cv2.imshow(WIN, disp)
        key = cv2.waitKey(1) & 0xFF

        if key == ord('q'):
            print("Finishing."); break

        elif key == ord('u') and view_idx > 0:
            view_idx -= 1
            for suffix in [".ply", "_depth.npy", "_color.png", "_right.png"]:
                p = out_dir / f"view_{view_idx:03d}{suffix}"
                if p.exists(): p.unlink()
            print(f"  Undid view {view_idx}")
            saved_files   = [f for f in saved_files if f"view_{view_idx:03d}" not in str(f)]
            prev_pcd_auto = None
            continue

        trigger = (key == ord(' '))
        if mode == "auto" and not trigger and view_idx > 0:
            # auto-trigger only after first frame is confirmed by SPACE
            curr_pcd = depth_to_pcd(raw_depth, frame, intr,
                                    roi_mask=auto_roi_depth(raw_depth))
            trans, rot = _estimate_motion(prev_pcd_auto, curr_pcd)
            if trans > AUTO_TRANS_M or rot > AUTO_ROT_DEG:
                trigger = True
                print(f"  Auto-trigger: trans={trans*1000:.0f}mm rot={rot:.1f}°")

        if not trigger:
            continue

        print(f"\n--- Capturing view {view_idx} ---")
        depth_fused, color_bgr, right_bgr = grab_fused(zed)
        if depth_fused is None:
            print("  ERROR: capture failed"); continue

        if mode == "manual":
            roi_mask = select_roi_manual(color_bgr, view_idx)
        else:
            roi_mask = auto_roi_depth(depth_fused)

        if roi_mask is not None:
            print(f"  ROI pixels: {roi_mask.sum():,}")
        else:
            print("  No ROI — full frame")

        pcd = save_keyframe(out_dir, view_idx, depth_fused, color_bgr, roi_mask,
                            right_bgr=right_bgr)
        n_pts = len(pcd.points)
        if n_pts < 500:
            print(f"  Too few points ({n_pts}) — skip"); continue

        print(f"  view_{view_idx:03d}  {n_pts:,} pts  right={'saved' if right_bgr is not None else 'none'}")
        prev_pcd_auto = pcd
        view_idx += 1

    cv2.destroyAllWindows()
    zed.close()

    if view_idx == 0:
        print("No views captured."); return out_dir

    has_right = (out_dir / "view_000_right.png").exists()
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
        "roi_in_depth":   True,
        "has_right_png":  has_right,
    }
    (out_dir / "capture_meta.json").write_text(json.dumps(meta, indent=2))
    print(f"\nCapture complete: {view_idx} views → {out_dir}")
    if has_right:
        print("  right images saved — run pipeline.py recon --ffs to use FFS depth")
    return out_dir


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--mode", choices=["manual", "auto"], default="manual")
    parser.add_argument("--out",  type=str, default=None)
    args = parser.parse_args()
    capture_session(mode=args.mode, out_dir=args.out)
