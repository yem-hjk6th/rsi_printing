"""
multiview_capture.py — Handheld multi-view point cloud capture for ZED 2i.

Captures N viewpoints and saves each as a PLY (with RGB colors) + metadata JSON.
Does NO registration — use a separate reconstruction script on the saved PLYs.

Recommended:
  - Working distance 800–1200 mm (from distance sweep experiments)
  - Overlap between adjacent views: ~30–50% of FOV
  - Move camera ~30–45° between positions; keep object still
  - Use ROI selection to exclude background (critical for ICP quality)

Controls:
  [SPACE]  : capture current view (opens ROI selector on frozen frame)
  [u]      : undo last captured view
  [q]      : finish and save

Output (vision_demo_test_res/multiview_<timestamp>/):
  view_000.ply, view_001.ply, ...   per-view point clouds (camera-space, with RGB)
  capture_meta.json                 camera intrinsics + capture params
"""

import sys, os, time, json
from pathlib import Path

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
import zed_setup  # noqa: E402
import pyzed.sl as sl
import cv2
import numpy as np

# ── Config ────────────────────────────────────────────────────────────────────
N_FRAMES        = 15       # frames to mean-fuse per viewpoint
SETTLE_SKIP     = 6        # frames to skip for auto-exposure
DEPTH_MIN_M     = 0.40     # ignore depth closer than this (m)
DEPTH_MAX_M     = 2.00     # ignore depth farther than this (m)
DISPLAY_SCALE   = 0.5
MAX_VIEWS       = 20

SCRIPT_DIR  = Path(__file__).resolve().parent
VISION_DIR  = SCRIPT_DIR.parent
OUTPUT_ROOT = VISION_DIR / "vision_demo_test_res"


# ══════════════════════════════════════════════════════════════════════════════
#  Capture helpers
# ══════════════════════════════════════════════════════════════════════════════

def grab_frames(zed, n_frames=N_FRAMES, settle_skip=SETTLE_SKIP):
    """Grab N frames with auto-exposure settle, return mean-fused depth + last color."""
    left_mat  = sl.Mat()
    depth_mat = sl.Mat()
    runtime   = sl.RuntimeParameters()

    for _ in range(settle_skip):
        zed.grab(runtime)

    depths    = []
    color_bgr = None
    for i in range(n_frames):
        if zed.grab(runtime) != sl.ERROR_CODE.SUCCESS:
            print(f"  grab failed frame {i}"); continue
        zed.retrieve_image(left_mat,  sl.VIEW.LEFT)
        zed.retrieve_measure(depth_mat, sl.MEASURE.DEPTH)
        raw = depth_mat.get_data().astype(np.float32) / 1000.0   # mm → m
        raw[~np.isfinite(raw)] = np.nan
        depths.append(raw)
        color_bgr = left_mat.get_data()[:, :, :3].copy()

    if not depths:
        return None, color_bgr
    stack       = np.stack(depths, axis=0)
    valid_mask  = np.isfinite(stack)
    valid_count = valid_mask.sum(axis=0)
    safe_stack  = np.where(valid_mask, stack, 0.0)
    fused       = (safe_stack.sum(axis=0) / np.maximum(valid_count, 1)).astype(np.float32)
    fused[valid_count == 0] = np.nan
    return fused, color_bgr


def depth_to_pointcloud(depth_m, color_bgr, fx, fy, cx, cy,
                        depth_min=DEPTH_MIN_M, depth_max=DEPTH_MAX_M, roi=None):
    """Back-project depth map → open3d PointCloud with RGB colors.
    roi: (x1, y1, x2, y2) in full-resolution pixel coords, or None for full frame.
    """
    import open3d as o3d
    H, W  = depth_m.shape
    valid = np.isfinite(depth_m) & (depth_m > depth_min) & (depth_m < depth_max)
    if roi is not None:
        x1, y1, x2, y2 = roi
        roi_mask = np.zeros((H, W), dtype=bool)
        roi_mask[max(0,y1):min(H,y2), max(0,x1):min(W,x2)] = True
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


# ══════════════════════════════════════════════════════════════════════════════
#  Main
# ══════════════════════════════════════════════════════════════════════════════

def main():
    import open3d as o3d

    zed  = sl.Camera()
    init = sl.InitParameters()
    init.camera_resolution      = sl.RESOLUTION.HD2K
    init.camera_fps             = 15
    init.depth_mode             = sl.DEPTH_MODE.ULTRA
    init.coordinate_units       = sl.UNIT.MILLIMETER
    init.depth_minimum_distance = 300

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
    print("Handheld multi-view capture  |  Recommended: 800–1200mm, ~30–45° between views")
    print("Controls: [SPACE]=capture  [u]=undo  [q]=finish\n")

    out_dir = OUTPUT_ROOT / f"multiview_{time.strftime('%Y%m%d_%H%M%S')}"
    out_dir.mkdir(parents=True, exist_ok=True)

    left_mat  = sl.Mat()
    depth_mat = sl.Mat()
    runtime   = sl.RuntimeParameters()
    WIN       = "multiview_capture — ZED 2i"
    cv2.namedWindow(WIN, cv2.WINDOW_NORMAL)

    view_idx = 0

    while view_idx < MAX_VIEWS:
        if zed.grab(runtime) != sl.ERROR_CODE.SUCCESS:
            continue
        zed.retrieve_image(left_mat, sl.VIEW.LEFT)
        zed.retrieve_measure(depth_mat, sl.MEASURE.DEPTH)
        frame     = left_mat.get_data()[:, :, :3].copy()
        raw_depth = depth_mat.get_data().astype(np.float32)

        display = cv2.resize(frame, (int(W * DISPLAY_SCALE), int(H * DISPLAY_SCALE)))

        # live center depth OSD
        cxp, cyp = int(cx), int(cy)
        patch   = raw_depth[max(0, cyp-20):cyp+20, max(0, cxp-20):cxp+20]
        valid_d = patch[np.isfinite(patch) & (patch > 0)]
        live_z  = f"{float(np.median(valid_d)):.0f}mm" if len(valid_d) > 0 else "---"

        cv2.drawMarker(display, (int(cx*DISPLAY_SCALE), int(cy*DISPLAY_SCALE)),
                       (0, 200, 255), cv2.MARKER_CROSS, 30, 1, cv2.LINE_AA)
        cv2.putText(display, f"views: {view_idx}  |  live_z={live_z}",
                    (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.50, (0,255,255), 1)
        cv2.putText(display, "[SPACE]=capture  [u]=undo  [q]=finish",
                    (8, 44), cv2.FONT_HERSHEY_SIMPLEX, 0.40, (200,200,200), 1)
        bar = "[" + "#" * view_idx + "." * (8 - min(view_idx, 8)) + f"] {view_idx} views"
        cv2.putText(display, bar, (8, 64), cv2.FONT_HERSHEY_SIMPLEX, 0.40, (0,255,80), 1)

        cv2.imshow(WIN, display)
        key = cv2.waitKey(1) & 0xFF

        if key == ord('q'):
            print("Finishing capture.")
            break

        elif key == ord('u') and view_idx > 0:
            removed = out_dir / f"view_{view_idx-1:03d}.ply"
            view_idx -= 1
            if removed.exists():
                removed.unlink()
            print(f"  Undid view {view_idx}")

        elif key == ord(' '):
            print(f"\n--- Capturing view {view_idx} ---")
            depth_fused, color_bgr = grab_frames(zed)
            if depth_fused is None:
                print("  ERROR: capture failed"); continue

            # ROI selection on frozen frame
            roi_frame = cv2.resize(color_bgr, (int(W * DISPLAY_SCALE), int(H * DISPLAY_SCALE)))
            cv2.putText(roi_frame,
                        f"View {view_idx}: drag ROI around object, ENTER=confirm, C=full frame",
                        (8, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.50, (0, 255, 255), 1)
            roi_win = "Select ROI"
            r = cv2.selectROI(roi_win, roi_frame, fromCenter=False, showCrosshair=True)
            cv2.destroyWindow(roi_win)
            if r[2] > 10 and r[3] > 10:
                s   = 1.0 / DISPLAY_SCALE
                roi = (int(r[0]*s), int(r[1]*s), int((r[0]+r[2])*s), int((r[1]+r[3])*s))
                print(f"  ROI set: {roi}")
            else:
                roi = None
                print("  No ROI — using full frame")

            pcd = depth_to_pointcloud(depth_fused, color_bgr, fx, fy, cx, cy, roi=roi)
            n_pts = len(pcd.points)
            if n_pts < 1000:
                print(f"  Too few points ({n_pts}) — skip"); continue

            ply_path = out_dir / f"view_{view_idx:03d}.ply"
            o3d.io.write_point_cloud(str(ply_path), pcd)
            print(f"  view_{view_idx:03d}.ply  {n_pts:,} pts")
            view_idx += 1

            if view_idx >= MAX_VIEWS:
                print(f"  Max views ({MAX_VIEWS}) reached."); break

    cv2.destroyAllWindows()
    zed.close()

    if view_idx == 0:
        print("No views captured."); return

    # Save camera intrinsics for reference
    meta = {
        "timestamp":   time.strftime('%Y%m%d_%H%M%S'),
        "n_views":     view_idx,
        "camera":      "ZED 2i",
        "resolution":  [W, H],
        "intrinsics":  {"fx": fx, "fy": fy, "cx": cx, "cy": cy},
        "baseline_mm": B_mm,
        "depth_min_m": DEPTH_MIN_M,
        "depth_max_m": DEPTH_MAX_M,
        "n_frames_fused": N_FRAMES,
    }
    (out_dir / "capture_meta.json").write_text(json.dumps(meta, indent=2))

    print(f"\nCapture complete: {view_idx} views saved to:")
    print(f"  {out_dir}")
    print(f"  view_000.ply ... view_{view_idx-1:03d}.ply")
    print(f"  capture_meta.json")
    print(f"\nRun reconstruction with:")
    print(f"  python reconstruction/demo1_recon_coloredICP.py \"{out_dir}\"")


if __name__ == "__main__":
    main()
