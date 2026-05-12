"""
demo10_multiview_icp.py — Multi-viewpoint point cloud capture + ICP fusion (handheld mode).

Pipeline:
  1. Handheld camera: user moves to each position and presses [SPACE] to capture.
  2. Each capture → N-frame median-fused depth → back-projected point cloud (PLY).
  3. After all views captured, ICP registration chains them together:
       view0 (anchor) ← view1 ← view2 ← ... ← viewN
  4. Merged + voxel-downsampled point cloud saved, optional Poisson mesh.

Recommended:
  - Working distance 800–1200 mm (from distance sweep experiments)
  - Overlap between adjacent views: ~30–50% of FOV
  - Move camera ~30–45° between positions; keep object still

Controls:
  [SPACE]  : capture current view and advance
  [u]      : undo last capture
  [r]      : run ICP registration + save (can also run after [q])
  [q]      : quit capture loop → automatically run ICP on collected views

Output (vision_demo_test_res/multiview_<timestamp>/):
  view_000.ply, view_001.ply, ...   individual raw point clouds
  merged_raw.ply                    all views concatenated (no alignment)
  merged_icp.ply                    ICP-aligned and merged
  merged_icp_mesh.ply               optional Poisson mesh from merged cloud
  icp_log.txt                       per-pair registration fitness & RMSE
"""

import sys, os, time
from pathlib import Path

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
import zed_setup  # noqa: E402
import pyzed.sl as sl
import cv2
import numpy as np

# ── Config ────────────────────────────────────────────────────────────────────
N_FRAMES        = 15       # frames to median-fuse per viewpoint
SETTLE_SKIP     = 6        # frames to skip for auto-exposure
DEPTH_MIN_M     = 0.40     # ignore closer than this (m)
DEPTH_MAX_M     = 2.00     # ignore farther (m)
VOXEL_SIZE      = 0.005    # 5 mm voxel for downsampling during ICP
VOXEL_FINAL     = 0.003    # 3 mm voxel for final merged cloud
ICP_MAX_ITER    = 100
ICP_MAX_DIST    = 0.05     # 50 mm max correspondence distance for ICP
DISPLAY_SCALE   = 0.5
MAX_VIEWS       = 20

SCRIPT_DIR  = Path(__file__).resolve().parent
VISION_DIR  = SCRIPT_DIR.parent
OUTPUT_ROOT = VISION_DIR / "vision_demo_test_res"


# ══════════════════════════════════════════════════════════════════════════════
#  Capture helpers  (reused from demo8/9)
# ══════════════════════════════════════════════════════════════════════════════

def grab_frames(zed, n_frames=N_FRAMES, settle_skip=SETTLE_SKIP):
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
    """Back-project depth map → open3d PointCloud.
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
    zz = depth_m[valid]
    pts = np.stack([(uu[valid] - cx) * zz / fx,
                    (vv[valid] - cy) * zz / fy,
                    zz], axis=-1).astype(np.float64)
    cols = cv2.cvtColor(color_bgr, cv2.COLOR_BGR2RGB)[valid].astype(np.float64) / 255.0

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pts)
    pcd.colors = o3d.utility.Vector3dVector(cols)
    return pcd


# ══════════════════════════════════════════════════════════════════════════════
#  ICP Registration
# ══════════════════════════════════════════════════════════════════════════════

def preprocess_pcd(pcd, voxel_size):
    """Downsample + compute FPFH features for coarse registration."""
    import open3d as o3d
    down = pcd.voxel_down_sample(voxel_size)
    radius_normal = voxel_size * 2
    down.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))
    radius_feature = voxel_size * 5
    fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        down,
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100))
    return down, fpfh


def coarse_register(src_down, dst_down, src_fpfh, dst_fpfh, voxel_size):
    """FPFH + RANSAC global registration → initial transformation estimate."""
    import open3d as o3d
    dist_thresh = voxel_size * 1.5
    result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
        src_down, dst_down, src_fpfh, dst_fpfh,
        mutual_filter=True,
        max_correspondence_distance=dist_thresh,
        estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPoint(False),
        ransac_n=4,
        checkers=[
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(dist_thresh),
        ],
        criteria=o3d.pipelines.registration.RANSACConvergenceCriteria(4000000, 500),
    )
    return result.transformation


def fine_register(src, dst, init_transform, max_dist=ICP_MAX_DIST):
    """Point-to-Plane ICP fine registration."""
    import open3d as o3d
    dst.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=max_dist * 2, max_nn=30))
    result = o3d.pipelines.registration.registration_icp(
        src, dst,
        max_correspondence_distance=max_dist,
        init=init_transform,
        estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPlane(),
        criteria=o3d.pipelines.registration.ICPConvergenceCriteria(
            max_iteration=ICP_MAX_ITER),
    )
    return result.transformation, result.fitness, result.inlier_rmse


def register_all_views(pcds, voxel_size=VOXEL_SIZE, log_path=None):
    """
    Chain registration: view[0] is anchor, each subsequent view is aligned to
    the accumulated merged cloud for robustness.

    Returns list of 4×4 transforms (view[0] → identity).
    """
    import open3d as o3d

    transforms = [np.eye(4)]   # view 0 is anchor
    log_lines  = ["view_pair  fitness  inlier_rmse  coarse_fitness"]
    merged = pcds[0].voxel_down_sample(voxel_size)

    for i in range(1, len(pcds)):
        print(f"  Registering view {i} → accumulated cloud ...")
        src = pcds[i].voxel_down_sample(voxel_size)
        dst = merged

        # coarse
        src_d, src_f = preprocess_pcd(src, voxel_size)
        dst_d, dst_f = preprocess_pcd(dst, voxel_size)
        T_coarse = coarse_register(src_d, dst_d, src_f, dst_f, voxel_size)

        # fine
        T_fine, fitness, rmse = fine_register(src, dst, T_coarse)
        print(f"    fitness={fitness:.4f}  inlier_rmse={rmse*1000:.2f}mm")
        log_lines.append(f"0-{i}  {fitness:.4f}  {rmse*1000:.3f}mm")

        if fitness < 0.10:
            print(f"    WARNING: low fitness ({fitness:.3f}) — overlap may be insufficient")

        transforms.append(T_fine)

        # merge transformed src into accumulated cloud
        src_t = pcds[i].transform(T_fine)
        merged = merged + src_t
        merged = merged.voxel_down_sample(voxel_size)

    if log_path:
        Path(log_path).write_text("\n".join(log_lines))

    return transforms


# ══════════════════════════════════════════════════════════════════════════════
#  Save helpers
# ══════════════════════════════════════════════════════════════════════════════

def save_merged(out_dir, pcds, transforms, voxel_final=VOXEL_FINAL):
    """Apply transforms, merge, downsample, save PLY + optional Poisson mesh."""
    import open3d as o3d

    merged = o3d.geometry.PointCloud()
    for pcd, T in zip(pcds, transforms):
        c = pcd.transform(T.copy())
        merged += c

    merged_raw_path = out_dir / "merged_icp.ply"
    o3d.io.write_point_cloud(str(merged_raw_path), merged)
    print(f"  merged_icp.ply  ({len(merged.points):,} pts before downsample)")

    merged_ds = merged.voxel_down_sample(voxel_final)
    # Remove statistical outliers (isolated scatter points from background noise)
    n_before = len(merged_ds.points)
    merged_ds, _ = merged_ds.remove_statistical_outlier(nb_neighbors=30, std_ratio=2.0)
    print(f"  statistical outlier removal: {n_before:,} → {len(merged_ds.points):,} pts")
    merged_ds_path = out_dir / "merged_icp_ds.ply"
    o3d.io.write_point_cloud(str(merged_ds_path), merged_ds)
    print(f"  merged_icp_ds.ply  ({len(merged_ds.points):,} pts, voxel={voxel_final*1000:.0f}mm)")

    # Poisson mesh
    print("  Running Poisson surface reconstruction ...")
    merged_ds.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_final * 3, max_nn=30))
    merged_ds.orient_normals_consistent_tangent_plane(30)
    mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
        merged_ds, depth=9, scale=1.1, linear_fit=False)
    # trim low-density faces (artifacts at boundary)
    dens_arr = np.asarray(densities)
    keep = dens_arr > np.quantile(dens_arr, 0.05)
    mesh = mesh.select_by_index(np.where(keep)[0])
    mesh_path = out_dir / "merged_icp_mesh.ply"
    o3d.io.write_triangle_mesh(str(mesh_path), mesh, write_vertex_colors=True)
    print(f"  merged_icp_mesh.ply  ({len(mesh.vertices):,} verts, {len(mesh.triangles):,} tris)")

    return merged_ds


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
    print("Handheld multi-view capture mode")
    print("Recommended: 800–1200mm distance, ~30–45° between views, keep object still")
    print("Controls: [SPACE]=capture view  [u]=undo last  [r]=run ICP now  [q]=finish+ICP\n")

    out_dir = OUTPUT_ROOT / f"multiview_{time.strftime('%Y%m%d_%H%M%S')}"
    out_dir.mkdir(parents=True, exist_ok=True)

    left_mat  = sl.Mat()
    depth_mat = sl.Mat()
    runtime   = sl.RuntimeParameters()
    WIN       = "demo10 — multi-view ICP (handheld)"
    cv2.namedWindow(WIN, cv2.WINDOW_NORMAL)

    pcds      = []    # list of open3d PointCloud, one per view
    view_idx  = 0

    while view_idx < MAX_VIEWS:
        if zed.grab(runtime) != sl.ERROR_CODE.SUCCESS:
            continue
        zed.retrieve_image(left_mat, sl.VIEW.LEFT)
        zed.retrieve_measure(depth_mat, sl.MEASURE.DEPTH)
        frame     = left_mat.get_data()[:, :, :3].copy()
        raw_depth = depth_mat.get_data().astype(np.float32)

        display = cv2.resize(frame, (int(W * DISPLAY_SCALE), int(H * DISPLAY_SCALE)))

        # live center depth
        cxp, cyp = int(cx), int(cy)
        patch = raw_depth[max(0, cyp-20):cyp+20, max(0, cxp-20):cxp+20]
        valid_d = patch[np.isfinite(patch) & (patch > 0)]
        live_z = f"{float(np.median(valid_d)):.0f}mm" if len(valid_d) > 0 else "---"

        # crosshair
        cv2.drawMarker(display, (int(cx*DISPLAY_SCALE), int(cy*DISPLAY_SCALE)),
                       (0, 200, 255), cv2.MARKER_CROSS, 30, 1, cv2.LINE_AA)

        line1 = f"views captured: {view_idx}  |  live_z={live_z}"
        line2 = f"[SPACE]=capture  [u]=undo  [r]=ICP now  [q]=finish+ICP"
        cv2.putText(display, line1, (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.50, (0,255,255), 1)
        cv2.putText(display, line2, (8, 44), cv2.FONT_HERSHEY_SIMPLEX, 0.40, (200,200,200), 1)

        # view count bar
        bar = "[" + "#" * view_idx + "." * (8 - min(view_idx, 8)) + f"] {view_idx} views"
        cv2.putText(display, bar, (8, 64), cv2.FONT_HERSHEY_SIMPLEX, 0.40, (0,255,80), 1)

        cv2.imshow(WIN, display)
        key = cv2.waitKey(1) & 0xFF

        if key == ord('q'):
            print("Finishing capture.")
            break
        elif key == ord('r') and len(pcds) >= 2:
            break
        elif key == ord('u') and pcds:
            removed_path = out_dir / f"view_{view_idx-1:03d}.ply"
            pcds.pop()
            view_idx -= 1
            if removed_path.exists():
                removed_path.unlink()
            print(f"  Undid view {view_idx}")
        elif key == ord(' '):
            print(f"\n--- Capturing view {view_idx} ---")
            depth_fused, color_bgr = grab_frames(zed)

            if depth_fused is None:
                print("  ERROR: capture failed"); continue

            # ── ROI selection on frozen frame ─────────────────────────────
            roi_frame = cv2.resize(color_bgr, (int(W * DISPLAY_SCALE), int(H * DISPLAY_SCALE)))
            cv2.putText(roi_frame,
                        f"View {view_idx}: drag ROI around object, ENTER=confirm, C=full frame",
                        (8, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.50, (0, 255, 255), 1)
            roi_win = "Select ROI"
            r = cv2.selectROI(roi_win, roi_frame, fromCenter=False, showCrosshair=True)
            cv2.destroyWindow(roi_win)
            if r[2] > 10 and r[3] > 10:   # valid rectangle
                s = 1.0 / DISPLAY_SCALE
                roi = (int(r[0]*s), int(r[1]*s),
                       int((r[0]+r[2])*s), int((r[1]+r[3])*s))
                print(f"  ROI set: {roi}")
            else:
                roi = None
                print("  No ROI — using full frame (background will be included)")
            # ─────────────────────────────────────────────────────────────

            pcd = depth_to_pointcloud(depth_fused, color_bgr, fx, fy, cx, cy, roi=roi)
            n_pts = len(pcd.points)
            if n_pts < 1000:
                print(f"  Too few points ({n_pts}) — move closer or check depth range"); continue

            ply_path = out_dir / f"view_{view_idx:03d}.ply"
            o3d.io.write_point_cloud(str(ply_path), pcd)
            print(f"  view_{view_idx:03d}.ply  {n_pts:,} pts  → {ply_path}")

            pcds.append(pcd)
            view_idx += 1

            if view_idx >= MAX_VIEWS:
                print(f"  Max views ({MAX_VIEWS}) reached — finishing."); break

    cv2.destroyAllWindows()

    if len(pcds) < 2:
        print(f"Need at least 2 views for ICP (got {len(pcds)}). Exiting.")
        zed.close()
        return

    zed.close()

    print(f"\n{'='*60}")
    print(f"Running ICP on {len(pcds)} views ...")
    t0 = time.perf_counter()
    transforms = register_all_views(pcds, voxel_size=VOXEL_SIZE,
                                    log_path=str(out_dir / "icp_log.txt"))
    print(f"ICP done in {time.perf_counter()-t0:.1f}s")

    print("\nSaving merged cloud + Poisson mesh ...")
    save_merged(out_dir, pcds, transforms, voxel_final=VOXEL_FINAL)

    print(f"\nOutput dir: {out_dir}")
    print(f"  view_000.ply ... view_{len(pcds)-1:03d}.ply  (raw per-view clouds)")
    print(f"  merged_icp.ply / merged_icp_ds.ply / merged_icp_mesh.ply")
    print(f"  icp_log.txt  (fitness & RMSE per pair)")


if __name__ == "__main__":
    main()
