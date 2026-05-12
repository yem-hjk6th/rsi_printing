"""
demo12_tsdf_recon.py — TSDF volume fusion with Pose Graph optimization.

Implements the Artec-style pipeline:
  1. Load per-view depth.npy + color.png from demo11_tsdf_capture.py output
  2. Pairwise registration: FPFH+RANSAC coarse → Geometric ICP → Colored ICP
  3. Build Pose Graph (all pairs relative to view 0)
  4. Global Pose Graph optimization (Levenberg-Marquardt, with loop closure)
  5. TSDF volume integration using optimized poses
  6. Marching Cubes mesh extraction

Why this is better than chain ICP (demo1):
  - Pose Graph optimizes ALL poses simultaneously, no error accumulation
  - TSDF fusion averages overlapping depth observations instead of stacking points
  - Result: sharper surfaces, no double-layer artifacts

Usage:
    python demo12_tsdf_recon.py <input_dir>

    <input_dir>: path to a tsdf_* directory from demo11_tsdf_capture.py
                 must contain view_NNN_depth.npy + view_NNN_color.png + capture_meta.json

Output (written into <input_dir>/tsdf_recon/):
    mesh.ply           Marching Cubes mesh from TSDF volume
    merged_ds.ply      voxel-downsampled point cloud (for quick inspection)
    recon_log.txt      per-pair pairwise registration fitness & RMSE
"""

import sys
import json
import time
import argparse
from pathlib import Path

import numpy as np
import open3d as o3d

# ── Default input (edit here to run without arguments) ───────────────────────
DEFAULT_INPUT_DIR = r"Vision/vision_demo_test_res/tsdf_20260505_153042"

# ── Config ────────────────────────────────────────────────────────────────────
VOXEL_COARSE    = 0.010    # 10mm — FPFH feature extraction voxel
VOXEL_GEOM      = 0.005    # 5mm  — geometric ICP voxel
VOXEL_COLOR     = 0.003    # 3mm  — colored ICP voxel
TSDF_VOXEL      = 0.001    # 1mm  — TSDF volume voxel (final mesh resolution)
TSDF_TRUNC      = 0.004    # 4mm  — TSDF truncation distance
DEPTH_MIN_M     = 0.40
DEPTH_MAX_M     = 2.00
RANSAC_ITER     = 4_000_000
RANSAC_CONF     = 500
ICP_MAX_ITER    = 100
ICP_GEOM_DIST   = 0.020    # 20mm
ICP_COLOR_DIST  = 0.010    # 10mm
OUTLIER_NB      = 30
OUTLIER_STD     = 2.0
POSEGRAPH_MAX_DIST = 0.005  # 5mm — for information matrix & edge pruning


# ══════════════════════════════════════════════════════════════════════════════
#  Registration helpers (same as demo1, reused)
# ══════════════════════════════════════════════════════════════════════════════

def compute_fpfh(pcd, voxel_size):
    down = pcd.voxel_down_sample(voxel_size)
    down.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size * 2, max_nn=30))
    fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        down,
        o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size * 5, max_nn=100))
    return down, fpfh


def coarse_ransac(src_down, dst_down, src_fpfh, dst_fpfh, voxel_size):
    dist = voxel_size * 1.5
    result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
        src_down, dst_down, src_fpfh, dst_fpfh,
        mutual_filter=True,
        max_correspondence_distance=dist,
        estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPoint(False),
        ransac_n=4,
        checkers=[
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(dist),
        ],
        criteria=o3d.pipelines.registration.RANSACConvergenceCriteria(RANSAC_ITER, RANSAC_CONF),
    )
    return result.transformation


def pairwise_registration(src, dst):
    """FPFH+RANSAC → Geometric ICP → Colored ICP. Returns (T, fitness, rmse)."""
    # coarse
    src_d, src_f = compute_fpfh(src, VOXEL_COARSE)
    dst_d, dst_f = compute_fpfh(dst, VOXEL_COARSE)
    T_coarse = coarse_ransac(src_d, dst_d, src_f, dst_f, VOXEL_COARSE)

    # geometric ICP
    src_g = src.voxel_down_sample(VOXEL_GEOM)
    dst_g = dst.voxel_down_sample(VOXEL_GEOM)
    for pc in [src_g, dst_g]:
        pc.estimate_normals(
            o3d.geometry.KDTreeSearchParamHybrid(radius=VOXEL_GEOM * 2, max_nn=30))
    res_g = o3d.pipelines.registration.registration_icp(
        src_g, dst_g,
        max_correspondence_distance=ICP_GEOM_DIST,
        init=T_coarse,
        estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPlane(),
        criteria=o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=ICP_MAX_ITER),
    )

    # colored ICP
    src_c = src.voxel_down_sample(VOXEL_COLOR)
    dst_c = dst.voxel_down_sample(VOXEL_COLOR)
    for pc in [src_c, dst_c]:
        pc.estimate_normals(
            o3d.geometry.KDTreeSearchParamHybrid(radius=VOXEL_COLOR * 2, max_nn=30))
    res_c = o3d.pipelines.registration.registration_colored_icp(
        src_c, dst_c,
        max_correspondence_distance=ICP_COLOR_DIST,
        init=res_g.transformation,
        estimation_method=o3d.pipelines.registration.TransformationEstimationForColoredICP(),
        criteria=o3d.pipelines.registration.ICPConvergenceCriteria(
            relative_fitness=1e-6, relative_rmse=1e-6, max_iteration=ICP_MAX_ITER),
    )
    return res_c.transformation, res_c.fitness, res_c.inlier_rmse


def compute_info_matrix(src, dst, T, max_dist=POSEGRAPH_MAX_DIST):
    """Compute information matrix for pose graph edge."""
    src_t = o3d.geometry.PointCloud(src)
    src_t.transform(T)
    return o3d.pipelines.registration.get_information_matrix_from_point_clouds(
        src_t, dst, max_dist, np.eye(4))


# ══════════════════════════════════════════════════════════════════════════════
#  Pose Graph build + optimize
# ══════════════════════════════════════════════════════════════════════════════

def build_pose_graph(pcds, log_lines):
    """
    Register all view pairs relative to view 0, build Pose Graph.
    Adds:
      - Sequential edges: (i-1) → i  (certain, direct neighbours)
      - Loop edges: 0 → i            (uncertain, via chained transforms)
    """
    n = len(pcds)
    pose_graph = o3d.pipelines.registration.PoseGraph()

    # Nodes — initial poses (all identity; optimizer will refine)
    odometry = np.eye(4)
    pose_graph.nodes.append(o3d.pipelines.registration.PoseGraphNode(odometry))

    pairwise_transforms = {}  # (i, j) → T_i_to_j

    for i in range(n - 1):
        j = i + 1
        print(f"\n  View {i} → {j}")
        T, fitness, rmse = pairwise_registration(pcds[i], pcds[j])
        pairwise_transforms[(i, j)] = T
        print(f"    colored ICP  fitness={fitness:.4f}  rmse={rmse*1000:.2f}mm")
        log_lines.append(f"{i}-{j}  {fitness:.4f}  {rmse*1000:.3f}mm")

        if fitness < 0.10:
            print(f"    WARNING: low fitness ({fitness:.3f})")

        # Update cumulative odometry for node initialisation
        odometry = T @ odometry
        pose_graph.nodes.append(o3d.pipelines.registration.PoseGraphNode(
            np.linalg.inv(odometry)))

        # Sequential edge (certain = low uncertainty)
        info = compute_info_matrix(pcds[i], pcds[j], T)
        pose_graph.edges.append(o3d.pipelines.registration.PoseGraphEdge(
            i, j, T, info, uncertain=False))

    # Loop closure edges: connect each view back to view 0
    # (uncertain = True → treated as soft constraints in optimizer)
    T_cumulative = np.eye(4)
    for i in range(1, n):
        T_cumulative = pairwise_transforms[(i-1, i)] @ T_cumulative
        info = compute_info_matrix(pcds[0], pcds[i], np.linalg.inv(T_cumulative))
        pose_graph.edges.append(o3d.pipelines.registration.PoseGraphEdge(
            0, i, np.linalg.inv(T_cumulative), info, uncertain=True))

    return pose_graph


def optimize_pose_graph(pose_graph):
    """Run Levenberg-Marquardt global optimization on the pose graph."""
    print("\n  Running Pose Graph global optimization ...")
    o3d.pipelines.registration.global_optimization(
        pose_graph,
        o3d.pipelines.registration.GlobalOptimizationLevenbergMarquardt(),
        o3d.pipelines.registration.GlobalOptimizationConvergenceCriteria(),
        o3d.pipelines.registration.GlobalOptimizationOption(
            max_correspondence_distance=POSEGRAPH_MAX_DIST,
            edge_prune_threshold=0.25,
            reference_node=0,
        ),
    )
    return [pg_node.pose for pg_node in pose_graph.nodes]


# ══════════════════════════════════════════════════════════════════════════════
#  TSDF integration
# ══════════════════════════════════════════════════════════════════════════════

def mask_depth_from_pcd(depth_m, pcd, fx, fy, cx, cy, margin=60):
    """Zero depth outside the bounding box of back-projected PCD points.
    Handles existing data where depth.npy was saved full-frame."""
    pts = np.asarray(pcd.points)
    if len(pts) == 0:
        return depth_m
    H, W = depth_m.shape
    u = (fx * pts[:, 0] / pts[:, 2] + cx).astype(int)
    v = (fy * pts[:, 1] / pts[:, 2] + cy).astype(int)
    u1 = max(0, int(u.min()) - margin)
    u2 = min(W, int(u.max()) + margin)
    v1 = max(0, int(v.min()) - margin)
    v2 = min(H, int(v.max()) + margin)
    masked = depth_m.copy()
    roi_mask = np.zeros((H, W), dtype=bool)
    roi_mask[v1:v2, u1:u2] = True
    masked[~roi_mask] = np.nan
    return masked


def integrate_tsdf(depths, colors, poses, intrinsic, pcds=None):
    """
    Integrate all RGBD frames into a shared TSDF volume using optimized poses.
    depths: list of (H, W) float32 arrays in meters
    colors: list of (H, W, 3) uint8 RGB arrays
    poses:  list of 4×4 world-to-camera transforms (i.e. inv of camera-in-world)
    pcds:   list of ROI-cropped PointClouds — used to mask background from depth
    """
    fx, fy = intrinsic.get_focal_length()
    cx, cy = intrinsic.get_principal_point()
    volume = o3d.pipelines.integration.ScalableTSDFVolume(
        voxel_length=TSDF_VOXEL,
        sdf_trunc=TSDF_TRUNC,
        color_type=o3d.pipelines.integration.TSDFVolumeColorType.RGB8,
    )

    for i, (depth_m, color_rgb, T_world) in enumerate(zip(depths, colors, poses)):
        # Mask background using PLY back-projection (handles full-frame depth.npy)
        if pcds is not None:
            depth_m = mask_depth_from_pcd(depth_m, pcds[i], fx, fy, cx, cy)
        # Replace NaN with 0 (TSDF treats 0 as invalid)
        depth_clean = np.nan_to_num(depth_m, nan=0.0).astype(np.float32)

        color_o3d = o3d.geometry.Image(color_rgb.astype(np.uint8))
        depth_o3d = o3d.geometry.Image(depth_clean)

        rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
            color_o3d, depth_o3d,
            depth_scale=1.0,       # depth is already in meters
            depth_trunc=DEPTH_MAX_M,
            convert_rgb_to_intensity=False,
        )

        # T_world is camera-in-world pose (from pose graph nodes)
        # TSDF integrate expects extrinsic = world-to-camera = inv(T_world)
        extrinsic = np.linalg.inv(T_world)
        volume.integrate(rgbd, intrinsic, extrinsic)
        print(f"  Integrated view {i}")

    return volume


# ══════════════════════════════════════════════════════════════════════════════
#  Main
# ══════════════════════════════════════════════════════════════════════════════

def main():
    parser = argparse.ArgumentParser(
        description="TSDF reconstruction with Pose Graph optimization (Artec-style)")
    parser.add_argument("input_dir", nargs="?", default=DEFAULT_INPUT_DIR,
                        help="Path to tsdf_* directory from demo11_tsdf_capture.py")
    args = parser.parse_args()

    input_dir = Path(args.input_dir).resolve()
    if not input_dir.exists():
        print(f"ERROR: directory not found: {input_dir}"); return

    # Load metadata
    meta_path = input_dir / "capture_meta.json"
    if not meta_path.exists():
        print(f"ERROR: capture_meta.json not found in {input_dir}"); return
    meta = json.loads(meta_path.read_text())
    fx = meta["intrinsics"]["fx"]
    fy = meta["intrinsics"]["fy"]
    cx = meta["intrinsics"]["cx"]
    cy = meta["intrinsics"]["cy"]
    W, H = meta["resolution"]

    intrinsic = o3d.camera.PinholeCameraIntrinsic(W, H, fx, fy, cx, cy)

    # Find views
    depth_files = sorted(input_dir.glob("view_*_depth.npy"))
    color_files = sorted(input_dir.glob("view_*_color.png"))
    ply_files   = sorted(input_dir.glob("view_*.ply"))

    if len(depth_files) < 2:
        print(f"ERROR: need at least 2 view_*_depth.npy files, found {len(depth_files)}")
        print("  Use demo11_tsdf_capture.py to capture data with this script.")
        return

    if len(depth_files) != len(color_files):
        print(f"ERROR: depth count ({len(depth_files)}) != color count ({len(color_files)})")
        return

    n = len(depth_files)
    print(f"Loading {n} views from: {input_dir}")

    depths, colors, pcds = [], [], []
    for df, cf, pf in zip(depth_files, color_files, ply_files):
        depth_m = np.load(str(df)).astype(np.float32)
        color_bgr = __import__('cv2').imread(str(cf))
        color_rgb = __import__('cv2').cvtColor(color_bgr, __import__('cv2').COLOR_BGR2RGB)
        pcd = o3d.io.read_point_cloud(str(pf))
        depths.append(depth_m)
        colors.append(color_rgb)
        pcds.append(pcd)
        print(f"  {df.name}  {depth_m.shape}  pts={len(pcd.points):,}  "
              f"colors={'yes' if pcd.has_colors() else 'NO'}")

    out_dir = input_dir / "tsdf_recon"
    out_dir.mkdir(exist_ok=True)

    log_lines = ["pair  fitness  rmse_mm"]

    # ── Step 1: Pairwise registration + Pose Graph ─────────────────────────
    print(f"\n{'='*60}")
    print(f"Step 1/3 — Pairwise registration + Pose Graph ({n} views) ...")
    t0 = time.perf_counter()
    pose_graph = build_pose_graph(pcds, log_lines)
    print(f"\n  Pairwise registration done in {time.perf_counter()-t0:.1f}s")

    # ── Step 2: Global Pose Graph optimization ─────────────────────────────
    print(f"\n{'='*60}")
    print("Step 2/3 — Global Pose Graph optimization ...")
    optimized_poses = optimize_pose_graph(pose_graph)
    print(f"  Optimized {len(optimized_poses)} poses")

    # ── Step 3: TSDF integration ───────────────────────────────────────────
    print(f"\n{'='*60}")
    print(f"Step 3/3 — TSDF volume integration  "
          f"(voxel={TSDF_VOXEL*1000:.0f}mm, trunc={TSDF_TRUNC*1000:.0f}mm) ...")
    t1 = time.perf_counter()
    volume = integrate_tsdf(depths, colors, optimized_poses, intrinsic, pcds)
    print(f"  Integration done in {time.perf_counter()-t1:.1f}s")

    # ── Save ───────────────────────────────────────────────────────────────
    print("\nExtracting mesh ...")
    mesh = volume.extract_triangle_mesh()
    mesh.compute_vertex_normals()
    mesh_path = out_dir / "mesh.ply"
    o3d.io.write_triangle_mesh(str(mesh_path), mesh, write_vertex_colors=True)
    print(f"  mesh.ply  ({len(mesh.vertices):,} verts, {len(mesh.triangles):,} tris)")

    # Also save point cloud version
    pcd_merged = volume.extract_point_cloud()
    n_before = len(pcd_merged.points)
    pcd_merged, _ = pcd_merged.remove_statistical_outlier(
        nb_neighbors=OUTLIER_NB, std_ratio=OUTLIER_STD)
    ds_path = out_dir / "merged_ds.ply"
    o3d.io.write_point_cloud(str(ds_path), pcd_merged)
    print(f"  merged_ds.ply  ({n_before:,} → {len(pcd_merged.points):,} pts after outlier removal)")

    # Save log
    (out_dir / "recon_log.txt").write_text("\n".join(log_lines))

    total = time.perf_counter() - t0
    print(f"\nTotal: {total:.1f}s")
    print(f"\nOutput: {out_dir}")
    print(f"  mesh.ply       — TSDF Marching Cubes mesh (open in MeshLab/CloudCompare)")
    print(f"  merged_ds.ply  — point cloud")
    print(f"  recon_log.txt  — per-pair fitness & RMSE")


if __name__ == "__main__":
    main()
