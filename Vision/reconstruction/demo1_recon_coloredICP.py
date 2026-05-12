"""
demo1_recon_coloredICP.py — Multi-view point cloud fusion using Colored ICP.

Loads per-view PLY files from a multiview_capture.py output directory and
fuses them with FPFH+RANSAC coarse registration followed by Colored ICP
(Park et al. 2017), which jointly optimizes geometry + RGB color residuals.

Colored ICP is significantly more robust than geometric-only ICP on objects
with rotationally symmetric geometry (e.g. concentric-square prints) because
color gradients break the degeneracy that causes RANSAC/geometric ICP to
converge to wrong rotations.

Usage:
    python demo1_recon_coloredICP.py <input_dir>

    <input_dir>: path to a multiview_* directory containing view_*.ply files
                 e.g. vision_demo_test_res/multiview_20260505_133235

Output (written into <input_dir>/colored_icp/):
    merged_ds.ply          voxel-downsampled + outlier-cleaned merged cloud
    merged_mesh.ply        Poisson surface mesh
    recon_log.txt          per-pair fitness / RMSE
"""

import sys
import time
import argparse
from pathlib import Path

import numpy as np
import open3d as o3d

# ── Config ────────────────────────────────────────────────────────────────────
VOXEL_COARSE    = 0.010    # 10 mm  — voxel size for FPFH feature extraction
VOXEL_GEOM      = 0.005    # 5 mm   — voxel size for geometric ICP (stage 1)
VOXEL_COLOR     = 0.003    # 3 mm   — voxel size for Colored ICP (stage 2)
VOXEL_FINAL     = 0.003    # 3 mm   — final merged cloud voxel
RANSAC_ITER     = 4_000_000
RANSAC_CONF     = 500
ICP_GEOM_DIST   = 0.020    # 20 mm  — geometric ICP max correspondence distance
ICP_COLOR_DIST  = 0.010    # 10 mm  — Colored ICP max correspondence distance
ICP_MAX_ITER    = 100
OUTLIER_NB      = 30
OUTLIER_STD     = 2.0
POISSON_DEPTH   = 9


# ══════════════════════════════════════════════════════════════════════════════
#  Registration helpers
# ══════════════════════════════════════════════════════════════════════════════

def compute_fpfh(pcd, voxel_size):
    """Downsample, estimate normals, compute FPFH features."""
    down = pcd.voxel_down_sample(voxel_size)
    down.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size * 2, max_nn=30))
    fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        down,
        o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size * 5, max_nn=100))
    return down, fpfh


def coarse_ransac(src_down, dst_down, src_fpfh, dst_fpfh, voxel_size):
    """FPFH + RANSAC global registration → rough initial transform."""
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


def geom_icp(src, dst, T_init, max_dist=ICP_GEOM_DIST):
    """Stage 1: Point-to-Plane ICP at coarser voxel to get close."""
    src_d = src.voxel_down_sample(VOXEL_GEOM)
    dst_d = dst.voxel_down_sample(VOXEL_GEOM)
    for pc in [src_d, dst_d]:
        pc.estimate_normals(
            o3d.geometry.KDTreeSearchParamHybrid(radius=VOXEL_GEOM * 2, max_nn=30))
    result = o3d.pipelines.registration.registration_icp(
        src_d, dst_d,
        max_correspondence_distance=max_dist,
        init=T_init,
        estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPlane(),
        criteria=o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=ICP_MAX_ITER),
    )
    return result.transformation, result.fitness, result.inlier_rmse


def colored_icp(src, dst, T_init, max_dist=ICP_COLOR_DIST):
    """Stage 2: Colored ICP — jointly optimizes geometry + RGB color residuals.
    Both src and dst must have colors. Uses VOXEL_COLOR resolution.
    """
    src_d = src.voxel_down_sample(VOXEL_COLOR)
    dst_d = dst.voxel_down_sample(VOXEL_COLOR)
    for pc in [src_d, dst_d]:
        pc.estimate_normals(
            o3d.geometry.KDTreeSearchParamHybrid(radius=VOXEL_COLOR * 2, max_nn=30))
    result = o3d.pipelines.registration.registration_colored_icp(
        src_d, dst_d,
        max_correspondence_distance=max_dist,
        init=T_init,
        estimation_method=o3d.pipelines.registration.TransformationEstimationForColoredICP(),
        criteria=o3d.pipelines.registration.ICPConvergenceCriteria(
            relative_fitness=1e-6, relative_rmse=1e-6, max_iteration=ICP_MAX_ITER),
    )
    return result.transformation, result.fitness, result.inlier_rmse


def register_all(pcds, log_path=None):
    """
    Chain registration: view[0] is anchor.
    Per pair: FPFH+RANSAC coarse → Geometric ICP → Colored ICP.
    Each new view is aligned to the accumulated merged cloud for robustness.
    Returns list of 4×4 transforms.
    """
    transforms = [np.eye(4)]
    log_lines  = ["pair  coarse_ok  geom_fitness  geom_rmse_mm  color_fitness  color_rmse_mm"]
    merged = pcds[0].voxel_down_sample(VOXEL_COLOR)

    for i in range(1, len(pcds)):
        print(f"\n  View {i} → accumulated cloud")
        src = pcds[i]
        dst = merged

        # ── coarse: FPFH+RANSAC ────────────────────────────────────────────
        src_d, src_f = compute_fpfh(src, VOXEL_COARSE)
        dst_d, dst_f = compute_fpfh(dst, VOXEL_COARSE)
        T_coarse = coarse_ransac(src_d, dst_d, src_f, dst_f, VOXEL_COARSE)
        print(f"    coarse RANSAC done")

        # ── stage 1: geometric ICP ─────────────────────────────────────────
        T_geom, g_fit, g_rmse = geom_icp(src, dst, T_coarse)
        print(f"    geom ICP    fitness={g_fit:.4f}  rmse={g_rmse*1000:.2f}mm")

        # ── stage 2: colored ICP ───────────────────────────────────────────
        T_final, c_fit, c_rmse = colored_icp(src, dst, T_geom)
        print(f"    colored ICP fitness={c_fit:.4f}  rmse={c_rmse*1000:.2f}mm")

        if c_fit < 0.10:
            print(f"    WARNING: low colored ICP fitness ({c_fit:.3f})")

        transforms.append(T_final)
        log_lines.append(
            f"0-{i}  ok  {g_fit:.4f}  {g_rmse*1000:.3f}  {c_fit:.4f}  {c_rmse*1000:.3f}")

        # accumulate
        src_t = pcds[i].voxel_down_sample(VOXEL_COLOR)
        src_t.transform(T_final)
        merged = merged + src_t
        merged = merged.voxel_down_sample(VOXEL_COLOR)

    if log_path:
        Path(log_path).write_text("\n".join(log_lines))

    return transforms


# ══════════════════════════════════════════════════════════════════════════════
#  Save
# ══════════════════════════════════════════════════════════════════════════════

def save_results(out_dir, pcds, transforms):
    merged = o3d.geometry.PointCloud()
    for pcd, T in zip(pcds, transforms):
        c = o3d.geometry.PointCloud(pcd)
        c.transform(T)
        merged += c

    # downsample + outlier removal
    merged_ds = merged.voxel_down_sample(VOXEL_FINAL)
    n_before  = len(merged_ds.points)
    merged_ds, _ = merged_ds.remove_statistical_outlier(
        nb_neighbors=OUTLIER_NB, std_ratio=OUTLIER_STD)
    print(f"\n  outlier removal: {n_before:,} → {len(merged_ds.points):,} pts")

    ds_path = out_dir / "merged_ds.ply"
    o3d.io.write_point_cloud(str(ds_path), merged_ds)
    print(f"  merged_ds.ply  ({len(merged_ds.points):,} pts, voxel={VOXEL_FINAL*1000:.0f}mm)")

    # Poisson mesh
    print("  Running Poisson reconstruction ...")
    merged_ds.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=VOXEL_FINAL * 3, max_nn=30))
    merged_ds.orient_normals_consistent_tangent_plane(30)
    mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
        merged_ds, depth=POISSON_DEPTH, scale=1.1, linear_fit=False)
    keep = np.asarray(densities) > np.quantile(np.asarray(densities), 0.05)
    mesh = mesh.select_by_index(np.where(keep)[0])
    mesh_path = out_dir / "merged_mesh.ply"
    o3d.io.write_triangle_mesh(str(mesh_path), mesh, write_vertex_colors=True)
    print(f"  merged_mesh.ply  ({len(mesh.vertices):,} verts, {len(mesh.triangles):,} tris)")

    return merged_ds


# ══════════════════════════════════════════════════════════════════════════════
#  Main
# ══════════════════════════════════════════════════════════════════════════════

def main():
    parser = argparse.ArgumentParser(
        description="Multi-view Colored ICP fusion from multiview_capture.py output")
    parser.add_argument("input_dir", type=str,
                        help="Path to multiview_* directory with view_*.ply files")
    args = parser.parse_args()

    input_dir = Path(args.input_dir).resolve()
    if not input_dir.exists():
        print(f"ERROR: directory not found: {input_dir}"); return

    ply_files = sorted(input_dir.glob("view_*.ply"))
    if len(ply_files) < 2:
        print(f"ERROR: need at least 2 view_*.ply files, found {len(ply_files)}"); return

    print(f"Loading {len(ply_files)} views from: {input_dir}")
    pcds = []
    for p in ply_files:
        pcd = o3d.io.read_point_cloud(str(p))
        if not pcd.has_colors():
            print(f"  WARNING: {p.name} has no colors — Colored ICP will fall back to geometric")
        print(f"  {p.name}  {len(pcd.points):,} pts  colors={'yes' if pcd.has_colors() else 'NO'}")
        pcds.append(pcd)

    out_dir = input_dir / "colored_icp"
    out_dir.mkdir(exist_ok=True)

    print(f"\n{'='*60}")
    print(f"Running Colored ICP fusion on {len(pcds)} views ...")
    print(f"  RANSAC voxel={VOXEL_COARSE*1000:.0f}mm  "
          f"geom ICP voxel={VOXEL_GEOM*1000:.0f}mm  "
          f"colored ICP voxel={VOXEL_COLOR*1000:.0f}mm")
    t0 = time.perf_counter()

    transforms = register_all(pcds, log_path=str(out_dir / "recon_log.txt"))

    elapsed = time.perf_counter() - t0
    print(f"\nRegistration done in {elapsed:.1f}s")

    print("\nSaving results ...")
    save_results(out_dir, pcds, transforms)

    print(f"\nOutput: {out_dir}")
    print(f"  merged_ds.ply   — point cloud (open in MeshLab/CloudCompare)")
    print(f"  merged_mesh.ply — Poisson mesh")
    print(f"  recon_log.txt   — per-pair fitness & RMSE")


if __name__ == "__main__":
    main()
