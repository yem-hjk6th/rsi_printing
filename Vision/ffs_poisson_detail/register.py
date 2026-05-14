"""
register.py — Pose estimation for artec_imit pipeline.

Two strategies, selected automatically based on view count and warmup state:

  Strategy A — Pose Graph (FPFH+RANSAC → Colored ICP → global opt)
    Used for: first FTM_WARMUP_FRAMES views, or when frame-to-model tracking fails.
    Same as demo12 but with corrected info matrix and all params from config.py.

  Strategy B — Frame-to-model (FTM) tracking
    Used for: views after warmup, when initial TSDF model is available.
    Each new frame is registered against the *current TSDF point cloud* via
    Colored ICP, initialized from the previous pose.
    Advantage: registration target is multi-view fused → no rotational symmetry trap.
    Fallback: if fitness < FTM_MIN_FITNESS, retries with RANSAC from scratch.

Public API:
    poses, log = register(pcds, depths, colors, intrinsic, roi_in_depth=False)

    Returns:
        poses: list of 4×4 ndarray (camera-in-world, same convention as PoseGraph nodes)
        log:   list of strings for recon_log.txt
"""

import time
import numpy as np
import open3d as o3d

from config import (
    VOXEL_COARSE, VOXEL_GEOM, VOXEL_COLOR,
    RANSAC_ITER, RANSAC_CONF,
    ICP_MAX_ITER, ICP_GEOM_DIST, ICP_COLOR_DIST,
    POSEGRAPH_MAX_DIST, POSEGRAPH_EDGE_PRUNE, LOOP_OVERLAP_MIN,
    FTM_WARMUP_FRAMES, FTM_ICP_DIST, FTM_MIN_FITNESS,
    TSDF_VOXEL, TSDF_TRUNC, DEPTH_MAX_M,
)
import fuse


# ══════════════════════════════════════════════════════════════════════════════
#  Low-level registration primitives
# ══════════════════════════════════════════════════════════════════════════════

def _compute_fpfh(pcd, voxel_size):
    down = pcd.voxel_down_sample(voxel_size)
    down.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size * 2, max_nn=30))
    fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        down,
        o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size * 5, max_nn=100))
    return down, fpfh


def _ransac(src_d, dst_d, src_f, dst_f, voxel_size):
    dist = voxel_size * 1.5
    res  = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
        src_d, dst_d, src_f, dst_f,
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
    return res.transformation


def _icp_geom(src, dst, T_init, max_dist=ICP_GEOM_DIST):
    vox = VOXEL_GEOM
    s   = src.voxel_down_sample(vox)
    d   = dst.voxel_down_sample(vox)
    for pc in [s, d]:
        pc.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=vox*2, max_nn=30))
    return o3d.pipelines.registration.registration_icp(
        s, d, max_dist, T_init,
        o3d.pipelines.registration.TransformationEstimationPointToPlane(),
        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=ICP_MAX_ITER),
    )


def _icp_colored(src, dst, T_init, max_dist=ICP_COLOR_DIST):
    vox = VOXEL_COLOR
    s   = src.voxel_down_sample(vox)
    d   = dst.voxel_down_sample(vox)
    for pc in [s, d]:
        pc.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=vox*2, max_nn=30))
    return o3d.pipelines.registration.registration_colored_icp(
        s, d, max_dist, T_init,
        o3d.pipelines.registration.TransformationEstimationForColoredICP(),
        o3d.pipelines.registration.ICPConvergenceCriteria(
            relative_fitness=1e-6, relative_rmse=1e-6, max_iteration=ICP_MAX_ITER),
    )


def pairwise(src, dst):
    """FPFH+RANSAC → Geom ICP → Colored ICP. Returns (T, fitness, rmse)."""
    s_d, s_f = _compute_fpfh(src, VOXEL_COARSE)
    d_d, d_f = _compute_fpfh(dst, VOXEL_COARSE)
    T_coarse  = _ransac(s_d, d_d, s_f, d_f, VOXEL_COARSE)
    res_g     = _icp_geom(src, dst, T_coarse)
    res_c     = _icp_colored(src, dst, res_g.transformation)
    return res_c.transformation, res_c.fitness, res_c.inlier_rmse


def _info_matrix(src, dst, T, max_dist=POSEGRAPH_MAX_DIST):
    src_t = o3d.geometry.PointCloud(src)
    src_t.transform(T)
    return o3d.pipelines.registration.get_information_matrix_from_point_clouds(
        src_t, dst, max_dist, np.eye(4))


# ══════════════════════════════════════════════════════════════════════════════
#  Strategy A — Pose Graph
# ══════════════════════════════════════════════════════════════════════════════

def _build_pose_graph(pcds, log):
    n    = len(pcds)
    pg   = o3d.pipelines.registration.PoseGraph()
    odom = np.eye(4)
    pg.nodes.append(o3d.pipelines.registration.PoseGraphNode(odom))
    T_chain = {}

    for i in range(n - 1):
        j = i + 1
        print(f"  [reg] Pair {i}→{j}")
        T, fit, rmse = pairwise(pcds[i], pcds[j])
        T_chain[(i, j)] = T
        print(f"    fit={fit:.4f}  rmse={rmse*1000:.2f}mm")
        log.append(f"{i}-{j}  {fit:.4f}  {rmse*1000:.3f}mm")
        if fit < 0.10:
            print(f"    WARNING: low fitness")

        odom = T @ odom
        pg.nodes.append(o3d.pipelines.registration.PoseGraphNode(np.linalg.inv(odom)))
        info = _info_matrix(pcds[i], pcds[j], T)
        pg.edges.append(o3d.pipelines.registration.PoseGraphEdge(
            i, j, T, info, uncertain=False))

    # Loop closure: view 0 → each view i
    T_cum = np.eye(4)
    for i in range(1, n):
        T_cum = T_chain[(i-1, i)] @ T_cum
        T_0i  = np.linalg.inv(T_cum)
        # Only add if there is enough overlap
        info  = _info_matrix(pcds[0], pcds[i], T_0i)
        pg.edges.append(o3d.pipelines.registration.PoseGraphEdge(
            0, i, T_0i, info, uncertain=True))

    return pg


def _optimize_pose_graph(pg):
    print("  [reg] Pose Graph global optimization ...")
    o3d.pipelines.registration.global_optimization(
        pg,
        o3d.pipelines.registration.GlobalOptimizationLevenbergMarquardt(),
        o3d.pipelines.registration.GlobalOptimizationConvergenceCriteria(),
        o3d.pipelines.registration.GlobalOptimizationOption(
            max_correspondence_distance=POSEGRAPH_MAX_DIST,
            edge_prune_threshold=POSEGRAPH_EDGE_PRUNE,
            reference_node=0,
        ),
    )
    return [node.pose for node in pg.nodes]


def register_pose_graph(pcds, log):
    """Full Pose Graph pipeline. Returns list of 4×4 world poses."""
    pg    = _build_pose_graph(pcds, log)
    poses = _optimize_pose_graph(pg)
    return poses


# ══════════════════════════════════════════════════════════════════════════════
#  Strategy B — Frame-to-model
# ══════════════════════════════════════════════════════════════════════════════

def _extract_model_pcd(volume):
    """Extract current TSDF surface as PointCloud for ICP target."""
    pcd = volume.extract_point_cloud()
    pcd, _ = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
    return pcd


def _ftm_register(new_pcd, model_pcd, T_prev):
    """
    Register new_pcd against model_pcd via:
      1. Geometric ICP initialized from T_prev (small inter-frame motion assumed)
      2. Colored ICP fine refinement
    Returns (T_world, fitness, rmse, ok).
    """
    try:
        res_g = _icp_geom(new_pcd, model_pcd, T_init=T_prev, max_dist=0.05)
        res_c = _icp_colored(new_pcd, model_pcd, T_init=res_g.transformation)
        ok = res_c.fitness >= FTM_MIN_FITNESS
        return res_c.transformation, res_c.fitness, res_c.inlier_rmse, ok
    except RuntimeError:
        return T_prev, 0.0, 0.0, False


def register_frame_to_model(pcds, depths, colors, intrinsic,
                             roi_in_depth=False, log=None):
    """
    Frame-to-model tracking pipeline.

    1. Warmup: register first FTM_WARMUP_FRAMES via Pose Graph, build initial TSDF
    2. Subsequent frames: ICP against current TSDF point cloud
       - fallback to RANSAC pairwise if fitness too low

    Returns list of 4×4 world poses.
    """
    if log is None:
        log = []
    n = len(pcds)

    if n <= FTM_WARMUP_FRAMES:
        print(f"  [reg] Only {n} views, using Pose Graph only")
        return register_pose_graph(pcds, log)

    # ── Warmup phase ──────────────────────────────────────────────────────
    warmup_pcds = pcds[:FTM_WARMUP_FRAMES]
    print(f"  [reg] Warmup: Pose Graph on first {FTM_WARMUP_FRAMES} views ...")
    warmup_log  = []
    warmup_poses = register_pose_graph(warmup_pcds, warmup_log)
    log.extend(warmup_log)

    # Build initial TSDF from warmup frames
    warmup_depths  = depths[:FTM_WARMUP_FRAMES]
    warmup_colors  = colors[:FTM_WARMUP_FRAMES]
    warmup_pcds_   = pcds[:FTM_WARMUP_FRAMES]
    volume = fuse.integrate(warmup_depths, warmup_colors, warmup_poses,
                            intrinsic, pcds=warmup_pcds_,
                            roi_in_depth=roi_in_depth)
    model_pcd = _extract_model_pcd(volume)
    print(f"  [reg] Initial model: {len(model_pcd.points):,} pts")

    all_poses = list(warmup_poses)
    T_prev    = warmup_poses[-1]  # last known pose as initialization

    # ── Frame-to-model phase ──────────────────────────────────────────────
    for i in range(FTM_WARMUP_FRAMES, n):
        print(f"\n  [reg] FTM view {i}/{n-1}")
        T, fit, rmse, ok = _ftm_register(pcds[i], model_pcd, T_prev)

        if not ok:
            # Drop frame: append None so pipeline.py's second integrate pass
            # skips this frame entirely (T_prev caused leak into final mesh).
            print(f"    FTM fitness={fit:.3f} < {FTM_MIN_FITNESS} — DROP")
            log.append(f"{i} FTM-DROP  {fit:.4f}  skip")
            all_poses.append(None)
            continue

        print(f"    FTM fit={fit:.4f}  rmse={rmse*1000:.2f}mm")
        log.append(f"{i} FTM  {fit:.4f}  {rmse*1000:.3f}mm")

        all_poses.append(T)
        T_prev = T

        # Update TSDF model with newly registered frame
        fuse.integrate_one(volume, depths[i], colors[i], T, intrinsic,
                           pcd=pcds[i], roi_in_depth=roi_in_depth)
        print(f"  [fuse] Integrated view {i}")
        # Re-extract model every frame (or every k frames for speed)
        model_pcd = _extract_model_pcd(volume)

    return all_poses


# ══════════════════════════════════════════════════════════════════════════════
#  Public API
# ══════════════════════════════════════════════════════════════════════════════

def register(pcds, depths=None, colors=None, intrinsic=None,
             roi_in_depth=False, use_ftm=True):
    """
    Estimate per-view world poses.

    Args:
        pcds:         list of ROI-cropped PointClouds
        depths:       list of depth arrays (required for frame-to-model)
        colors:       list of RGB arrays   (required for frame-to-model)
        intrinsic:    o3d.camera.PinholeCameraIntrinsic (required for FTM)
        roi_in_depth: whether depth.npy has NaN outside ROI
        use_ftm:      True → frame-to-model after warmup; False → Pose Graph only

    Returns:
        (poses, log_lines)
    """
    log = ["pair  method  fitness  rmse_mm"]

    if use_ftm and depths is not None and colors is not None and intrinsic is not None:
        poses = register_frame_to_model(pcds, depths, colors, intrinsic,
                                        roi_in_depth=roi_in_depth, log=log)
    else:
        poses = register_pose_graph(pcds, log)

    return poses, log
