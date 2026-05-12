"""
fuse.py — TSDF volume integration for artec_imit pipeline.

Consumes per-view (depth.npy, color.png, PLY) produced by capture.py and
a list of optimized 4×4 world poses from register.py.

Key improvements over demo12:
  - depth_max = 1.2m (config.py) — cuts far-field noise
  - Exact ROI mask: uses depth.npy NaN pattern if roi_in_depth=True (new data),
    otherwise falls back to PLY back-projection bbox (legacy demo11 data)
  - Configurable ROI_MARGIN_PX for bbox fallback

Usage (imported by pipeline.py or register.py):
    volume = integrate(depths, colors, poses, intrinsic, pcds=pcds)
    mesh, pcd = extract(volume)
"""

import numpy as np
import open3d as o3d

from config import (
    TSDF_VOXEL, TSDF_TRUNC, DEPTH_MAX_M,
    ROI_MARGIN_PX, OUTLIER_NB, OUTLIER_STD,
)


# ══════════════════════════════════════════════════════════════════════════════
#  ROI masking
# ══════════════════════════════════════════════════════════════════════════════

def _mask_from_nan(depth_m):
    """ROI mask from NaN pattern in depth.npy (new data from capture.py)."""
    return np.isfinite(depth_m)


def _mask_from_pcd_bbox(depth_m, pcd, fx, fy, cx, cy, margin=ROI_MARGIN_PX):
    """
    ROI mask from PLY back-projection bounding box.
    Fallback for legacy data (demo11) where depth.npy was saved full-frame.
    """
    pts = np.asarray(pcd.points)
    if len(pts) == 0:
        return np.ones(depth_m.shape, dtype=bool)
    H, W = depth_m.shape
    u = (fx * pts[:, 0] / pts[:, 2] + cx).astype(int)
    v = (fy * pts[:, 1] / pts[:, 2] + cy).astype(int)
    u1 = max(0,  int(u.min()) - margin)
    u2 = min(W,  int(u.max()) + margin)
    v1 = max(0,  int(v.min()) - margin)
    v2 = min(H,  int(v.max()) + margin)
    mask = np.zeros((H, W), dtype=bool)
    mask[v1:v2, u1:u2] = True
    return mask


def apply_roi_mask(depth_m, pcd, intrinsic, roi_in_depth=False):
    """
    Return depth_m with background zeroed out.
    roi_in_depth=True  → NaN pattern in depth is the exact ROI (new data)
    roi_in_depth=False → fall back to PLY bbox (legacy data)
    """
    fx, fy = intrinsic.get_focal_length()
    cx, cy = intrinsic.get_principal_point()

    if roi_in_depth:
        mask = _mask_from_nan(depth_m)
    else:
        mask = _mask_from_pcd_bbox(depth_m, pcd, fx, fy, cx, cy)

    masked = depth_m.copy()
    masked[~mask] = np.nan
    return masked


# ══════════════════════════════════════════════════════════════════════════════
#  TSDF integration
# ══════════════════════════════════════════════════════════════════════════════

def integrate(depths, colors, poses, intrinsic, pcds=None, roi_in_depth=False):
    """
    Integrate RGBD keyframes into a ScalableTSDFVolume.

    Args:
        depths:       list of (H,W) float32 arrays, meters
        colors:       list of (H,W,3) uint8 RGB arrays
        poses:        list of 4×4 ndarray — camera-in-world transforms
                      (pose graph node poses, i.e. inv of extrinsic)
        intrinsic:    o3d.camera.PinholeCameraIntrinsic
        pcds:         list of ROI-cropped PointClouds (used for legacy bbox mask)
        roi_in_depth: True if depth.npy already has NaN outside ROI (new data)

    Returns:
        o3d.pipelines.integration.ScalableTSDFVolume
    """
    volume = o3d.pipelines.integration.ScalableTSDFVolume(
        voxel_length=TSDF_VOXEL,
        sdf_trunc=TSDF_TRUNC,
        color_type=o3d.pipelines.integration.TSDFVolumeColorType.RGB8,
    )

    for i, (depth_m, color_rgb, T_world) in enumerate(zip(depths, colors, poses)):
        if T_world is None:
            continue  # dropped frame — skip to avoid stacking noise at T_prev pose
        # Apply ROI mask
        if pcds is not None or roi_in_depth:
            pcd_i = pcds[i] if pcds is not None else None
            depth_m = apply_roi_mask(depth_m, pcd_i, intrinsic, roi_in_depth)

        depth_clean = np.nan_to_num(depth_m, nan=0.0).astype(np.float32)

        rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
            o3d.geometry.Image(color_rgb.astype(np.uint8)),
            o3d.geometry.Image(depth_clean),
            depth_scale=1.0,
            depth_trunc=DEPTH_MAX_M,
            convert_rgb_to_intensity=False,
        )
        volume.integrate(rgbd, intrinsic, np.linalg.inv(T_world))
        print(f"  [fuse] Integrated view {i}")

    return volume


def integrate_one(volume, depth_m, color_rgb, T_world, intrinsic,
                  pcd=None, roi_in_depth=False):
    """Integrate a single frame into an existing TSDF volume (for FTM incremental update)."""
    depth_m = apply_roi_mask(depth_m, pcd, intrinsic, roi_in_depth)
    depth_clean = np.nan_to_num(depth_m, nan=0.0).astype(np.float32)
    rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
        o3d.geometry.Image(color_rgb.astype(np.uint8)),
        o3d.geometry.Image(depth_clean),
        depth_scale=1.0,
        depth_trunc=DEPTH_MAX_M,
        convert_rgb_to_intensity=False,
    )
    volume.integrate(rgbd, intrinsic, np.linalg.inv(T_world))


# ══════════════════════════════════════════════════════════════════════════════
#  Extract mesh and point cloud from TSDF
# ══════════════════════════════════════════════════════════════════════════════

def extract(volume):
    """
    Extract triangle mesh and point cloud from TSDF volume.

    Returns:
        mesh:  o3d.geometry.TriangleMesh  (vertex normals computed)
        pcd:   o3d.geometry.PointCloud    (statistical outlier removed)
    """
    mesh = volume.extract_triangle_mesh()
    mesh.compute_vertex_normals()

    pcd_raw = volume.extract_point_cloud()
    pcd, _  = pcd_raw.remove_statistical_outlier(
        nb_neighbors=OUTLIER_NB, std_ratio=OUTLIER_STD)

    return mesh, pcd
