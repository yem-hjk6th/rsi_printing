"""
mesh.py — Mesh extraction + post-processing for ffs_poisson_detail.

Two backends, selected by config.MESH_BACKEND:

  "marching_cubes"  — original artec_ffs behaviour: iso-surface straight out
                      of the TSDF volume, then cleanup + Laplacian smooth.

  "poisson"         — Screened Poisson Surface Reconstruction (Kazhdan & Hoppe,
                      ACM ToG 2013) on the fused, normal-bearing point cloud.
                      The screened formulation pins the surface to the input
                      samples instead of letting TSDF averaging shrink fine
                      relief away — this is the detail-recovery win.

Public API (called by pipeline.py):
    mesh_clean = build(mesh_raw, pcd)   # dispatches on MESH_BACKEND
    save(mesh_clean, path)

`build` keeps the artec_ffs `process(mesh)` semantics for the marching-cubes
path; the Poisson path ignores `mesh_raw` and consumes `pcd` instead.
"""

import numpy as np
import open3d as o3d
from pathlib import Path

from config import (
    SMOOTH_ITER, SMOOTH_LAMBDA, MIN_CLUSTER_FRAC,
    MESH_BACKEND, POISSON_DEPTH, POISSON_SCALE, POISSON_DENSITY_QUANTILE,
)


# ══════════════════════════════════════════════════════════════════════════════
#  Shared cleanup
# ══════════════════════════════════════════════════════════════════════════════

def _remove_small_clusters(mesh, min_frac=MIN_CLUSTER_FRAC):
    """Keep only connected components >= min_frac of the largest component."""
    triangle_clusters, cluster_n_triangles, _ = mesh.cluster_connected_triangles()
    triangle_clusters   = np.asarray(triangle_clusters)
    cluster_n_triangles = np.asarray(cluster_n_triangles)

    if len(cluster_n_triangles) == 0:
        return mesh

    max_n     = cluster_n_triangles.max()
    threshold = max_n * min_frac
    remove_mask = cluster_n_triangles[triangle_clusters] < threshold
    mesh.remove_triangles_by_mask(remove_mask.tolist())
    mesh.remove_unreferenced_vertices()
    n_removed = int(remove_mask.sum())
    if n_removed:
        print(f"  [mesh] Removed {n_removed:,} triangles in small clusters "
              f"(threshold={threshold:.0f} of {max_n:.0f})")
    return mesh


def _cleanup(mesh):
    """Degenerate/duplicate removal shared by both backends."""
    n0 = len(mesh.triangles)
    mesh = mesh.remove_degenerate_triangles()
    mesh = mesh.remove_duplicated_triangles()
    mesh = mesh.remove_duplicated_vertices()
    mesh = mesh.remove_non_manifold_edges()
    n1 = len(mesh.triangles)
    if n0 - n1:
        print(f"  [mesh] Removed {n0 - n1:,} degenerate/duplicate triangles")
    return _remove_small_clusters(mesh)


def _smooth(mesh):
    if SMOOTH_ITER > 0:
        mesh = mesh.filter_smooth_laplacian(
            number_of_iterations=SMOOTH_ITER,
            lambda_filter=SMOOTH_LAMBDA,
        )
        print(f"  [mesh] Laplacian smooth: {SMOOTH_ITER} iters, lambda={SMOOTH_LAMBDA}")
    mesh.compute_vertex_normals()
    return mesh


# ══════════════════════════════════════════════════════════════════════════════
#  Backend: Marching Cubes  (original artec_ffs behaviour)
# ══════════════════════════════════════════════════════════════════════════════

def _build_marching_cubes(mesh_raw):
    mesh = _cleanup(mesh_raw)
    mesh = _smooth(mesh)
    print(f"  [mesh] Final (marching_cubes): {len(mesh.vertices):,} verts, "
          f"{len(mesh.triangles):,} tris")
    return mesh


# ══════════════════════════════════════════════════════════════════════════════
#  Backend: Screened Poisson
# ══════════════════════════════════════════════════════════════════════════════

def _build_poisson(pcd):
    """
    Screened Poisson reconstruction on the fused point cloud.

    Requires oriented normals. The TSDF point cloud from fuse.extract() already
    carries normals derived from the volume's SDF gradient — consistent and
    well-oriented, better than estimating from scratch. If they are somehow
    missing we estimate + orient as a fallback.
    """
    if not pcd.has_normals():
        print("  [mesh] pcd has no normals — estimating + orienting")
        pcd.estimate_normals(
            o3d.geometry.KDTreeSearchParamHybrid(radius=0.01, max_nn=30))
        pcd.orient_normals_consistent_tangent_plane(30)

    print(f"  [mesh] Screened Poisson: depth={POISSON_DEPTH}, scale={POISSON_SCALE}, "
          f"input={len(pcd.points):,} pts")
    mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
        pcd, depth=POISSON_DEPTH, scale=POISSON_SCALE, linear_fit=False,
    )
    densities = np.asarray(densities)

    # Trim "balloon" surface hallucinated in unscanned regions: vertices whose
    # underlying sample density is in the lowest POISSON_DENSITY_QUANTILE.
    if POISSON_DENSITY_QUANTILE > 0.0 and len(densities) > 0:
        cutoff = np.quantile(densities, POISSON_DENSITY_QUANTILE)
        low    = densities < cutoff
        mesh.remove_vertices_by_mask(low.tolist())
        print(f"  [mesh] Trimmed {int(low.sum()):,} low-density verts "
              f"(quantile={POISSON_DENSITY_QUANTILE}, cutoff={cutoff:.2f})")

    mesh = _cleanup(mesh)
    mesh = _smooth(mesh)
    print(f"  [mesh] Final (poisson): {len(mesh.vertices):,} verts, "
          f"{len(mesh.triangles):,} tris")
    return mesh


# ══════════════════════════════════════════════════════════════════════════════
#  Dispatch
# ══════════════════════════════════════════════════════════════════════════════

def build(mesh_raw, pcd):
    """
    Build the final mesh using the backend selected in config.MESH_BACKEND.

    Args:
        mesh_raw: o3d.geometry.TriangleMesh — raw TSDF marching-cubes mesh
                  (used by the "marching_cubes" backend, ignored by "poisson")
        pcd:      o3d.geometry.PointCloud — fused TSDF point cloud with normals
                  (used by the "poisson" backend, ignored by "marching_cubes")

    Returns:
        o3d.geometry.TriangleMesh
    """
    if MESH_BACKEND == "poisson":
        return _build_poisson(pcd)
    elif MESH_BACKEND == "marching_cubes":
        return _build_marching_cubes(mesh_raw)
    else:
        raise ValueError(
            f"Unknown MESH_BACKEND={MESH_BACKEND!r} — "
            f'expected "poisson" or "marching_cubes"')


# Back-compat: artec_ffs called this `process(mesh)`. Keep a thin shim so any
# old caller still works (marching-cubes path only).
def process(mesh):
    return _build_marching_cubes(mesh)


def save(mesh, path):
    path = Path(path)
    o3d.io.write_triangle_mesh(str(path), mesh, write_vertex_colors=True)
    print(f"  [mesh] Saved -> {path.name}")
