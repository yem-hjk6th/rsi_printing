"""
mesh.py — Mesh post-processing for artec_imit pipeline.

Steps applied after TSDF Marching Cubes:
  1. Remove degenerate triangles (zero-area)
  2. Remove small disconnected components (< MIN_CLUSTER_FRAC of largest)
  3. Laplacian smoothing (configurable iterations)
  4. Recompute vertex normals

Usage (imported by pipeline.py):
    mesh_clean = process(mesh_raw)
    save(mesh_clean, path)
"""

import numpy as np
import open3d as o3d
from pathlib import Path

from config import SMOOTH_ITER, SMOOTH_LAMBDA, MIN_TRIANGLE_AREA, MIN_CLUSTER_FRAC


def _remove_small_clusters(mesh, min_frac=MIN_CLUSTER_FRAC):
    """Keep only connected components >= min_frac of the largest component."""
    triangle_clusters, cluster_n_triangles, _ = mesh.cluster_connected_triangles()
    triangle_clusters  = np.asarray(triangle_clusters)
    cluster_n_triangles = np.asarray(cluster_n_triangles)

    if len(cluster_n_triangles) == 0:
        return mesh

    max_n = cluster_n_triangles.max()
    threshold = max_n * min_frac

    # Mark triangles to remove (belong to small clusters)
    keep = cluster_n_triangles[triangle_clusters] >= threshold
    remove_mask = ~keep
    mesh.remove_triangles_by_mask(remove_mask.tolist())
    mesh.remove_unreferenced_vertices()
    mesh_clean = mesh
    n_removed = remove_mask.sum()
    if n_removed:
        print(f"  [mesh] Removed {n_removed:,} triangles in small clusters "
              f"(threshold={threshold:.0f} of {max_n:.0f})")
    return mesh_clean


def process(mesh):
    """
    Full post-processing pipeline:
      1. Degenerate triangle removal
      2. Small cluster removal
      3. Laplacian smoothing
      4. Normal recomputation

    Args:
        mesh: o3d.geometry.TriangleMesh (raw from TSDF extract)

    Returns:
        o3d.geometry.TriangleMesh (cleaned)
    """
    n0 = len(mesh.triangles)

    # 1. Degenerate triangles
    mesh = mesh.remove_degenerate_triangles()
    mesh = mesh.remove_duplicated_triangles()
    mesh = mesh.remove_duplicated_vertices()
    mesh = mesh.remove_non_manifold_edges()
    n1 = len(mesh.triangles)
    if n0 - n1:
        print(f"  [mesh] Removed {n0 - n1:,} degenerate/duplicate triangles")

    # 2. Small connected components
    mesh = _remove_small_clusters(mesh)

    # 3. Laplacian smoothing
    if SMOOTH_ITER > 0:
        mesh = mesh.filter_smooth_laplacian(
            number_of_iterations=SMOOTH_ITER,
            lambda_filter=SMOOTH_LAMBDA,
        )
        print(f"  [mesh] Laplacian smooth: {SMOOTH_ITER} iters, λ={SMOOTH_LAMBDA}")

    # 4. Normals
    mesh.compute_vertex_normals()

    print(f"  [mesh] Final: {len(mesh.vertices):,} verts, {len(mesh.triangles):,} tris")
    return mesh


def save(mesh, path):
    path = Path(path)
    o3d.io.write_triangle_mesh(str(path), mesh, write_vertex_colors=True)
    print(f"  [mesh] Saved → {path.name}")
