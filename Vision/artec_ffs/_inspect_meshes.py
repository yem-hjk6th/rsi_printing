"""Quick mesh/pcd quality inspection for the new dataset.

Usage: python _inspect_meshes.py <capture_dir>
       python _inspect_meshes.py        # uses DEFAULT below
"""
import sys
import open3d as o3d
import numpy as np
from pathlib import Path

DEFAULT = Path(__file__).resolve().parent.parent / "vision_demo_test_res" / "ffs_20260508_160056"
base = Path(sys.argv[1]).resolve() if len(sys.argv) > 1 else DEFAULT

def stats(name, sub):
    print(f"\n=== {name} ({sub}) ===")
    mp = base / sub / "mesh.ply"
    pp = base / sub / "pcd.ply"
    mesh = o3d.io.read_triangle_mesh(str(mp))
    pcd  = o3d.io.read_point_cloud(str(pp))
    nv, nt = len(mesh.vertices), len(mesh.triangles)
    print(f"  mesh: {nv:,} verts  {nt:,} tris")
    print(f"  watertight={mesh.is_watertight()}  edge_manifold={mesh.is_edge_manifold(allow_boundary_edges=True)}  vertex_manifold={mesh.is_vertex_manifold()}")
    bb = mesh.get_axis_aligned_bounding_box()
    ext = bb.get_extent()
    print(f"  mesh bbox extent (mm): x={ext[0]*1000:.1f} y={ext[1]*1000:.1f} z={ext[2]*1000:.1f}")
    if nv:
        verts = np.asarray(mesh.vertices)
        print(f"  vert range x[{verts[:,0].min()*1000:.0f},{verts[:,0].max()*1000:.0f}] y[{verts[:,1].min()*1000:.0f},{verts[:,1].max()*1000:.0f}] z[{verts[:,2].min()*1000:.0f},{verts[:,2].max()*1000:.0f}] mm")
    pts = np.asarray(pcd.points)
    print(f"  pcd: {len(pts):,} pts")
    cc, ncc, _ = mesh.cluster_connected_triangles()
    ncc = np.asarray(ncc)
    sizes = sorted(ncc.tolist(), reverse=True)
    print(f"  components: {len(ncc)}  top5 tri counts: {sizes[:5]}")
    if len(sizes) > 1:
        print(f"  largest/total: {sizes[0]/sum(sizes)*100:.1f}%   2nd-cluster size: {sizes[1]:,} tris")
    # avg edge length proxy for resolution
    if nt:
        tris = np.asarray(mesh.triangles)
        verts_a = np.asarray(mesh.vertices)
        v0, v1, v2 = verts_a[tris[:,0]], verts_a[tris[:,1]], verts_a[tris[:,2]]
        e = np.concatenate([np.linalg.norm(v1-v0,axis=1), np.linalg.norm(v2-v1,axis=1), np.linalg.norm(v0-v2,axis=1)])
        print(f"  edge len mm  median={np.median(e)*1000:.2f}  p10={np.percentile(e,10)*1000:.2f}  p90={np.percentile(e,90)*1000:.2f}")

stats("v1 ZED", "artec_recon_v1")
stats("v2 FFS", "artec_recon_v2")

# Per-view depth stats for FFS dataset
print("\n=== Per-view FFS vs ZED depth (new dataset) ===")
for i in [0, 5, 10, 13, 15, 19, 20, 25, 29]:
    df = base / f"view_{i:03d}_depth.npy"
    zf = base / f"view_{i:03d}_depth_zed.npy"
    if not (df.exists() and zf.exists()):
        continue
    d = np.load(df); z = np.load(zf)
    dv = d[np.isfinite(d)]; zv = z[np.isfinite(z)]
    if len(dv) == 0 or len(zv) == 0:
        print(f"  view {i:02d}: empty"); continue
    dp = np.percentile(dv, [10, 50, 90])
    zp = np.percentile(zv, [10, 50, 90])
    print(f"  view {i:02d}  FFS pts={len(dv):>9,d}  p10/50/90={dp[0]:.3f}/{dp[1]:.3f}/{dp[2]:.3f}  | ZED pts={len(zv):>9,d}  p10/50/90={zp[0]:.3f}/{zp[1]:.3f}/{zp[2]:.3f}")
