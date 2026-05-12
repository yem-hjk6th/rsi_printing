"""
demo7_call_ffs_output_mesh.py — Capture 3 frames → FFS depth → PLY point cloud + mesh.

Pipeline:
  1. Open ZED 2i HD2K, grab 3 rectified stereo pairs
  2. Call FFS (ffs conda env) for disparity → depth
  3. Median-fuse 3 depth maps, mask non-overlap region
  4. Back-project valid depth to 3D point cloud with color
  5. Export PLY point cloud (fast) + mesh via Open3D Poisson reconstruction
  6. Report timing for each stage

Output (in temp dir):
  - pointcloud.ply    — colored point cloud (Rhino-readable)
  - mesh.ply          — Poisson surface mesh (Rhino-readable)

Environment: ffs (conda)
Keys:
  s : capture frames → FFS → mesh pipeline
  i/I : decrease/increase FFS iters (default 8)
  n/N : decrease/increase frame count (default 3)
  q : quit
"""
import sys, os, subprocess, time, tempfile
from pathlib import Path

import cv2
import numpy as np

# ── paths ──
SCRIPT_DIR = Path(__file__).resolve().parent
VISION_DIR = SCRIPT_DIR.parent
REPO_DIR = VISION_DIR.parent.parent / "Repo"
FFS_DIR = REPO_DIR / "ffs"
FFS_RUN_SCRIPT = FFS_DIR / "run_depth_images.py"
FFS_CONDA_ENV = "ffs"
CONDA_EXE = r"C:\Users\888y9\miniconda3\Scripts\conda.exe"
OUTPUT_ROOT = Path(r"C:\Users\888y9\Desktop\rsi_printing\Vision\vision_demo_test_res")

# ── ZED setup ──
sys.path.insert(0, str(VISION_DIR))
import zed_setup  # noqa: E402
import pyzed.sl as sl


# ============================================================
#  Stage 1: Capture
# ============================================================

def grab_frames(zed, n_frames=3, settle_skip=5):
    """Grab n_frames rectified stereo pairs. Skip first few for AE/AWB."""
    left_mat, right_mat = sl.Mat(), sl.Mat()
    runtime = sl.RuntimeParameters()
    for _ in range(settle_skip):
        zed.grab(runtime)

    pairs = []
    for i in range(n_frames):
        if zed.grab(runtime) != sl.ERROR_CODE.SUCCESS:
            print(f"  grab failed on frame {i}")
            continue
        zed.retrieve_image(left_mat, sl.VIEW.LEFT)
        zed.retrieve_image(right_mat, sl.VIEW.RIGHT)
        pairs.append((left_mat.get_data()[:, :, :3].copy(),
                       right_mat.get_data()[:, :, :3].copy()))
        print(f"  captured frame {i+1}/{n_frames}")
    return pairs


def save_frames_and_intrinsics(pairs, calib, out_dir):
    """Save stereo pairs as PNG + K.txt."""
    left_dir = out_dir / "left"
    right_dir = out_dir / "right"
    left_dir.mkdir(parents=True, exist_ok=True)
    right_dir.mkdir(parents=True, exist_ok=True)

    for i, (l, r) in enumerate(pairs):
        cv2.imwrite(str(left_dir / f"frame_{i:03d}.png"), l)
        cv2.imwrite(str(right_dir / f"frame_{i:03d}.png"), r)

    fx = calib.left_cam.fx
    fy = calib.left_cam.fy
    cx = calib.left_cam.cx
    cy = calib.left_cam.cy
    baseline_m = calib.get_camera_baseline() / 1000.0

    k_path = out_dir / "K.txt"
    k_path.write_text(f"{fx} 0 {cx} 0 {fy} {cy} 0 0 1\n{baseline_m:.6f}\n")
    print(f"  saved {len(pairs)} pairs + K.txt to {out_dir}")
    return k_path, (fx, fy, cx, cy)


# ============================================================
#  Stage 2: FFS Inference
# ============================================================

def run_ffs(out_dir, k_path, scale=1.0, valid_iters=8):
    """Call FFS via conda run. Returns (result_dir, elapsed_seconds) or (None, 0)."""
    result_dir = out_dir / "ffs_output"
    cmd = [
        CONDA_EXE, "run", "-n", FFS_CONDA_ENV,
        "python", str(FFS_RUN_SCRIPT),
        "--left_dir", str(out_dir / "left"),
        "--right_dir", str(out_dir / "right"),
        "--intrinsic_file", str(k_path),
        "--out_dir", str(result_dir),
        "--scale", str(scale),
        "--valid_iters", str(valid_iters),
        "--save_npy",
    ]
    print(f"  running FFS (scale={scale}, iters={valid_iters}) ...")
    t0 = time.perf_counter()
    proc = subprocess.run(cmd, capture_output=True, text=True)
    elapsed = time.perf_counter() - t0

    if proc.returncode != 0:
        print(f"  FFS failed (exit {proc.returncode}):")
        print(proc.stderr[-2000:] if proc.stderr else "(no stderr)")
        return None, elapsed
    print(f"  FFS inference done in {elapsed:.1f}s")
    return result_dir, elapsed


# ============================================================
#  Stage 3: Fusion + Overlap Mask
# ============================================================

def load_and_fuse(result_dir, n_frames):
    """Load depth + disp, median-fuse, mask non-overlap region."""
    depths, disps = [], []
    for i in range(n_frames):
        dp = result_dir / f"frame_{i:03d}_depth.npy"
        sp = result_dir / f"frame_{i:03d}_disp.npy"
        if dp.exists():
            depths.append(np.load(str(dp)))
        if sp.exists():
            disps.append(np.load(str(sp)))
    if not depths:
        print("  no depth files found!")
        return None, None

    if len(depths) == 1:
        depth, disp = depths[0], (disps[0] if disps else None)
    else:
        depth = np.median(np.stack(depths, axis=0), axis=0).astype(np.float32)
        disp = np.median(np.stack(disps, axis=0), axis=0).astype(np.float32) if disps else None
        print(f"  fused {len(depths)} frames via median")

    if disp is not None:
        H, W = disp.shape
        col_idx = np.arange(W)[np.newaxis, :]
        invalid = disp > col_idx
        n_masked = invalid.sum()
        depth[invalid] = np.nan
        print(f"  masked {n_masked} invalid pixels ({100*n_masked/(H*W):.1f}%)")

    return depth, disp


# ============================================================
#  Stage 4: Depth → 3D Point Cloud (back-projection)
# ============================================================

def depth_to_points(depth, color_bgr, fx, fy, cx, cy, depth_max=10.0):
    """Back-project depth map to colored 3D point cloud.

    Returns (points_Nx3, colors_Nx3_uint8) with NaN/invalid filtered out.
    """
    H, W = depth.shape
    valid = np.isfinite(depth) & (depth > 0) & (depth < depth_max)

    u, v = np.meshgrid(np.arange(W), np.arange(H))
    z = depth[valid]
    x = (u[valid] - cx) * z / fx
    y = (v[valid] - cy) * z / fy

    points = np.stack([x, y, z], axis=-1).astype(np.float32)
    colors = cv2.cvtColor(color_bgr, cv2.COLOR_BGR2RGB)[valid]  # Nx3 uint8
    return points, colors


# ============================================================
#  Stage 5: Export PLY + Mesh  <<<< MESH MODULE >>>>
# ============================================================
#
# PLY point cloud: written directly with numpy (fastest, no deps)
# Mesh: Open3D Poisson reconstruction
#   - Requires normals estimation → then poisson_surface_reconstruction
#   - Output: mesh.ply (triangle mesh, Rhino-importable)
#

def write_ply_pointcloud(path, points, colors):
    """Write colored point cloud to binary PLY. Fast, no external deps."""
    N = len(points)
    header = (
        "ply\n"
        "format binary_little_endian 1.0\n"
        f"element vertex {N}\n"
        "property float x\n"
        "property float y\n"
        "property float z\n"
        "property uchar red\n"
        "property uchar green\n"
        "property uchar blue\n"
        "end_header\n"
    )
    # pack: 3 floats + 3 bytes per vertex
    dtype = np.dtype([('x', '<f4'), ('y', '<f4'), ('z', '<f4'),
                      ('r', 'u1'), ('g', 'u1'), ('b', 'u1')])
    arr = np.empty(N, dtype=dtype)
    arr['x'] = points[:, 0]
    arr['y'] = points[:, 1]
    arr['z'] = points[:, 2]
    arr['r'] = colors[:, 0]
    arr['g'] = colors[:, 1]
    arr['b'] = colors[:, 2]

    with open(path, 'wb') as f:
        f.write(header.encode('ascii'))
        f.write(arr.tobytes())
    print(f"  PLY point cloud: {N:,} points → {path}")


def build_mesh_from_points(points, colors, out_path, depth=9, density_quantile=0.01):
    """
    <<<< MESH MODULE: Open3D Poisson Reconstruction >>>>
    (Legacy — poor for single-viewpoint, kept for reference)
    """
    import open3d as o3d

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points.astype(np.float64))
    pcd.colors = o3d.utility.Vector3dVector(colors.astype(np.float64) / 255.0)

    voxel_size = 0.005
    pcd_down = pcd.voxel_down_sample(voxel_size)
    n_down = len(pcd_down.points)
    print(f"  downsampled: {len(pcd.points):,} → {n_down:,} points (voxel={voxel_size*1000:.0f}mm)")

    pcd_down.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.02, max_nn=30))
    pcd_down.orient_normals_towards_camera_location(camera_location=np.array([0.0, 0.0, 0.0]))

    print(f"  running Poisson reconstruction (depth={depth}) ...")
    mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
        pcd_down, depth=depth, linear_fit=False
    )

    densities = np.asarray(densities)
    threshold = np.quantile(densities, density_quantile)
    vertices_to_remove = densities < threshold
    mesh.remove_vertices_by_mask(vertices_to_remove)
    mesh.compute_vertex_normals()

    n_verts = len(mesh.vertices)
    n_tris = len(mesh.triangles)
    print(f"  mesh: {n_verts:,} vertices, {n_tris:,} triangles")

    o3d.io.write_triangle_mesh(str(out_path), mesh, write_vertex_colors=True)
    print(f"  mesh PLY saved → {out_path}")

    obj_path = str(out_path).replace(".ply", ".obj")
    o3d.io.write_triangle_mesh(obj_path, mesh, write_vertex_colors=True)
    print(f"  mesh OBJ saved → {obj_path}")

    return mesh


def build_organized_mesh(depth_map, color_bgr, fx, fy, cx, cy, out_path,
                         depth_max=10.0, depth_disc_thresh=0.05, step=1):
    """
    <<<< MESH MODULE: Organized Depth-Image Mesh >>>>

    Correct method for single-viewpoint depth:
      1. Treat depth map as a 2D grid
      2. Each 2x2 pixel quad → 2 triangles
      3. Skip triangles where adjacent depth difference > threshold (discontinuity)
      4. Export as PLY + OBJ with vertex colors

    Args:
        depth_disc_thresh: relative depth discontinuity threshold.
            Triangle is rejected if max_depth/min_depth > (1 + thresh).
            0.05 = 5% depth jump → break mesh.
        step: pixel stride (1=full res, 2=half, etc.)
    """
    import open3d as o3d

    H, W = depth_map.shape
    color_rgb = cv2.cvtColor(color_bgr, cv2.COLOR_BGR2RGB)

    # subsample grid
    rows = np.arange(0, H, step)
    cols = np.arange(0, W, step)
    Nr, Nc = len(rows), len(cols)

    # back-project all grid points
    uu, vv = np.meshgrid(cols, rows)  # (Nr, Nc)
    zz = depth_map[vv, uu]

    valid = np.isfinite(zz) & (zz > 0) & (zz < depth_max)
    xx = (uu - cx) * zz / fx
    yy = (vv - cy) * zz / fy

    # vertex array (Nr*Nc, 3) — invalid points get NaN
    verts = np.stack([xx, yy, zz], axis=-1).reshape(-1, 3).astype(np.float64)
    vert_colors = color_rgb[vv, uu].reshape(-1, 3).astype(np.float64) / 255.0
    valid_flat = valid.reshape(-1)

    # build triangle indices for each 2x2 quad
    # vertex index at grid (r, c) = r * Nc + c
    r_idx = np.arange(Nr - 1)
    c_idx = np.arange(Nc - 1)
    rr, cc = np.meshgrid(r_idx, c_idx, indexing='ij')
    rr = rr.ravel()
    cc = cc.ravel()

    # four corners of each quad
    i00 = rr * Nc + cc          # top-left
    i01 = rr * Nc + (cc + 1)    # top-right
    i10 = (rr + 1) * Nc + cc    # bottom-left
    i11 = (rr + 1) * Nc + (cc + 1)  # bottom-right

    # triangle 1: i00, i10, i01  (top-left, bottom-left, top-right)
    # triangle 2: i01, i10, i11  (top-right, bottom-left, bottom-right)
    tri1 = np.stack([i00, i10, i01], axis=-1)
    tri2 = np.stack([i01, i10, i11], axis=-1)

    def filter_triangles(tris):
        """Remove triangles with invalid vertices or depth discontinuities."""
        v0, v1, v2 = tris[:, 0], tris[:, 1], tris[:, 2]
        # all three vertices must be valid
        mask = valid_flat[v0] & valid_flat[v1] & valid_flat[v2]

        # depth discontinuity check
        z0 = zz.ravel()[v0]
        z1 = zz.ravel()[v1]
        z2 = zz.ravel()[v2]
        z_stack = np.stack([z0, z1, z2], axis=-1)
        z_min = z_stack.min(axis=-1)
        z_max = z_stack.max(axis=-1)
        # reject if max/min ratio exceeds threshold
        ratio_ok = z_max < z_min * (1.0 + depth_disc_thresh)
        mask &= ratio_ok

        return tris[mask]

    tri1_filt = filter_triangles(tri1)
    tri2_filt = filter_triangles(tri2)
    all_tris = np.concatenate([tri1_filt, tri2_filt], axis=0)

    print(f"  organized mesh: {Nr}x{Nc} grid, step={step}")
    print(f"  triangles: {len(all_tris):,} (from {len(tri1)+len(tri2):,} candidates)")
    print(f"  depth discontinuity threshold: {depth_disc_thresh*100:.0f}%")

    # build Open3D mesh
    mesh = o3d.geometry.TriangleMesh()
    mesh.vertices = o3d.utility.Vector3dVector(verts)
    mesh.vertex_colors = o3d.utility.Vector3dVector(vert_colors)
    mesh.triangles = o3d.utility.Vector3iVector(all_tris.astype(np.int32))

    # remove unreferenced vertices (invalid ones)
    mesh.remove_unreferenced_vertices()
    mesh.compute_vertex_normals()

    n_verts = len(mesh.vertices)
    n_tris = len(mesh.triangles)
    print(f"  final: {n_verts:,} vertices, {n_tris:,} triangles")

    # save PLY
    o3d.io.write_triangle_mesh(str(out_path), mesh, write_vertex_colors=True)
    print(f"  mesh PLY saved → {out_path}")

    # save OBJ
    obj_path = str(out_path).replace(".ply", ".obj")
    o3d.io.write_triangle_mesh(obj_path, mesh, write_vertex_colors=True)
    print(f"  mesh OBJ saved → {obj_path}")

    return mesh


# ============================================================
#  Visualization
# ============================================================

def depth_to_colormap(depth, vmin=0.2, vmax=5.0):
    """Depth (meters) → color image. NaN → black."""
    mask = np.isnan(depth)
    d = np.clip(np.nan_to_num(depth, nan=vmin), vmin, vmax)
    d = ((d - vmin) / (vmax - vmin) * 255).astype(np.uint8)
    vis = cv2.applyColorMap(d, cv2.COLORMAP_TURBO)
    vis[mask] = 0
    return vis


# ============================================================
#  Main
# ============================================================

def main():
    zed = sl.Camera()
    init = sl.InitParameters()
    init.camera_resolution = sl.RESOLUTION.HD2K
    init.camera_fps = 15
    init.depth_mode = sl.DEPTH_MODE.NONE

    if zed.open(init) != sl.ERROR_CODE.SUCCESS:
        print("Failed to open ZED camera")
        return

    info = zed.get_camera_information()
    calib = info.camera_configuration.calibration_parameters
    res = info.camera_configuration.resolution
    print(f"ZED 2i opened: {res.width}x{res.height}")
    print(f"  fx={calib.left_cam.fx:.1f}  baseline={calib.get_camera_baseline():.1f}mm")
    n_frames = 3
    valid_iters = 8
    print(f"Press [s] to capture → FFS → mesh, [i/I] iters, [n/N] frames, [q] to quit")

    left_mat = sl.Mat()
    runtime = sl.RuntimeParameters()
    cv2.namedWindow("ZED Preview", cv2.WINDOW_NORMAL)

    while True:
        if zed.grab(runtime) != sl.ERROR_CODE.SUCCESS:
            continue
        zed.retrieve_image(left_mat, sl.VIEW.LEFT)
        preview = left_mat.get_data()[:, :, :3].copy()
        h, w = preview.shape[:2]
        preview_small = cv2.resize(preview, (w // 2, h // 2))

        # show current settings
        info_text = f"iters={valid_iters}  frames={n_frames}  [s]run [i/I]iter [n/N]frames [q]uit"
        cv2.putText(preview_small, info_text, (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 1)
        cv2.imshow("ZED Preview", preview_small)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('i'):
            valid_iters = max(2, valid_iters - 2)
            print(f"  iters → {valid_iters}")
        elif key == ord('I'):
            valid_iters = min(32, valid_iters + 2)
            print(f"  iters → {valid_iters}")
        elif key == ord('n'):
            n_frames = max(1, n_frames - 1)
            print(f"  frames → {n_frames}")
        elif key == ord('N'):
            n_frames = min(20, n_frames + 1)
            print(f"  frames → {n_frames}")
        elif key == ord('s'):
            timings = {}

            # ── Stage 1: Capture ──
            print("\n" + "=" * 60)
            print(f"Stage 1: Capturing {n_frames} frames (iters={valid_iters})")
            t0 = time.perf_counter()
            pairs = grab_frames(zed, n_frames=n_frames)
            timings['capture'] = time.perf_counter() - t0
            if not pairs:
                print("  no frames captured"); continue

            out_dir = OUTPUT_ROOT / time.strftime("%Y%m%d_%H%M%S")
            out_dir.mkdir(parents=True, exist_ok=True)
            k_path, (fx, fy, cx, cy) = save_frames_and_intrinsics(pairs, calib, out_dir)

            # ── Stage 2: FFS Inference ──
            print("\nStage 2: FFS depth estimation")
            result_dir, t_ffs = run_ffs(out_dir, k_path, scale=1.0, valid_iters=valid_iters)
            timings['ffs_inference'] = t_ffs
            if result_dir is None:
                continue

            # ── Stage 3: Fusion + Mask ──
            print("\nStage 3: Fusion + overlap mask")
            t0 = time.perf_counter()
            depth, disp = load_and_fuse(result_dir, len(pairs))
            timings['fusion'] = time.perf_counter() - t0
            if depth is None:
                continue

            # ── Stage 4: Back-project to 3D ──
            print("\nStage 4: Back-projection to 3D")
            t0 = time.perf_counter()
            color_bgr = pairs[-1][0]  # use last frame's color
            points, colors = depth_to_points(depth, color_bgr, fx, fy, cx, cy)
            timings['backproject'] = time.perf_counter() - t0
            print(f"  {len(points):,} valid 3D points")

            # ── Stage 5a: PLY point cloud ──
            print("\nStage 5a: Export PLY point cloud")
            ply_pc_path = out_dir / "pointcloud.ply"
            t0 = time.perf_counter()
            write_ply_pointcloud(str(ply_pc_path), points, colors)
            timings['ply_export'] = time.perf_counter() - t0

            # ── Stage 5b: Mesh reconstruction  <<<< MESH MODULE >>>> ──
            print("\nStage 5b: Organized mesh reconstruction  <<<< MESH MODULE >>>>")
            mesh_path = out_dir / "mesh.ply"
            t0 = time.perf_counter()
            try:
                build_organized_mesh(depth, color_bgr, fx, fy, cx, cy, mesh_path,
                                     depth_max=10.0, depth_disc_thresh=0.05, step=1)
                timings['mesh'] = time.perf_counter() - t0
            except Exception as e:
                timings['mesh'] = time.perf_counter() - t0
                print(f"  mesh failed: {e}")

            # ── Timing Report ──
            print("\n" + "=" * 60)
            print("TIMING REPORT")
            print("-" * 40)
            total = 0
            for stage, t in timings.items():
                print(f"  {stage:<20s} {t:>7.1f}s")
                total += t
            print("-" * 40)
            print(f"  {'TOTAL':<20s} {total:>7.1f}s")
            print("=" * 60)

            # ── Display ──
            vis = depth_to_colormap(depth)
            if vis.shape[:2] != color_bgr.shape[:2]:
                vis = cv2.resize(vis, (color_bgr.shape[1], color_bgr.shape[0]),
                                 interpolation=cv2.INTER_NEAREST)
            canvas = np.hstack([color_bgr, vis])
            canvas_small = cv2.resize(canvas, (canvas.shape[1] // 2, canvas.shape[0] // 2))
            cv2.imshow("FFS Depth + Mesh Output", canvas_small)

            print(f"\n  depth range: {np.nanmin(depth):.3f} ~ {np.nanmax(depth):.3f} m")
            print(f"  output dir: {out_dir}")
            print(f"  files: pointcloud.ply, mesh.ply, mesh.obj")
            print("  → Open in Rhino: File > Import > select .ply or .obj")

    zed.close()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
