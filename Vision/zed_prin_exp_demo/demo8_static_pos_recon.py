"""
demo8_static_pos_recon.py — Static single-viewpoint reconstruction via ZED SDK native depth.

Eye-to-hand fixed camera setup (~1.2m above print bed).
Uses ZED ULTRA depth directly (no FFS subprocess) for simplicity and speed.

Pipeline:
  1. Live preview with optional ROI rectangle drawn via mouse drag
  2. Press [s] → capture N frames, fuse depth maps (median), organized depth mesh on ROI
  3. Output: pointcloud.ply + mesh.ply + mesh.obj + raw_depth.npy
     → vision_demo_test_res/<YYYYMMDD_HHMMSS>/

Controls:
  Mouse drag   : draw ROI rectangle on preview (hold & release)
  r            : reset ROI (use full frame)
  s            : capture → reconstruct → save
  n / N        : decrease / increase frame count (default 5)
  d / D        : decrease / increase depth discontinuity threshold (default 5%)
  q            : quit

Notes:
  - ROI is applied AFTER depth computation; depth is always computed on full frame.
  - Dead zone (left ~d_max columns in left image) produces no depth — ensure ROI
    is placed within the valid region shown by demo2_preview_match_region.py.
  - Coordinate frame: ZED left camera (X right, Y down, Z forward, in meters).
"""

import sys, os, time
from pathlib import Path

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
import zed_setup  # noqa: E402
import pyzed.sl as sl
import cv2
import numpy as np

# ── Output path ──────────────────────────────────────────────────────────────
SCRIPT_DIR   = Path(__file__).resolve().parent
VISION_DIR   = SCRIPT_DIR.parent
OUTPUT_ROOT  = VISION_DIR / "vision_demo_test_res"

# ── Defaults ──────────────────────────────────────────────────────────────────
DEFAULT_N_FRAMES   = 5
DEFAULT_DISC_THRESH = 0.05   # 5% depth jump → break triangle
DEPTH_MIN_M        = 0.5     # metres — ignore closer hits (noise)
DEPTH_MAX_M        = 3.0     # metres — ignore far background
MESH_STEP          = 1       # pixel stride for organised mesh (1 = full res)


# ══════════════════════════════════════════════════════════════════════════════
#  ROI drawing helper (mouse callback)
# ══════════════════════════════════════════════════════════════════════════════

class ROISelector:
    """Draw a rectangle on the preview by dragging the mouse."""
    def __init__(self):
        self.drawing  = False
        self.start    = None
        self.end      = None
        self.roi_set  = False      # True once user released mouse

    def callback(self, event, x, y, flags, param):
        scale = param.get("scale", 1.0)
        # map display coords → full-res coords
        fx_s = 1.0 / scale
        if event == cv2.EVENT_LBUTTONDOWN:
            self.drawing = True
            self.start   = (int(x * fx_s), int(y * fx_s))
            self.end     = self.start
            self.roi_set = False
        elif event == cv2.EVENT_MOUSEMOVE and self.drawing:
            self.end = (int(x * fx_s), int(y * fx_s))
        elif event == cv2.EVENT_LBUTTONUP:
            self.drawing = False
            self.end     = (int(x * fx_s), int(y * fx_s))
            self.roi_set = True

    def get_rect(self, frame_w, frame_h):
        """Return (x0, y0, x1, y1) clamped to frame, or None if not set."""
        if not self.roi_set or self.start is None:
            return None
        x0 = max(0, min(self.start[0], self.end[0]))
        y0 = max(0, min(self.start[1], self.end[1]))
        x1 = min(frame_w - 1, max(self.start[0], self.end[0]))
        y1 = min(frame_h - 1, max(self.start[1], self.end[1]))
        if x1 - x0 < 10 or y1 - y0 < 10:
            return None
        return (x0, y0, x1, y1)

    def reset(self):
        self.drawing  = False
        self.start    = None
        self.end      = None
        self.roi_set  = False


# ══════════════════════════════════════════════════════════════════════════════
#  Capture
# ══════════════════════════════════════════════════════════════════════════════

def grab_depth_frames(zed, n_frames=5, settle_skip=5):
    """Capture n_frames depth maps + last colour frame from ZED.

    Returns:
        depth_list : list of float32 arrays (metres), NaN where invalid
        color_bgr  : last left colour frame (H, W, 3) uint8
        (fx, fy, cx, cy) : left camera intrinsics (pixels)
    """
    left_mat  = sl.Mat()
    depth_mat = sl.Mat()
    runtime   = sl.RuntimeParameters()

    # Let auto-exposure settle
    for _ in range(settle_skip):
        zed.grab(runtime)

    depth_list = []
    color_bgr  = None

    for i in range(n_frames):
        if zed.grab(runtime) != sl.ERROR_CODE.SUCCESS:
            print(f"  grab failed on frame {i}")
            continue

        zed.retrieve_image(left_mat, sl.VIEW.LEFT)
        zed.retrieve_measure(depth_mat, sl.MEASURE.DEPTH)

        raw_depth = depth_mat.get_data().astype(np.float32)    # metres or mm?
        # ZED SDK returns depth in the units set by coordinate_units.
        # We set MILLIMETER in init → convert to metres.
        raw_depth = raw_depth / 1000.0

        # Replace ±inf / very large values with NaN
        raw_depth[~np.isfinite(raw_depth)] = np.nan
        depth_list.append(raw_depth)

        color_bgr = left_mat.get_data()[:, :, :3].copy()
        print(f"  frame {i+1}/{n_frames} — valid px: "
              f"{np.isfinite(raw_depth).sum():,} / {raw_depth.size:,}")

    return depth_list, color_bgr


def fuse_depths(depth_list):
    """Median-fuse a list of depth maps. Returns float32 (H, W) in metres."""
    if len(depth_list) == 1:
        return depth_list[0].copy()
    stack = np.stack(depth_list, axis=0)  # (N, H, W)
    # median ignores NaN by treating as large; use nanmedian
    fused = np.nanmedian(stack, axis=0).astype(np.float32)
    print(f"  fused {len(depth_list)} depth maps (median)")
    return fused


# ══════════════════════════════════════════════════════════════════════════════
#  Organised depth-image mesh  (same approach as demo7 build_organized_mesh)
# ══════════════════════════════════════════════════════════════════════════════

def build_organised_mesh(depth_m, color_bgr, fx, fy, cx, cy,
                         depth_min=DEPTH_MIN_M, depth_max=DEPTH_MAX_M,
                         disc_thresh=DEFAULT_DISC_THRESH,
                         step=MESH_STEP):
    """
    Correct single-viewpoint mesh: treat depth map as 2-D grid,
    quad → 2 triangles, reject discontinuities.

    Returns open3d.geometry.TriangleMesh.
    """
    import open3d as o3d

    H, W = depth_m.shape
    color_rgb = cv2.cvtColor(color_bgr, cv2.COLOR_BGR2RGB)

    rows = np.arange(0, H, step)
    cols = np.arange(0, W, step)
    Nr, Nc = len(rows), len(cols)

    uu, vv = np.meshgrid(cols, rows)          # (Nr, Nc)
    zz = depth_m[vv, uu]
    valid = np.isfinite(zz) & (zz > depth_min) & (zz < depth_max)

    xx = (uu - cx) * zz / fx
    yy = (vv - cy) * zz / fy

    verts       = np.stack([xx, yy, zz], axis=-1).reshape(-1, 3).astype(np.float64)
    vert_colors = color_rgb[vv, uu].reshape(-1, 3).astype(np.float64) / 255.0
    valid_flat  = valid.reshape(-1)
    zz_flat     = zz.ravel()

    # quad triangle indices
    r_idx = np.arange(Nr - 1)
    c_idx = np.arange(Nc - 1)
    rr, cc = np.meshgrid(r_idx, c_idx, indexing='ij')
    rr = rr.ravel(); cc = cc.ravel()

    i00 = rr * Nc + cc
    i01 = rr * Nc + (cc + 1)
    i10 = (rr + 1) * Nc + cc
    i11 = (rr + 1) * Nc + (cc + 1)

    tri1 = np.stack([i00, i10, i01], axis=-1)
    tri2 = np.stack([i01, i10, i11], axis=-1)

    def _filter(tris):
        v0, v1, v2 = tris[:, 0], tris[:, 1], tris[:, 2]
        mask = valid_flat[v0] & valid_flat[v1] & valid_flat[v2]
        z_stack = np.stack([zz_flat[v0], zz_flat[v1], zz_flat[v2]], axis=-1)
        z_min = z_stack.min(axis=-1)
        z_max = z_stack.max(axis=-1)
        mask &= z_max < z_min * (1.0 + disc_thresh)
        return tris[mask]

    all_tris = np.concatenate([_filter(tri1), _filter(tri2)], axis=0)
    print(f"  organised mesh: {Nr}×{Nc} grid, step={step}, "
          f"{len(all_tris):,} triangles kept (disc_thresh={disc_thresh*100:.0f}%)")

    mesh = o3d.geometry.TriangleMesh()
    mesh.vertices      = o3d.utility.Vector3dVector(verts)
    mesh.vertex_colors = o3d.utility.Vector3dVector(vert_colors)
    mesh.triangles     = o3d.utility.Vector3iVector(all_tris.astype(np.int32))
    mesh.remove_unreferenced_vertices()
    mesh.compute_vertex_normals()
    print(f"  final mesh: {len(mesh.vertices):,} verts, {len(mesh.triangles):,} tris")
    return mesh


def write_ply_pointcloud(path, points, colors):
    """Write colored point cloud to binary PLY (no external deps)."""
    N = len(points)
    header = (
        "ply\nformat binary_little_endian 1.0\n"
        f"element vertex {N}\n"
        "property float x\nproperty float y\nproperty float z\n"
        "property uchar red\nproperty uchar green\nproperty uchar blue\n"
        "end_header\n"
    )
    dtype = np.dtype([('x','<f4'),('y','<f4'),('z','<f4'),
                      ('r','u1'),('g','u1'),('b','u1')])
    arr = np.empty(N, dtype=dtype)
    arr['x'] = points[:, 0]; arr['y'] = points[:, 1]; arr['z'] = points[:, 2]
    arr['r'] = colors[:, 0]; arr['g'] = colors[:, 1]; arr['b'] = colors[:, 2]
    with open(path, 'wb') as f:
        f.write(header.encode('ascii'))
        f.write(arr.tobytes())
    print(f"  point cloud PLY → {path}  ({N:,} pts)")


# ══════════════════════════════════════════════════════════════════════════════
#  Reconstruction pipeline (triggered by [s])
# ══════════════════════════════════════════════════════════════════════════════

def run_reconstruction(zed, calib, roi_rect, n_frames, disc_thresh):
    """Full pipeline: capture → fuse → crop ROI → mesh → save."""
    import open3d as o3d

    info  = zed.get_camera_information()
    res   = info.camera_configuration.resolution
    W, H  = res.width, res.height
    fx    = calib.left_cam.fx
    fy    = calib.left_cam.fy
    cx    = calib.left_cam.cx
    cy    = calib.left_cam.cy

    # ── Stage 1: Capture ────────────────────────────────────────────────────
    print(f"\n{'='*60}")
    print(f"Stage 1: Capturing {n_frames} frames")
    t0 = time.perf_counter()
    depth_list, color_bgr = grab_depth_frames(zed, n_frames)
    print(f"  done in {time.perf_counter()-t0:.1f}s")
    if not depth_list:
        print("  ERROR: no frames captured"); return

    # ── Stage 2: Fuse ────────────────────────────────────────────────────────
    print("\nStage 2: Median fusion")
    depth_fused = fuse_depths(depth_list)

    # ── Stage 3: ROI crop ────────────────────────────────────────────────────
    if roi_rect is not None:
        x0, y0, x1, y1 = roi_rect
        print(f"\nStage 3: ROI crop → [{x0}:{x1}, {y0}:{y1}]")
        depth_crop = depth_fused[y0:y1+1, x0:x1+1].copy()
        color_crop = color_bgr[y0:y1+1, x0:x1+1].copy()
        # shift principal point for the crop
        cx_crop = cx - x0
        cy_crop = cy - y0
    else:
        print("\nStage 3: No ROI — using full frame")
        depth_crop = depth_fused
        color_crop = color_bgr
        cx_crop, cy_crop = cx, cy

    # ── Stage 4: Output directory ─────────────────────────────────────────────
    out_dir = OUTPUT_ROOT / time.strftime("%Y%m%d_%H%M%S")
    out_dir.mkdir(parents=True, exist_ok=True)
    print(f"\nOutput → {out_dir}")

    # save raw depth npy
    np.save(str(out_dir / "raw_depth_fused.npy"), depth_fused)
    if roi_rect is not None:
        np.save(str(out_dir / "roi_depth.npy"), depth_crop)
    # save colour
    cv2.imwrite(str(out_dir / "color_left.png"), color_bgr)
    if roi_rect is not None:
        cv2.imwrite(str(out_dir / "color_roi.png"), color_crop)

    # ── Stage 5: Point cloud ─────────────────────────────────────────────────
    print("\nStage 4: Back-project to 3D point cloud")
    Hc, Wc = depth_crop.shape
    valid  = np.isfinite(depth_crop) & (depth_crop > DEPTH_MIN_M) & (depth_crop < DEPTH_MAX_M)
    uu, vv = np.meshgrid(np.arange(Wc), np.arange(Hc))
    zz = depth_crop[valid]
    pts = np.stack([(uu[valid] - cx_crop) * zz / fx,
                    (vv[valid] - cy_crop) * zz / fy,
                    zz], axis=-1).astype(np.float32)
    cols_rgb = cv2.cvtColor(color_crop, cv2.COLOR_BGR2RGB)[valid]
    print(f"  {len(pts):,} valid points")
    write_ply_pointcloud(str(out_dir / "pointcloud.ply"), pts, cols_rgb)

    # ── Stage 6: Organised mesh ──────────────────────────────────────────────
    print("\nStage 5: Organised depth-image mesh")
    t0 = time.perf_counter()
    mesh = build_organised_mesh(depth_crop, color_crop, fx, fy, cx_crop, cy_crop,
                                disc_thresh=disc_thresh)
    elapsed = time.perf_counter() - t0
    print(f"  mesh built in {elapsed:.1f}s")

    mesh_path = out_dir / "mesh.ply"
    o3d.io.write_triangle_mesh(str(mesh_path), mesh, write_vertex_colors=True)
    print(f"  mesh PLY → {mesh_path}")
    o3d.io.write_triangle_mesh(str(out_dir / "mesh.obj"), mesh, write_vertex_colors=True)
    print(f"  mesh OBJ → {out_dir / 'mesh.obj'}")

    # ── Summary ───────────────────────────────────────────────────────────────
    print(f"\n{'='*60}")
    print(f"Done.  Output: {out_dir}")
    print(f"  pointcloud.ply   {len(pts):,} points")
    print(f"  mesh.ply / .obj  {len(mesh.vertices):,} verts  {len(mesh.triangles):,} tris")
    return out_dir


# ══════════════════════════════════════════════════════════════════════════════
#  Main loop
# ══════════════════════════════════════════════════════════════════════════════

def main():
    zed  = sl.Camera()
    init = sl.InitParameters()
    init.camera_resolution     = sl.RESOLUTION.HD2K
    init.camera_fps            = 15
    init.depth_mode            = sl.DEPTH_MODE.ULTRA
    init.coordinate_units      = sl.UNIT.MILLIMETER   # depth_mat in mm → we ÷1000
    init.depth_minimum_distance = 500                 # mm (0.5 m)

    if zed.open(init) != sl.ERROR_CODE.SUCCESS:
        print("Failed to open ZED camera"); return

    info  = zed.get_camera_information()
    calib = info.camera_configuration.calibration_parameters
    res   = info.camera_configuration.resolution
    W, H  = res.width, res.height
    print(f"ZED 2i opened: {W}×{H}")
    print(f"  fx={calib.left_cam.fx:.1f}  fy={calib.left_cam.fy:.1f}"
          f"  baseline={calib.get_camera_baseline():.1f}mm")
    print(f"\nControls: drag=ROI  r=resetROI  s=reconstruct  "
          f"n/N=frames  d/D=disc_thresh  q=quit")

    n_frames    = DEFAULT_N_FRAMES
    disc_thresh = DEFAULT_DISC_THRESH
    roi_sel     = ROISelector()
    left_mat    = sl.Mat()
    runtime     = sl.RuntimeParameters()

    DISPLAY_SCALE = 0.5        # show at half res to fit screen
    WIN = "demo8 — static recon (drag ROI, press s)"
    cv2.namedWindow(WIN, cv2.WINDOW_NORMAL)
    cv2.setMouseCallback(WIN, roi_sel.callback, {"scale": DISPLAY_SCALE})

    while True:
        if zed.grab(runtime) != sl.ERROR_CODE.SUCCESS:
            continue
        zed.retrieve_image(left_mat, sl.VIEW.LEFT)
        frame = left_mat.get_data()[:, :, :3].copy()

        # draw current ROI on display copy
        display = cv2.resize(frame, (int(W * DISPLAY_SCALE), int(H * DISPLAY_SCALE)))
        rect = roi_sel.get_rect(W, H)
        if roi_sel.drawing and roi_sel.start and roi_sel.end:
            # live preview of in-progress drag (display coords)
            ds = (int(roi_sel.start[0] * DISPLAY_SCALE),
                  int(roi_sel.start[1] * DISPLAY_SCALE))
            de = (int(roi_sel.end[0]   * DISPLAY_SCALE),
                  int(roi_sel.end[1]   * DISPLAY_SCALE))
            cv2.rectangle(display, ds, de, (0, 200, 255), 1)
        elif rect:
            x0, y0, x1, y1 = rect
            cv2.rectangle(display,
                          (int(x0 * DISPLAY_SCALE), int(y0 * DISPLAY_SCALE)),
                          (int(x1 * DISPLAY_SCALE), int(y1 * DISPLAY_SCALE)),
                          (0, 255, 0), 2)

        status_txt = (f"frames={n_frames}  disc={disc_thresh*100:.0f}%  "
                      f"ROI={'set' if rect else 'FULL'}  "
                      f"[s]run [r]resetROI [n/N]frames [d/D]disc [q]uit")
        cv2.putText(display, status_txt, (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

        cv2.imshow(WIN, display)
        key = cv2.waitKey(1) & 0xFF

        if key == ord('q'):
            break
        elif key == ord('r'):
            roi_sel.reset()
            print("ROI reset → full frame")
        elif key == ord('n'):
            n_frames = max(1, n_frames - 1)
            print(f"frames → {n_frames}")
        elif key == ord('N'):
            n_frames = min(20, n_frames + 1)
            print(f"frames → {n_frames}")
        elif key == ord('d'):
            disc_thresh = max(0.01, round(disc_thresh - 0.01, 3))
            print(f"disc_thresh → {disc_thresh*100:.0f}%")
        elif key == ord('D'):
            disc_thresh = min(0.30, round(disc_thresh + 0.01, 3))
            print(f"disc_thresh → {disc_thresh*100:.0f}%")
        elif key == ord('s'):
            current_rect = roi_sel.get_rect(W, H)
            run_reconstruction(zed, calib, current_rect, n_frames, disc_thresh)

    zed.close()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
