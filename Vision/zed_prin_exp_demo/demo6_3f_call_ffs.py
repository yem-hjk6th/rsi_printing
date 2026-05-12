"""
demo6_3f_call_ffs.py — Capture 3 stereo frames from ZED 2i, then call FFS for depth estimation.

Pipeline:
  1. Open ZED 2i in HD2K, grab 3 rectified left/right frames
  2. Save frames + intrinsic file (K.txt) to a temp output folder
  3. Launch FFS (ffs conda env) via subprocess on the saved frames
  4. Load FFS disparity output, compute median across 3 frames, display result

Environment: ffs (conda)  — this file is launched with the ffs env
Dependencies: pyzed, torch, opencv, numpy (all in ffs env)

Keys:
  s : grab 3 frames and run FFS
  q : quit
"""
import sys, os, subprocess, time, tempfile, shutil
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

# ── ZED setup ──
sys.path.insert(0, str(VISION_DIR))
import zed_setup  # noqa: E402
import pyzed.sl as sl


def grab_frames(zed, n_frames=3, settle_skip=5):
    """Grab n_frames rectified stereo pairs. Skip first few for auto-exposure."""
    left_mat = sl.Mat()
    right_mat = sl.Mat()
    runtime = sl.RuntimeParameters()

    # skip settle_skip frames for AE/AWB
    for _ in range(settle_skip):
        zed.grab(runtime)

    pairs = []
    for i in range(n_frames):
        if zed.grab(runtime) != sl.ERROR_CODE.SUCCESS:
            print(f"  grab failed on frame {i}")
            continue
        zed.retrieve_image(left_mat, sl.VIEW.LEFT)
        zed.retrieve_image(right_mat, sl.VIEW.RIGHT)
        left = left_mat.get_data()[:, :, :3].copy()
        right = right_mat.get_data()[:, :, :3].copy()
        pairs.append((left, right))
        print(f"  captured frame {i+1}/{n_frames}")
    return pairs


def save_frames_and_intrinsics(pairs, calib, out_dir):
    """Save stereo pairs as PNG + K.txt intrinsic file."""
    left_dir = out_dir / "left"
    right_dir = out_dir / "right"
    left_dir.mkdir(parents=True, exist_ok=True)
    right_dir.mkdir(parents=True, exist_ok=True)

    for i, (l, r) in enumerate(pairs):
        cv2.imwrite(str(left_dir / f"frame_{i:03d}.png"), l)
        cv2.imwrite(str(right_dir / f"frame_{i:03d}.png"), r)

    # K.txt: line1 = flattened 3x3 intrinsic, line2 = baseline in meters
    fx = calib.left_cam.fx
    fy = calib.left_cam.fy
    cx = calib.left_cam.cx
    cy = calib.left_cam.cy
    baseline_m = calib.get_camera_baseline() / 1000.0  # mm -> m

    K_flat = f"{fx} 0 {cx} 0 {fy} {cy} 0 0 1"
    k_path = out_dir / "K.txt"
    k_path.write_text(f"{K_flat}\n{baseline_m:.6f}\n")
    print(f"  saved {len(pairs)} pairs + K.txt to {out_dir}")
    return k_path


def run_ffs(out_dir, k_path, scale=0.5, valid_iters=8):
    """Call FFS inference via conda run in ffs env."""
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
        return None
    print(f"  FFS done in {elapsed:.1f}s")
    return result_dir


def load_and_fuse(result_dir, n_frames):
    """Load depth + disp npy files, median-fuse, mask invalid overlap region."""
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
        depth = depths[0]
        disp = disps[0] if disps else None
    else:
        depth = np.median(np.stack(depths, axis=0), axis=0).astype(np.float32)
        disp = np.median(np.stack(disps, axis=0), axis=0).astype(np.float32) if disps else None
        print(f"  fused {len(depths)} frames via median")

    # mask non-overlap region: pixel at column x with disp > x has no valid match
    if disp is not None:
        H, W = disp.shape
        col_idx = np.arange(W)[np.newaxis, :]  # (1, W)
        invalid = disp > col_idx
        n_masked = invalid.sum()
        depth[invalid] = np.nan
        print(f"  masked {n_masked} invalid pixels ({100*n_masked/(H*W):.1f}% of image)")

    return depth, disp


def depth_to_colormap(depth, vmin=0.2, vmax=5.0):
    """Convert depth (meters) to a color visualization. NaN → black."""
    mask = np.isnan(depth)
    d = np.clip(np.nan_to_num(depth, nan=vmin), vmin, vmax)
    d = ((d - vmin) / (vmax - vmin) * 255).astype(np.uint8)
    vis = cv2.applyColorMap(d, cv2.COLORMAP_TURBO)
    vis[mask] = 0  # black for invalid
    return vis


def main():
    # ── open ZED ──
    zed = sl.Camera()
    init = sl.InitParameters()
    init.camera_resolution = sl.RESOLUTION.HD2K
    init.camera_fps = 15
    init.depth_mode = sl.DEPTH_MODE.NONE  # we use FFS, not ZED depth

    if zed.open(init) != sl.ERROR_CODE.SUCCESS:
        print("Failed to open ZED camera")
        return

    info = zed.get_camera_information()
    calib = info.camera_configuration.calibration_parameters
    res = info.camera_configuration.resolution
    print(f"ZED 2i opened: {res.width}x{res.height}")
    print(f"  fx={calib.left_cam.fx:.1f}  baseline={calib.get_camera_baseline():.1f}mm")
    print("Press [s] to capture 3 frames & run FFS, [q] to quit")

    left_mat = sl.Mat()
    runtime = sl.RuntimeParameters()
    cv2.namedWindow("ZED Preview", cv2.WINDOW_NORMAL)

    while True:
        if zed.grab(runtime) != sl.ERROR_CODE.SUCCESS:
            continue
        zed.retrieve_image(left_mat, sl.VIEW.LEFT)
        preview = left_mat.get_data()[:, :, :3].copy()
        # downsample for preview
        h, w = preview.shape[:2]
        preview_small = cv2.resize(preview, (w // 2, h // 2))
        cv2.imshow("ZED Preview", preview_small)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('s'):
            print("\n=== Capturing 3 frames ===")
            pairs = grab_frames(zed, n_frames=3)
            if not pairs:
                print("  no frames captured")
                continue

            # save to temp dir
            out_dir = Path(tempfile.mkdtemp(prefix="ffs_demo6_"))
            k_path = save_frames_and_intrinsics(pairs, calib, out_dir)

            # run FFS (full resolution)
            result_dir = run_ffs(out_dir, k_path, scale=1.0, valid_iters=8)
            if result_dir is None:
                continue

            # fuse depth + mask non-overlap
            depth, disp = load_and_fuse(result_dir, len(pairs))
            if depth is None:
                continue

            # display (NaN pixels → black)
            vis = depth_to_colormap(depth)
            # show side by side: last left frame + depth
            left_rgb = pairs[-1][0]
            if vis.shape[:2] != left_rgb.shape[:2]:
                vis = cv2.resize(vis, (left_rgb.shape[1], left_rgb.shape[0]),
                                 interpolation=cv2.INTER_NEAREST)
            canvas = np.hstack([left_rgb, vis])
            canvas_small = cv2.resize(canvas, (canvas.shape[1] // 2, canvas.shape[0] // 2))
            cv2.imshow("FFS Depth (3-frame median)", canvas_small)
            print(f"  depth range: {np.nanmin(depth):.3f} ~ {np.nanmax(depth):.3f} m")
            print(f"  output dir: {out_dir}")
            print("Press any key on depth window, or [s] again on preview")

    zed.close()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
