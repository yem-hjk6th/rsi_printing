"""
pipeline.py — Top-level entry point for the artec_imit reconstruction pipeline.

Subcommands:
    capture             Run live capture session (requires ZED camera)
    recon <data_dir>    Run offline reconstruction on existing data

Usage:
    # Live capture (manual ROI):
    python pipeline.py capture

    # Live capture (auto ROI + auto keyframe):
    python pipeline.py capture --mode auto

    # Offline reconstruction (Pose Graph only, fast):
    python pipeline.py recon "Vision/vision_demo_test_res/tsdf_20260505_153042"

    # Offline reconstruction (Frame-to-model, better quality):
    python pipeline.py recon "Vision/vision_demo_test_res/tsdf_20260505_153042" --ftm

Compatibility:
    Works with data captured by demo11_tsdf_capture.py (legacy, full-frame depth)
    and by capture.py (ROI-masked depth, roi_in_depth=True in capture_meta.json).

Output (written to <data_dir>/artec_recon/):
    mesh.ply          Marching Cubes mesh, post-processed
    pcd.ply           TSDF point cloud
    recon_log.txt     per-view registration fitness & RMSE
"""

import sys, json, time, argparse
from pathlib import Path

import numpy as np
import open3d as o3d
import cv2

# ── make artec_imit importable when run from any cwd ─────────────────────────
sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))  # Vision/

import config
import register as reg_mod
import fuse  as fuse_mod
import mesh  as mesh_mod


# ── Default data dir (edit to run without arguments) ─────────────────────────
DEFAULT_RECON_DIR = r"Vision/vision_demo_test_res/tsdf_20260505_153042"


# ══════════════════════════════════════════════════════════════════════════════
#  Load helpers
# ══════════════════════════════════════════════════════════════════════════════

def load_data(data_dir):
    """
    Load all keyframes from a capture directory.

    Supports both:
      - demo11 data  (capture_meta.json without "roi_in_depth")
      - capture.py data (capture_meta.json with "roi_in_depth": true)

    Returns:
        pcds, depths, colors, intrinsic, roi_in_depth
    """
    data_dir = Path(data_dir).resolve()
    meta     = json.loads((data_dir / "capture_meta.json").read_text())

    fx, fy = meta["intrinsics"]["fx"], meta["intrinsics"]["fy"]
    cx, cy = meta["intrinsics"]["cx"], meta["intrinsics"]["cy"]
    W, H   = meta["resolution"]
    roi_in_depth = meta.get("roi_in_depth", False)

    intrinsic = o3d.camera.PinholeCameraIntrinsic(W, H, fx, fy, cx, cy)

    depth_files = sorted(data_dir.glob("view_*_depth.npy"))
    color_files = sorted(data_dir.glob("view_*_color.png"))
    ply_files   = sorted(data_dir.glob("view_*.ply"))

    if len(depth_files) < 2:
        raise RuntimeError(f"Need ≥2 depth files, found {len(depth_files)} in {data_dir}")
    if len(depth_files) != len(color_files):
        raise RuntimeError(f"Depth/color count mismatch: {len(depth_files)} vs {len(color_files)}")
    # ply_files may be absent in future formats — only required for legacy mask
    have_ply = (len(ply_files) == len(depth_files))

    print(f"Loading {len(depth_files)} views from: {data_dir}")
    print(f"  roi_in_depth={roi_in_depth}  have_ply={have_ply}")

    pcds, depths, colors = [], [], []
    for i, (df, cf) in enumerate(zip(depth_files, color_files)):
        depth_m   = np.load(str(df)).astype(np.float32)
        color_rgb = cv2.cvtColor(cv2.imread(str(cf)), cv2.COLOR_BGR2RGB)
        if have_ply:
            pcd = o3d.io.read_point_cloud(str(ply_files[i]))
        else:
            # Build PCD from depth (no legacy bbox fallback needed if roi_in_depth)
            pcd = o3d.geometry.PointCloud()  # empty placeholder
        pcds.append(pcd)
        depths.append(depth_m)
        colors.append(color_rgb)
        pts = len(pcd.points) if have_ply else 0
        print(f"  {df.name}  depth={depth_m.shape}  pts={pts:,}")

    return pcds, depths, colors, intrinsic, roi_in_depth


# ══════════════════════════════════════════════════════════════════════════════
#  Reconstruct
# ══════════════════════════════════════════════════════════════════════════════

def reconstruct(data_dir, use_ftm=False, out_dir=None):
    """
    Full reconstruction pipeline on an existing capture directory.
    Writes output to out_dir (default: <data_dir>/artec_recon/).
    """
    data_dir = Path(data_dir).resolve()
    out_dir  = Path(out_dir).resolve() if out_dir else data_dir / "artec_recon"
    out_dir.mkdir(parents=True, exist_ok=True)

    pcds, depths, colors, intrinsic, roi_in_depth = load_data(data_dir)
    n = len(pcds)

    t0 = time.perf_counter()

    # ── Step 1: Registration ──────────────────────────────────────────────
    print(f"\n{'='*60}")
    strategy = "frame-to-model" if use_ftm else "pose-graph"
    print(f"Step 1/3 — Registration ({strategy}, {n} views) ...")
    poses, log_lines = reg_mod.register(
        pcds, depths, colors, intrinsic,
        roi_in_depth=roi_in_depth,
        use_ftm=use_ftm,
    )
    print(f"  Done in {time.perf_counter()-t0:.1f}s")

    # ── Step 2: TSDF fusion ───────────────────────────────────────────────
    print(f"\n{'='*60}")
    print(f"Step 2/3 — TSDF fusion  "
          f"(voxel={config.TSDF_VOXEL*1000:.0f}mm, trunc={config.TSDF_TRUNC*1000:.0f}mm) ...")
    t1 = time.perf_counter()

    # If frame-to-model was used, fuse.integrate was already called incrementally
    # inside register.py; here we do a clean final pass with optimized poses.
    volume = fuse_mod.integrate(
        depths, colors, poses, intrinsic,
        pcds=pcds if len(pcds[0].points) > 0 else None,
        roi_in_depth=roi_in_depth,
    )
    print(f"  Done in {time.perf_counter()-t1:.1f}s")

    # ── Step 3: Mesh extraction + post-processing ─────────────────────────
    print(f"\n{'='*60}")
    print("Step 3/3 — Mesh extraction + post-processing ...")
    mesh_raw, pcd_out = fuse_mod.extract(volume)

    mesh_clean = mesh_mod.process(mesh_raw)
    mesh_mod.save(mesh_clean, out_dir / "mesh.ply")

    o3d.io.write_point_cloud(str(out_dir / "pcd.ply"), pcd_out)
    print(f"  pcd.ply  ({len(pcd_out.points):,} pts)")

    # ── Log ───────────────────────────────────────────────────────────────
    (out_dir / "recon_log.txt").write_text("\n".join(log_lines))

    total = time.perf_counter() - t0
    print(f"\nTotal: {total:.1f}s")
    print(f"Output: {out_dir}")
    print(f"  mesh.ply  — open in MeshLab / CloudCompare")
    print(f"  pcd.ply   — point cloud")
    print(f"  recon_log.txt")


# ══════════════════════════════════════════════════════════════════════════════
#  Entry point
# ══════════════════════════════════════════════════════════════════════════════

def main():
    parser = argparse.ArgumentParser(description="artec_imit pipeline")
    sub    = parser.add_subparsers(dest="cmd")

    # capture subcommand
    cap_p = sub.add_parser("capture", help="Live capture (requires ZED camera)")
    cap_p.add_argument("--mode", choices=["manual", "auto"], default="manual",
                       help="manual=SPACE trigger+ROI dialog  auto=motion threshold")
    cap_p.add_argument("--out",  type=str, default=None,
                       help="Output directory (default: auto-timestamped)")

    # recon subcommand
    rec_p = sub.add_parser("recon", help="Offline reconstruction from capture directory")
    rec_p.add_argument("data_dir", nargs="?", default=DEFAULT_RECON_DIR,
                       help="Path to capture directory")
    rec_p.add_argument("--ftm", action="store_true",
                       help="Use frame-to-model tracking (better quality, slower)")
    rec_p.add_argument("--no-ftm", dest="ftm", action="store_false")
    rec_p.add_argument("--out", type=str, default=None,
                       help="Output directory (default: <data_dir>/artec_recon/)")
    rec_p.set_defaults(ftm=False)

    args = parser.parse_args()

    if args.cmd == "capture":
        from capture import capture_session
        capture_session(mode=args.mode, out_dir=args.out)

    elif args.cmd == "recon":
        reconstruct(args.data_dir, use_ftm=args.ftm, out_dir=args.out)

    else:
        # No subcommand: default to recon on DEFAULT_RECON_DIR
        print(f"No subcommand given — running recon on default dir:")
        print(f"  {DEFAULT_RECON_DIR}")
        reconstruct(DEFAULT_RECON_DIR, use_ftm=False)


if __name__ == "__main__":
    main()
