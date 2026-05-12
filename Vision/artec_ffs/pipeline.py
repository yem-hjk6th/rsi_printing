"""
pipeline.py — Top-level entry point for the artec_ffs reconstruction pipeline.

Extends artec_imit/pipeline.py with an optional --ffs flag that replaces
ZED SGM depth with Fast-FoundationStereo depth before registration.

Subcommands:
    capture             Run live capture session (requires ZED camera, zedenv)
    recon <data_dir>    Offline reconstruction

Usage:
    # Capture (zedenv):
    python Vision/artec_ffs/pipeline.py capture
    python Vision/artec_ffs/pipeline.py capture --mode auto

    # Reconstruct with ZED depth (zedenv):
    python Vision/artec_ffs/pipeline.py recon "Vision/vision_demo_test_res/ffs_20260506_XXXXXX"

    # Reconstruct with FFS depth (ffs env — needs GPU + PyTorch):
    python Vision/artec_ffs/pipeline.py recon "Vision/vision_demo_test_res/ffs_20260506_XXXXXX" --ffs

    # FFS + Frame-to-model (best quality):
    python Vision/artec_ffs/pipeline.py recon "Vision/vision_demo_test_res/ffs_20260506_XXXXXX" --ffs --ftm

    # Custom output dir:
    python Vision/artec_ffs/pipeline.py recon <data_dir> --ffs --out <out_dir>

Note on envs:
    --ffs requires the 'ffs' conda env (PyTorch + CUDA).
    Without --ffs the 'zedenv' env is sufficient (Open3D only).

Output (written to <out_dir>, default <data_dir>/artec_recon/):
    mesh.ply          Marching Cubes mesh, post-processed
    pcd.ply           TSDF point cloud
    recon_log.txt     per-view registration fitness & RMSE
"""

import sys, json, time, argparse
from pathlib import Path

import numpy as np
import open3d as o3d
import cv2

# ── importable from any cwd ──────────────────────────────────────────────────
sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))  # Vision/

import config
import register as reg_mod
import fuse     as fuse_mod
import mesh     as mesh_mod


DEFAULT_RECON_DIR = r"Vision/vision_demo_test_res/tsdf_20260505_153042"


# ══════════════════════════════════════════════════════════════════════════════
#  Load helpers
# ══════════════════════════════════════════════════════════════════════════════

def load_data(data_dir):
    data_dir = Path(data_dir).resolve()
    meta     = json.loads((data_dir / "capture_meta.json").read_text())

    fx, fy = meta["intrinsics"]["fx"], meta["intrinsics"]["fy"]
    cx, cy = meta["intrinsics"]["cx"], meta["intrinsics"]["cy"]
    W, H   = meta["resolution"]
    roi_in_depth = meta.get("roi_in_depth", False)

    intrinsic    = o3d.camera.PinholeCameraIntrinsic(W, H, fx, fy, cx, cy)
    depth_files  = sorted(data_dir.glob("view_*_depth.npy"))
    color_files  = sorted(data_dir.glob("view_*_color.png"))
    ply_files    = sorted(data_dir.glob("view_*.ply"))

    if len(depth_files) < 2:
        raise RuntimeError(f"Need ≥2 depth files, found {len(depth_files)}")
    if len(depth_files) != len(color_files):
        raise RuntimeError(f"Depth/color count mismatch")
    have_ply = (len(ply_files) == len(depth_files))

    print(f"Loading {len(depth_files)} views from: {data_dir}")
    print(f"  roi_in_depth={roi_in_depth}  have_ply={have_ply}")

    pcds, depths, colors = [], [], []
    for i, (df, cf) in enumerate(zip(depth_files, color_files)):
        depth_m   = np.load(str(df)).astype(np.float32)
        color_rgb = cv2.cvtColor(cv2.imread(str(cf)), cv2.COLOR_BGR2RGB)
        pcd = o3d.io.read_point_cloud(str(ply_files[i])) if have_ply \
              else o3d.geometry.PointCloud()
        pcds.append(pcd)
        depths.append(depth_m)
        colors.append(color_rgb)
        pts = len(pcd.points) if have_ply else 0
        print(f"  {df.name}  depth={depth_m.shape}  pts={pts:,}")

    return pcds, depths, colors, intrinsic, roi_in_depth


# ══════════════════════════════════════════════════════════════════════════════
#  Reconstruct
# ══════════════════════════════════════════════════════════════════════════════

def reconstruct(data_dir, use_ftm=False, use_ffs=False, out_dir=None):
    data_dir = Path(data_dir).resolve()
    out_dir  = Path(out_dir).resolve() if out_dir else data_dir / "artec_recon"
    out_dir.mkdir(parents=True, exist_ok=True)

    # ── Optional: replace ZED depth with FFS depth ────────────────────────
    if use_ffs:
        print(f"\n{'='*60}")
        print("Step 0/3 — FFS depth refinement ...")
        t_ffs = time.perf_counter()
        import ffs_depth
        model = ffs_depth.load_model()
        refined = ffs_depth.refine_dir(data_dir, model)
        del model
        import torch; torch.cuda.empty_cache()
        print(f"  Refined {len(refined)} views in {time.perf_counter()-t_ffs:.1f}s")

    pcds, depths, colors, intrinsic, roi_in_depth = load_data(data_dir)
    n   = len(pcds)
    t0  = time.perf_counter()

    # ── Step 1: Registration ──────────────────────────────────────────────
    print(f"\n{'='*60}")
    strategy = "frame-to-model" if use_ftm else "pose-graph"
    depth_src = "FFS" if use_ffs else "ZED"
    print(f"Step 1/3 — Registration ({strategy}, {n} views, depth={depth_src}) ...")
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

    (out_dir / "recon_log.txt").write_text("\n".join(log_lines))

    total = time.perf_counter() - t0
    print(f"\nTotal: {total:.1f}s  (depth_src={depth_src})")
    print(f"Output: {out_dir}")
    print(f"  mesh.ply / pcd.ply / recon_log.txt")


# ══════════════════════════════════════════════════════════════════════════════
#  Entry point
# ══════════════════════════════════════════════════════════════════════════════

def main():
    parser = argparse.ArgumentParser(description="artec_ffs pipeline")
    sub    = parser.add_subparsers(dest="cmd")

    cap_p = sub.add_parser("capture", help="Live capture (zedenv)")
    cap_p.add_argument("--mode", choices=["manual", "auto"], default="manual")
    cap_p.add_argument("--out",  type=str, default=None)

    rec_p = sub.add_parser("recon", help="Offline reconstruction")
    rec_p.add_argument("data_dir", nargs="?", default=DEFAULT_RECON_DIR)
    rec_p.add_argument("--ftm",    action="store_true",
                       help="Frame-to-model tracking (better quality)")
    rec_p.add_argument("--no-ftm", dest="ftm", action="store_false")
    rec_p.add_argument("--ffs",    action="store_true",
                       help="Replace ZED depth with FFS depth (requires ffs env + GPU)")
    rec_p.add_argument("--no-ffs", dest="ffs", action="store_false")
    rec_p.add_argument("--out",    type=str, default=None,
                       help="Output directory (default: <data_dir>/artec_recon/)")
    rec_p.set_defaults(ftm=False, ffs=False)

    args = parser.parse_args()

    if args.cmd == "capture":
        from capture import capture_session
        capture_session(mode=args.mode, out_dir=args.out)

    elif args.cmd == "recon":
        reconstruct(args.data_dir, use_ftm=args.ftm, use_ffs=args.ffs, out_dir=args.out)

    else:
        print(f"No subcommand — running recon on default dir:\n  {DEFAULT_RECON_DIR}")
        reconstruct(DEFAULT_RECON_DIR)


if __name__ == "__main__":
    main()
