"""
pipeline.py — Top-level entry point for the ffs_poisson_detail pipeline.

Derived from artec_ffs/pipeline.py. Same capture + FFS-depth + registration
stages; the reconstruction stage adds the detail-recovery ladder:

  * finer TSDF voxels                       (config.FUSE_VOXEL / FUSE_TRUNC)
  * Screened Poisson mesh extraction        (config.MESH_BACKEND = "poisson")
  * a fast data-quality gate after capture  (quality_check.py)

Subcommands:
    capture             Live capture session (requires ZED camera)
    qc <data_dir>       Data-quality check — run right after capture, ~15 s,
                        tells you per-view + per-pair quality WITHOUT a full
                        reconstruction so you can decide to re-record now.
    recon <data_dir>    Offline reconstruction

Usage (all in the 'ffs' conda env):
    python Vision/ffs_poisson_detail/pipeline.py capture --mode auto
    python Vision/ffs_poisson_detail/pipeline.py qc     "Vision/vision_demo_test_res/ffs_XXXXXX"
    python Vision/ffs_poisson_detail/pipeline.py recon  "Vision/vision_demo_test_res/ffs_XXXXXX" --ffs

Reconstruction defaults: --ftm ON, --ffs OFF, MESH_BACKEND from config.py.
Use --no-ftm / --mesh marching_cubes to fall back to artec_ffs behaviour.

Output (written to <out_dir>, default <data_dir>/poisson_recon/):
    mesh.ply          final mesh (Poisson or marching cubes)
    pcd.ply           fused TSDF point cloud
    recon_log.txt     per-view registration fitness & RMSE
"""

import sys, json, time, argparse
from pathlib import Path

# Force UTF-8 stdout/stderr on Windows so non-ASCII chars in print() don't
# crash with UnicodeEncodeError under the default cp1252 codepage.
if sys.platform == "win32":
    try:
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
        sys.stderr.reconfigure(encoding="utf-8", errors="replace")
    except Exception:
        pass

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
        raise RuntimeError(f"Need >=2 depth files, found {len(depth_files)}")
    if len(depth_files) != len(color_files):
        raise RuntimeError("Depth/color count mismatch")
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

def reconstruct(data_dir, use_ftm=True, use_ffs=False, out_dir=None,
                mesh_backend=None):
    data_dir = Path(data_dir).resolve()
    out_dir  = Path(out_dir).resolve() if out_dir else data_dir / "poisson_recon"
    out_dir.mkdir(parents=True, exist_ok=True)

    # Allow a CLI override of config.MESH_BACKEND for this run only.
    if mesh_backend is not None:
        config.MESH_BACKEND = mesh_backend
        mesh_mod.MESH_BACKEND = mesh_backend

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
    strategy  = "frame-to-model" if use_ftm else "pose-graph"
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
          f"(voxel={config.FUSE_VOXEL*1000:.1f}mm, trunc={config.FUSE_TRUNC*1000:.1f}mm) ...")
    t1 = time.perf_counter()
    volume = fuse_mod.integrate(
        depths, colors, poses, intrinsic,
        pcds=pcds if len(pcds[0].points) > 0 else None,
        roi_in_depth=roi_in_depth,
    )
    print(f"  Done in {time.perf_counter()-t1:.1f}s")

    # ── Step 3: Mesh extraction + post-processing ─────────────────────────
    print(f"\n{'='*60}")
    print(f"Step 3/3 — Mesh extraction (backend={config.MESH_BACKEND}) ...")
    mesh_raw, pcd_out = fuse_mod.extract(volume)
    mesh_clean = mesh_mod.build(mesh_raw, pcd_out)
    mesh_mod.save(mesh_clean, out_dir / "mesh.ply")
    o3d.io.write_point_cloud(str(out_dir / "pcd.ply"), pcd_out)
    print(f"  pcd.ply  ({len(pcd_out.points):,} pts)")

    (out_dir / "recon_log.txt").write_text("\n".join(log_lines))

    total = time.perf_counter() - t0
    print(f"\nTotal: {total:.1f}s  (depth_src={depth_src}, mesh={config.MESH_BACKEND})")
    print(f"Output: {out_dir}")
    print("  mesh.ply / pcd.ply / recon_log.txt")


# ══════════════════════════════════════════════════════════════════════════════
#  Entry point
# ══════════════════════════════════════════════════════════════════════════════

def main():
    parser = argparse.ArgumentParser(description="ffs_poisson_detail pipeline")
    sub    = parser.add_subparsers(dest="cmd")

    cap_p = sub.add_parser("capture", help="Live capture (ffs env: pyzed)")
    cap_p.add_argument("--mode", choices=["manual", "auto"], default="auto",
                       help="auto = move camera slowly, auto-triggers on motion (recommended)")
    cap_p.add_argument("--out",  type=str, default=None)

    qc_p = sub.add_parser("qc", help="Fast data-quality check (run right after capture)")
    qc_p.add_argument("data_dir")
    qc_p.add_argument("--no-pairs", action="store_true",
                      help="Skip the pairwise registration probe (per-view stats only)")

    rec_p = sub.add_parser("recon", help="Offline reconstruction")
    rec_p.add_argument("data_dir", nargs="?", default=DEFAULT_RECON_DIR)
    rec_p.add_argument("--ftm",    action="store_true",
                       help="Frame-to-model tracking (default; drift-resistant)")
    rec_p.add_argument("--no-ftm", dest="ftm", action="store_false",
                       help="Disable FTM and use pose-graph instead")
    rec_p.add_argument("--ffs",    action="store_true",
                       help="Replace ZED depth with FFS depth (requires ffs env + GPU)")
    rec_p.add_argument("--no-ffs", dest="ffs", action="store_false")
    rec_p.add_argument("--mesh",   choices=["poisson", "marching_cubes"], default=None,
                       help="Mesh backend (default: config.MESH_BACKEND = "
                            f"{config.MESH_BACKEND!r})")
    rec_p.add_argument("--out",    type=str, default=None,
                       help="Output directory (default: <data_dir>/poisson_recon/)")
    rec_p.set_defaults(ftm=True, ffs=False)

    args = parser.parse_args()

    if args.cmd == "capture":
        from capture import capture_session
        capture_session(mode=args.mode, out_dir=args.out)

    elif args.cmd == "qc":
        import quality_check
        report = quality_check.check_dir(args.data_dir, probe_pairs=not args.no_pairs)
        sys.exit(0 if report["verdict"] != "FAIL" else 1)

    elif args.cmd == "recon":
        reconstruct(args.data_dir, use_ftm=args.ftm, use_ffs=args.ffs,
                    out_dir=args.out, mesh_backend=args.mesh)

    else:
        parser.print_help()


if __name__ == "__main__":
    main()
