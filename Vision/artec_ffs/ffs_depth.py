"""
ffs_depth.py — Replace ZED SGM depth with Fast-FoundationStereo depth.

For each view_NNN in a capture directory that has both view_NNN_color.png
and view_NNN_right.png, runs FFS inference and overwrites view_NNN_depth.npy
with the FFS-derived depth (meters, float32, NaN outside ROI preserved).
Also regenerates view_NNN.ply from the new depth.

The original ZED depth is backed up as view_NNN_depth_zed.npy before overwrite.

Public API (called by pipeline.py --ffs):
    model = load_model()
    refine_dir(data_dir, model, intrinsic_dict)

Standalone usage:
    conda activate ffs
    python ffs_depth.py "Vision/vision_demo_test_res/ffs_20260506_XXXXXX"
"""

import sys, json, argparse, shutil
from pathlib import Path

import cv2
import numpy as np
import torch
import yaml

# ── timm compatibility shim (handles timm < 0.9 which lacks timm.layers) ─────
import importlib as _importlib
try:
    import timm.layers  # noqa — ensure it's importable before torch.load unpickles
except ImportError:
    try:
        import timm.models.layers as _tl
        sys.modules.setdefault("timm.layers", _tl)
        for _sub in ["activations", "attention", "blur_pool", "cond_conv2d",
                     "conv_act", "conv_bn_act", "drop", "eca", "evo_norm",
                     "gather_excite", "global_context", "helpers", "inplace_abn",
                     "lambda_layer", "linear", "mixed_conv2d", "mlp", "norm",
                     "norm_act", "padding", "patch_embed", "pool", "pos_embed",
                     "selective_kernel", "separable_conv", "space_to_depth",
                     "split_attn", "split_batchnorm", "squeeze_excite",
                     "std_conv", "strip_pool", "test_time_pool", "trace_utils",
                     "typing", "weight_init"]:
            try:
                _m = _importlib.import_module(f"timm.models.layers.{_sub}")
                sys.modules.setdefault(f"timm.layers.{_sub}", _m)
            except ImportError:
                pass
    except ImportError:
        raise ImportError(
            "timm not installed. Install with: pip install timm>=0.9.0\n"
            "Or activate the 'ffs' conda env before using --ffs."
        )

# ── FFS source (bundled in ffs_core/) ────────────────────────────────────────
_HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(_HERE / "ffs_core"))

from core.utils.utils import InputPadder
from Utils import AMP_DTYPE, set_seed, vis_disparity

from config import (
    FFS_WEIGHTS_DIR, FFS_VALID_ITERS, FFS_MAX_DISP, FFS_SCALE,
    DEPTH_MIN_M, DEPTH_MAX_M,
)


# ══════════════════════════════════════════════════════════════════════════════
#  Model loading
# ══════════════════════════════════════════════════════════════════════════════

def load_model(weights_path=None):
    """
    Load FFS model onto GPU.
    weights_path defaults to FFS_WEIGHTS_DIR from config.py.
    Must be called from the 'ffs' conda env (PyTorch + CUDA required).
    """
    weights_path = Path(weights_path or FFS_WEIGHTS_DIR)
    if not weights_path.exists():
        raise FileNotFoundError(
            f"FFS weights not found: {weights_path}\n"
            f"Check FFS_WEIGHTS_DIR in config.py"
        )
    print(f"[ffs] Loading model: {weights_path.parent.name}")
    model = torch.load(str(weights_path), map_location="cpu", weights_only=False)
    model.args.valid_iters = FFS_VALID_ITERS
    model.args.max_disp    = FFS_MAX_DISP
    model.cuda().eval()
    torch.autograd.set_grad_enabled(False)
    print(f"[ffs] Model ready  (valid_iters={FFS_VALID_ITERS}, max_disp={FFS_MAX_DISP})")
    return model


# ══════════════════════════════════════════════════════════════════════════════
#  Inference helpers
# ══════════════════════════════════════════════════════════════════════════════

def _disp_to_depth(disp, fx, baseline_m):
    """
    Convert FFS disparity (pixels) → depth (meters).
    depth = fx * baseline / disp
    Pixels with disp <= 0 → NaN.
    """
    disp = np.where(disp > 0, disp, np.nan)
    return (fx * baseline_m / disp).astype(np.float32)


def _run_ffs(model, img_left_rgb, img_right_rgb):
    """
    Run FFS on an RGB pair (HWC uint8).
    Returns disparity map (H, W) float32.
    """
    H0, W0 = img_left_rgb.shape[:2]
    if FFS_SCALE != 1.0:
        img_left_rgb  = cv2.resize(img_left_rgb,  (0, 0), fx=FFS_SCALE, fy=FFS_SCALE)
        img_right_rgb = cv2.resize(img_right_rgb, (img_left_rgb.shape[1],
                                                   img_left_rgb.shape[0]))
    t0 = torch.as_tensor(img_left_rgb ).cuda().float()[None].permute(0, 3, 1, 2)
    t1 = torch.as_tensor(img_right_rgb).cuda().float()[None].permute(0, 3, 1, 2)
    padder = InputPadder(t0.shape, divis_by=32, force_square=False)
    t0, t1 = padder.pad(t0, t1)
    with torch.amp.autocast("cuda", enabled=True, dtype=AMP_DTYPE):
        disp = model.forward(t0, t1, iters=FFS_VALID_ITERS, test_mode=True,
                             optimize_build_volume="pytorch1")
    disp = padder.unpad(disp.float())
    disp = disp.data.cpu().numpy().reshape(img_left_rgb.shape[0],
                                           img_left_rgb.shape[1]).clip(0, None)
    if FFS_SCALE != 1.0:
        disp = cv2.resize(disp, (W0, H0), interpolation=cv2.INTER_LINEAR) / FFS_SCALE
    return disp.astype(np.float32)


# ══════════════════════════════════════════════════════════════════════════════
#  Per-directory refinement
# ══════════════════════════════════════════════════════════════════════════════

def refine_dir(data_dir, model):
    """
    Replace ZED depth with FFS depth for all views that have right images.

    For each view_NNN with view_NNN_color.png + view_NNN_right.png:
      1. Run FFS → disparity
      2. Convert disparity → depth_m using intrinsics from capture_meta.json
      3. Re-apply ROI NaN mask from original depth.npy
      4. Backup original → view_NNN_depth_zed.npy
      5. Overwrite view_NNN_depth.npy with FFS depth
      6. Regenerate view_NNN.ply from new depth

    Returns list of view indices refined.
    """
    import open3d as o3d

    data_dir = Path(data_dir).resolve()
    meta     = json.loads((data_dir / "capture_meta.json").read_text())

    fx = meta["intrinsics"]["fx"]
    fy = meta["intrinsics"]["fy"]
    cx = meta["intrinsics"]["cx"]
    cy = meta["intrinsics"]["cy"]
    baseline_m = meta["baseline_mm"] / 1000.0

    color_files = sorted(data_dir.glob("view_*_color.png"))
    refined = []

    for cf in color_files:
        stem  = cf.stem.replace("_color", "")   # view_NNN
        idx   = int(stem.split("_")[1])
        rf    = data_dir / f"{stem}_right.png"
        df    = data_dir / f"{stem}_depth.npy"
        zed_backup = data_dir / f"{stem}_depth_zed.npy"
        ply_f = data_dir / f"{stem}.ply"

        if not rf.exists():
            print(f"  [ffs] {stem}: no right image, skipping")
            continue
        if not df.exists():
            print(f"  [ffs] {stem}: no depth.npy, skipping")
            continue

        print(f"  [ffs] {stem} ...", end=" ", flush=True)

        # Load images as RGB
        img_l = cv2.cvtColor(cv2.imread(str(cf)), cv2.COLOR_BGR2RGB)
        img_r = cv2.cvtColor(cv2.imread(str(rf)), cv2.COLOR_BGR2RGB)

        # Original depth for ROI mask
        orig_depth = np.load(str(df))
        roi_mask   = np.isfinite(orig_depth)   # NaN = outside ROI

        # FFS inference
        disp  = _run_ffs(model, img_l, img_r)
        depth = _disp_to_depth(disp, fx, baseline_m)

        # Apply depth range clip
        depth = np.where((depth >= DEPTH_MIN_M) & (depth <= DEPTH_MAX_M), depth, np.nan)

        # Re-apply ROI mask
        depth[~roi_mask] = np.nan

        # Backup ZED depth
        if not zed_backup.exists():
            shutil.copy2(str(df), str(zed_backup))

        # Save FFS depth
        np.save(str(df), depth)

        # Regenerate PLY from FFS depth
        H, W = depth.shape
        valid = np.isfinite(depth)
        uu, vv = np.meshgrid(np.arange(W), np.arange(H))
        zz  = depth[valid]
        pts = np.stack([(uu[valid] - cx) * zz / fx,
                        (vv[valid] - cy) * zz / fy,
                        zz], axis=-1).astype(np.float64)
        color_bgr = cv2.imread(str(cf))
        cols = cv2.cvtColor(color_bgr, cv2.COLOR_BGR2RGB)[valid].astype(np.float64) / 255.0
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(pts)
        pcd.colors = o3d.utility.Vector3dVector(cols)
        o3d.io.write_point_cloud(str(ply_f), pcd)

        n_valid = int(valid.sum())
        print(f"{n_valid:,} pts")
        refined.append(idx)

    print(f"[ffs] Refined {len(refined)}/{len(color_files)} views")
    return refined


# ══════════════════════════════════════════════════════════════════════════════
#  Standalone entry
# ══════════════════════════════════════════════════════════════════════════════

def main():
    set_seed(0)
    parser = argparse.ArgumentParser(description="FFS depth refinement for artec_ffs")
    parser.add_argument("data_dir", help="Capture directory (must contain right images)")
    parser.add_argument("--weights", type=str, default=None,
                        help="Path to FFS model .pth (default: FFS_WEIGHTS_DIR in config.py)")
    args = parser.parse_args()

    model = load_model(args.weights)
    refine_dir(args.data_dir, model)


if __name__ == "__main__":
    main()
