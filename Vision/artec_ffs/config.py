"""
config.py — Centralized parameters for the artec_ffs pipeline.
Same as artec_imit/config.py plus FFS depth-refinement section.
"""

from pathlib import Path

# ── Paths ─────────────────────────────────────────────────────────────────────
VISION_DIR  = Path(__file__).resolve().parent.parent
OUTPUT_ROOT = VISION_DIR / "vision_demo_test_res"

# ── Camera capture ────────────────────────────────────────────────────────────
N_FRAMES_FUSE   = 15
SETTLE_SKIP     = 6
DEPTH_MIN_M     = 0.55   # MIN = fx*baseline/MAX_DISP = 1907.2*0.1201/416 ≈ 0.55m (23-36-37 weights)
DEPTH_MAX_M     = 1.20
MAX_VIEWS       = 30
DISPLAY_SCALE   = 0.5

# Auto-trigger thresholds
AUTO_TRANS_M    = 0.04
AUTO_ROT_DEG    = 8.0

# ── Registration ──────────────────────────────────────────────────────────────
VOXEL_COARSE    = 0.010
VOXEL_GEOM      = 0.005
VOXEL_COLOR     = 0.003
RANSAC_ITER     = 4_000_000
RANSAC_CONF     = 500
ICP_MAX_ITER    = 100
ICP_GEOM_DIST   = 0.020
ICP_COLOR_DIST  = 0.010

# Frame-to-model tracking
FTM_WARMUP_FRAMES   = 3
FTM_ICP_DIST        = 0.020
FTM_MIN_FITNESS     = 0.50

# ── Pose Graph ────────────────────────────────────────────────────────────────
POSEGRAPH_MAX_DIST   = 0.005
POSEGRAPH_EDGE_PRUNE = 0.25
LOOP_OVERLAP_MIN     = 0.30

# ── TSDF ─────────────────────────────────────────────────────────────────────
TSDF_VOXEL      = 0.003
TSDF_TRUNC      = 0.012
ROI_MARGIN_PX   = 40

# ── Mesh post-processing ─────────────────────────────────────────────────────
SMOOTH_ITER         = 5
SMOOTH_LAMBDA       = 0.5
MIN_TRIANGLE_AREA   = 1e-6
MIN_CLUSTER_FRAC    = 0.05

# ── Outlier removal ───────────────────────────────────────────────────────────
OUTLIER_NB      = 30
OUTLIER_STD     = 2.0

# ── FFS depth refinement ─────────────────────────────────────────────────────
# Weights are NOT copied (too large). Point to original Fast-FoundationStereo repo.
# Variants (best→fastest):  23-36-37 / 20-26-39 / 20-30-48
#
# FFS_REPO_ROOT resolution order (per-machine, never commit a hardcoded path here):
#   1. FFS_REPO_ROOT environment variable
#   2. sibling config_local.py defining FFS_REPO_ROOT  (gitignored — see README)
#   3. placeholder that triggers FileNotFoundError in ffs_depth.load_model()
import os as _os
_env_root = _os.environ.get("FFS_REPO_ROOT")
if _env_root:
    FFS_REPO_ROOT = Path(_env_root)
else:
    try:
        from config_local import FFS_REPO_ROOT  # type: ignore[no-redef]
        FFS_REPO_ROOT = Path(FFS_REPO_ROOT)
    except ImportError:
        FFS_REPO_ROOT = Path("<UNSET — set FFS_REPO_ROOT env var or create config_local.py>")
FFS_WEIGHTS_DIR = FFS_REPO_ROOT / "weights" / "23-36-37" / "model_best_bp2_serialize.pth"
FFS_VALID_ITERS = 8       # ↓ to 4 for faster inference; ↑ to 16 for best quality
FFS_MAX_DISP    = 416     # must match model's baked-in max_disp (23-36-37 weights); ZED2i covers depth≥0.55m
FFS_SCALE       = 1.0     # resize input before inference (0.5 = 2× faster, slight quality loss)
