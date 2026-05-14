"""
config.py — Centralized parameters for the ffs_poisson_detail pipeline.

Derived from artec_ffs/config.py. Same capture / registration / FFS settings,
plus a detail-recovery section with a 3-rung "easy → hard" quality ladder:

  Rung 1 (easy)   FUSE_VOXEL / FUSE_TRUNC — finer TSDF voxels.
  Rung 2 (medium) MESH_BACKEND="poisson"  — Screened Poisson surface
                  reconstruction (Kazhdan & Hoppe, ToG 2013) instead of
                  Marching Cubes. This is the default here.
  Rung 3 (medium) POISSON_DEPTH / POISSON_DENSITY_QUANTILE — octree depth and
                  low-density vertex trimming.

Rung 0 is just running it; harder paradigms (neural SDF, Gaussian Splatting)
are deliberately NOT in this package — they are a different technical route
and belong in their own ffs_* package.
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

# ══════════════════════════════════════════════════════════════════════════════
#  Detail-recovery ladder  (the reason this package exists)
# ══════════════════════════════════════════════════════════════════════════════

# ── Rung 1 (easy): TSDF fusion resolution ────────────────────────────────────
# artec_ffs used 3 mm voxels → ≈6-9 mm effective smoothing, larger than the
# ~1-2 mm relief of eyelids / nostrils. Finer voxels are the cheapest win.
# Memory cost scales ~1/voxel³, so 1 mm is ~27× the RAM of 3 mm — fine for a
# face/small-object scan, watch out for full-room scans.
FUSE_VOXEL      = 0.0015   # was 0.003 in artec_ffs
FUSE_TRUNC      = 0.006    # was 0.012; keep ≈4× voxel size
ROI_MARGIN_PX   = 40

# Back-compat aliases (fuse.py imports these names verbatim from artec_ffs)
TSDF_VOXEL      = FUSE_VOXEL
TSDF_TRUNC      = FUSE_TRUNC

# ── Rung 2 (medium): mesh extraction backend ─────────────────────────────────
#   "marching_cubes"  — original artec_ffs behaviour (TSDF iso-surface)
#   "poisson"         — Screened Poisson on the fused, normal-bearing point
#                       cloud. Pins the surface to input samples instead of
#                       averaging it away. DEFAULT.
MESH_BACKEND    = "poisson"

# ── Rung 3 (medium): Screened Poisson parameters ─────────────────────────────
# POISSON_DEPTH    octree depth; each +1 doubles spatial resolution and cost.
#                  9 ≈ good for a face at this capture scale; try 10 for more.
# POISSON_SCALE    reconstruction cube size relative to samples' bounding box.
# POISSON_DENSITY_QUANTILE
#                  Poisson hallucinates surface in unscanned regions ("balloon"
#                  artefacts). Vertices whose sample density is below this
#                  quantile are trimmed. 0.0 = keep all, 0.1 = drop lowest 10%.
POISSON_DEPTH               = 9
POISSON_SCALE               = 1.1
POISSON_DENSITY_QUANTILE    = 0.10

# ── Mesh post-processing ─────────────────────────────────────────────────────
# Poisson output is already smooth; keep Laplacian light so detail survives.
# (Marching-cubes path still uses these the same way artec_ffs did.)
SMOOTH_ITER         = 1        # was 5 — Poisson needs little extra smoothing
SMOOTH_LAMBDA       = 0.5
MIN_TRIANGLE_AREA   = 1e-6
MIN_CLUSTER_FRAC    = 0.05

# ── Outlier removal ───────────────────────────────────────────────────────────
OUTLIER_NB      = 30
OUTLIER_STD     = 2.0

# ══════════════════════════════════════════════════════════════════════════════
#  quality_check.py thresholds  (data-quality gate, run right after capture)
# ══════════════════════════════════════════════════════════════════════════════
# A view / pair is flagged WARN or FAIL against these. Tune to taste.
QC_MIN_POINTS          = 150_000   # per-view valid point count; below → WARN
QC_FAIL_POINTS         =  50_000   # below → FAIL (almost certainly unusable)
QC_MIN_ROI_FILL        = 0.20      # fraction of ROI bbox that is valid depth
QC_PAIR_MIN_FITNESS    = 0.30      # consecutive-pair ICP fitness; below → WARN
QC_PAIR_FAIL_FITNESS   = 0.10      # below → FAIL (pair will break registration)
QC_PAIR_MAX_ROT_DEG    = 25.0      # inter-view rotation; above → WARN (too big a jump)

# Fast pairwise probe tuning. The probe is a coarse FPFH+RANSAC+ICP — it only
# needs to separate "registers fine" from "broken pair", not produce a precise
# transform, so it runs much coarser/cheaper than register.py.
QC_PROBE_VOXEL         = 0.008     # downsample voxel for the probe (coarser = faster)
QC_PROBE_RANSAC_ITER   = 20_000    # RANSAC iterations for the probe (was 100k inline)
QC_WORKERS             = 0         # parallel worker processes; 0 = auto (min(cpu//2, 8))

# ══════════════════════════════════════════════════════════════════════════════
#  FFS depth refinement
# ══════════════════════════════════════════════════════════════════════════════
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
