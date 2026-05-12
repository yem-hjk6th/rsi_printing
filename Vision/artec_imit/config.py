"""
config.py — Centralized parameters for the artec_imit pipeline.

Tuning guide (in comments beside each parameter).
"""

from pathlib import Path

# ── Paths ─────────────────────────────────────────────────────────────────────
VISION_DIR  = Path(__file__).resolve().parent.parent
OUTPUT_ROOT = VISION_DIR / "vision_demo_test_res"

# ── Camera capture ────────────────────────────────────────────────────────────
N_FRAMES_FUSE   = 15      # frames to mean-fuse per keyframe (↑=smoother depth, ↓=faster)
SETTLE_SKIP     = 6       # frames to skip for auto-exposure settle
DEPTH_MIN_M     = 0.40    # ignore depth closer than this — ZED2i reliable floor ~0.3m
DEPTH_MAX_M     = 1.20    # ← was 2.0m; at 800mm working distance 1.2m cuts background noise
                           #   (↓ to 1.0m if object is always close; ↑ to 1.5m for larger scenes)
MAX_VIEWS       = 30      # cap on keyframes per session
DISPLAY_SCALE   = 0.5

# Auto-trigger thresholds (capture mode = "auto")
AUTO_TRANS_M    = 0.04    # 40mm translation triggers new keyframe
AUTO_ROT_DEG    = 8.0     # 8° rotation triggers new keyframe
                           # ↓ both → denser coverage; ↑ → sparser (faster registration)

# ── Registration ──────────────────────────────────────────────────────────────
VOXEL_COARSE    = 0.010   # 10mm — FPFH downsampling (↑=faster RANSAC, ↓=more features)
VOXEL_GEOM      = 0.005   # 5mm  — geometric ICP voxel
VOXEL_COLOR     = 0.003   # 3mm  — colored ICP voxel (↓=sharper but slower)
RANSAC_ITER     = 4_000_000
RANSAC_CONF     = 500
ICP_MAX_ITER    = 100
ICP_GEOM_DIST   = 0.020   # 20mm — geometric ICP correspondence threshold
ICP_COLOR_DIST  = 0.010   # 10mm — colored ICP threshold (↓=stricter, fewer false matches)

# Frame-to-model tracking
FTM_WARMUP_FRAMES   = 3       # first N keyframes used to bootstrap initial TSDF model
FTM_ICP_DIST        = 0.020   # 20mm — ICP distance for frame-to-model
FTM_MIN_FITNESS     = 0.25    # below this → frame rejected (bad pose), retry from RANSAC
                               # ↑ → stricter acceptance; ↓ → more frames accepted (may add drift)

# ── Pose Graph ────────────────────────────────────────────────────────────────
POSEGRAPH_MAX_DIST  = 0.005   # 5mm — information matrix & edge pruning threshold
POSEGRAPH_EDGE_PRUNE = 0.25
LOOP_OVERLAP_MIN    = 0.30    # min ICP fitness to add a loop-closure edge
                               # ↓ → more loop edges (tighter global opt); ↑ → fewer (safer)

# ── TSDF ─────────────────────────────────────────────────────────────────────
TSDF_VOXEL      = 0.001   # 1mm — voxel size = finest detail recoverable
                           # ↑ to 2mm for faster run / large objects; ↓ to 0.5mm needs lots of RAM
TSDF_TRUNC      = 0.004   # 4mm — truncation = ~3-5× voxel; too small→holes, too large→blurry
                           # rule of thumb: TSDF_TRUNC = 4 × TSDF_VOXEL
ROI_MARGIN_PX   = 40      # pixels of padding around PLY bbox when masking depth
                           # ↓ → tighter crop; ↑ → safer for objects near roi edge

# ── Mesh post-processing ──────────────────────────────────────────────────────
SMOOTH_ITER         = 5       # Laplacian smoothing iterations (↑=smoother, ↓=preserve detail)
SMOOTH_LAMBDA       = 0.5     # Laplacian weight (0=no smooth, 1=full)
MIN_TRIANGLE_AREA   = 1e-6    # remove degenerate triangles below this area
MIN_CLUSTER_FRAC    = 0.05    # remove connected components < this fraction of largest cluster
                               # 0.05 = remove anything < 5% of main mesh size

# ── Outlier removal ───────────────────────────────────────────────────────────
OUTLIER_NB      = 30
OUTLIER_STD     = 2.0
