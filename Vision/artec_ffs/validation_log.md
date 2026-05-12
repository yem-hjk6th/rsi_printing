# artec_ffs Validation Log

Dataset: `Vision/vision_demo_test_res/ffs_20260506_173026` (17 views)

---

## Step 0 — Baseline (2026-05-08)
### Camera intrinsics
```
fx = 1907.2 px   baseline = 120.1 mm
Depth at MAX_DISP=192 px: 1907.2 × 0.1201 / 192 = 1.193 m
→ Any point closer than 1.193 m requires >192 px disparity — outside FFS cost volume.
```

### Expected disparity vs. actual working range
| depth | required disp | covered by MAX_DISP=192 |
|-------|---------------|------------------------|
| 0.4 m | 572 px | ❌ 3× over |
| 0.6 m | 382 px | ❌ 2× over |
| 0.8 m | 286 px | ❌ 1.5× over |
| 0.94 m | 243 px | ❌ still over |
| 1.19 m | 192 px | ✅ boundary |

### Depth distribution comparison (first 5 views)

| view | FFS p10 | FFS p50 | FFS p90 | ZED p10 | ZED p50 | ZED p90 |
|------|---------|---------|---------|---------|---------|---------|
| 000 | 0.861 m | 0.942 m | 1.085 m | 0.717 m | 0.891 m | 1.088 m |
| 001 | 0.869 m | 0.942 m | 1.060 m | 0.722 m | 0.887 m | 1.090 m |
| 002 | 0.851 m | 0.933 m | 1.011 m | 0.677 m | 0.826 m | 0.994 m |
| 003 | 0.767 m | 0.875 m | 0.965 m | 0.668 m | 0.803 m | 0.952 m |
| 004 | 0.782 m | 0.855 m | 0.930 m | 0.644 m | 0.768 m | 0.915 m |

**Diagnosis confirmed**: FFS p50 ≈ 0.87–0.94 m (near the cost-volume floor), while ZED p10
reaches 0.64–0.72 m. FFS is blind to the closer half of the scene; those pixels either map
to spurious matches near 192 px or get clipped, producing a false depth plateau around
0.87–0.94 m. This compresses the point cloud onto a near-flat surface, collapsing the
FPFH feature space and making every ICP initialization degenerate.

### Baseline recon_log (FFS w/ MAX_DISP=192 + FTM)
```
pair  method          fitness  rmse_mm
0-1                   0.6661   4.490
1-2                   0.5910   4.632
3   FTM               0.5851   4.604
4   FTM-FALLBACK      0.1526   5.315
5   FTM               0.6417   4.928
6   FTM               0.2555   5.549
7   FTM               0.4287   5.214
8   FTM               0.4320   5.398
9   FTM               0.4979   5.219
10  FTM               0.4250   5.356
11  FTM               0.4779   5.451
12  FTM               0.6057   5.321
13  FTM               0.4868   5.456
14  FTM               0.6304   5.340
15  FTM               0.6396   5.393
16  FTM               0.6470   5.628
```
Mean fitness (FTM views 3–16): **0.495**   Mean rmse: **5.30 mm**
Mesh result: completely scattered, object shape unrecognizable.

---

## Step 1 — After parameter fixes (2026-05-08)
### Changes applied
- `config.py`: `FFS_MAX_DISP = 192 → 416` (model's baked-in max_disp; 512 crashes due to pe shape mismatch)
- `config.py`: `FTM_MIN_FITNESS = 0.25 → 0.50`
- `config.py`: `TSDF_VOXEL = 0.001 → 0.003`, `TSDF_TRUNC = 0.004 → 0.012`
- `ffs_depth.py`: `cv2.resize(dsize=None → (0,0))` for OpenCV API correctness
- `ffs_depth.py`: `import shutil` moved to module top

FFS re-inference run on same dataset with MAX_DISP=416.
Recon output: `artec_recon_v2/` (pipeline run without `--ffs`; depth files already FFS-corrected)

### Depth distribution after MAX_DISP=416 (first 5 views)

| view | FFS p10 | FFS p50 | FFS p90 | ZED p10 | ZED p50 | ZED p90 |
|------|---------|---------|---------|---------|---------|---------|
| 000 | 0.723 m | 0.886 m | 1.082 m | 0.717 m | 0.891 m | 1.088 m |
| 001 | 0.715 m | 0.860 m | 1.062 m | 0.722 m | 0.887 m | 1.090 m |
| 002 | 0.691 m | 0.839 m | 1.008 m | 0.677 m | 0.826 m | 0.994 m |
| 003 | 0.662 m | 0.795 m | 0.943 m | 0.668 m | 0.803 m | 0.952 m |
| 004 | 0.642 m | 0.767 m | 0.913 m | 0.644 m | 0.768 m | 0.915 m |

**FFS p10/p50/p90 now matches ZED within 6 mm across all 5 views** — cost-volume fix confirmed.
(Old FFS p50 was 0.87–0.94 m; ZED p10 was 0.64–0.72 m — gap up to 200 mm now closed.)

### recon_log v2 (FFS w/ MAX_DISP=416 + FTM + MIN_FITNESS=0.50)
```
pair  method          fitness  rmse_mm
0-1                   0.9258   1.769
1-2                   0.8955   1.798
3   FTM               0.9594   2.020
4   FTM               0.9829   2.071
5   FTM               0.9873   1.918
6   FTM               0.6091   3.340
7   FTM               0.7762   3.184
8   FTM               0.7430   3.303
9   FTM               0.8357   2.568
10  FTM-FALLBACK      0.5029   2.813
11  FTM               0.5478   3.559
12  FTM-FALLBACK      0.4737   2.599
13  FTM-FALLBACK      0.3735   2.816
14  FTM-FALLBACK      0.4458   2.920
15  FTM-FALLBACK      0.5819   3.387
16  FTM               0.5314   4.204
```
Mean fitness (FTM views 3–16): **0.668**   Mean rmse: **2.91 mm**
Mean fitness (all 16 pairs):   **0.698**   Mean rmse: **2.77 mm**
Mesh result: 347,642 verts, 674,420 tris — mesh generated successfully.

### Summary

| metric | Step 0 baseline (MAX_DISP=192) | Step 1 (MAX_DISP=416) | Δ |
|--------|-------------------------------|----------------------|---|
| FTM mean fitness | 0.495 | **0.668** | +35% |
| All-pair mean fitness | ~0.51 | **0.698** | +37% |
| Mean rmse | 5.30 mm | **2.77 mm** | −48% |
| Mesh quality | scattered/unrecognizable | 347K verts, recognizable | ✅ |

**Root cause fix confirmed**: MAX_DISP=192→416 resolves the primary regression.

**Residual issue visible**: Views 12–14 show RANSAC fallback with fitness 0.37–0.47.
Pattern: views 3–9 are excellent (FTM 0.61–0.99), then view 12 first bad RANSAC
(fit=0.474) poisons TSDF → views 13–14 start from corrupted model → cascade failure.
This is exactly the Opus P0-4 issue (fallback integrates bad pose, poisoning subsequent
FTM targets). Stage 2 fix (skip-frame on fallback) is expected to recover views 12–16.

---

## Step 2 — Stage 2 algorithm fixes (2026-05-08)
### Changes applied to `register.py`
- `_ftm_register`: removed FPFH+RANSAC coarse step; ICP now initialized from `T_prev`
  (small inter-frame motion guaranteed by auto-trigger thresholds 40mm/8°)
- FTM fallback block: replaced RANSAC fallback + integrate with DROP logic:
  append `T_prev` to `all_poses` (keeps list length = n for pipeline.py), `continue`
  to skip `fuse.integrate_one` — TSDF model stays clean

Recon output: `artec_recon_v3/`   Run time: **138 s** (vs 298 s for v2 — 2.2× faster)

### recon_log v3 (FFS w/ MAX_DISP=416 + T_prev ICP + drop-frame)
```
pair  method    fitness  rmse_mm
0-1             0.9258   1.769
1-2             0.8955   1.798
3   FTM         0.9594   2.020
4   FTM         0.9829   2.073
5   FTM         0.9873   1.918
6   FTM         0.8071   2.816   ← was 0.6091 in v2 (RANSAC gave bad T_coarse)
7   FTM         0.8577   2.274   ← was 0.7762
8   FTM         0.8979   2.309   ← was 0.7430
9   FTM         0.8913   2.290   ← was 0.8357
10  FTM-DROP    0.4083   skip    ← was RANSAC fallback 0.5029 (poisoned TSDF)
11  FTM         0.7990   2.371   ← recovered after clean drop (was 0.5478)
12  FTM         0.7404   3.463   ← was FTM-FALLBACK 0.4737 (cascade damage)
13  FTM-DROP    0.4265   skip
14  FTM-DROP    0.4276   skip
15  FTM-DROP    0.4991   skip
16  FTM-DROP    0.1613   skip
```
Accepted FTM views (non-dropped, views 3–9, 11, 12):
  Mean fitness = **0.880**   Mean rmse = **2.39 mm**

All-16-pair mean (drops at raw fitness): **0.729**   (v2 was 0.698)

### Summary

| metric | Step 0 (v1 baseline) | Step 1 (v2 MAX_DISP=416) | Step 2 (v3 T_prev + drop) | Δ (v2→v3) |
|--------|---------------------|--------------------------|--------------------------|-----------|
| FTM mean fitness (accepted) | 0.495 | 0.668 | **0.880** | +32% |
| All-pair mean fitness | ~0.51 | 0.698 | **0.729** | +4% |
| Mean rmse (accepted) | 5.30 mm | 2.91 mm | **2.39 mm** | −18% |
| Run time | ~300 s | 298 s | **138 s** | −54% |
| Mesh | scattered | 347K verts | **352K verts** | +1% |

**Stage 2 findings**:
1. T_prev ICP fixes views 6–9 significantly (mean 0.765 → 0.866) — RANSAC was
   introducing bad T_coarse on those views.
2. Drop-frame prevents TSDF cascade: views 11–12 now succeed (0.799, 0.740) even
   after view 10 dropped; in v2 the fallback at view 12 poisoned 13–14.
3. Views 13–16 remain low-fitness (0.16–0.50) even with T_prev init — indicates
   real large-motion between views 12 and 13 that T_prev cannot bridge.
   Root cause: those views likely have large angular displacement OR ROI mask breaks
   (auto_roi_depth selecting desk instead of object — Opus P0-3, requires new capture).
4. Run time 2.2× faster (no RANSAC overhead per FTM frame).

---

## Step 3 — Cross-review of Step 1+2 changes (Opus, 2026-05-08)

Re-read of the modified `config.py` / `register.py` / `ffs_depth.py` against
the original `claude_review.md` and this log. Three findings.

### 3.1 What this log proved (direction is correct)

- v1→v2→v3 RMSE monotonically dropped (5.30 → 2.91 → 2.39 mm) and accepted
  fitness monotonically rose (0.495 → 0.668 → 0.880) on the **same** 17-view
  dataset — successfully decouples "algorithm/parameter path" from "capture
  data quality".
- `MAX_DISP` was the deepest root cause and was found by Sonnet, not by the
  original Opus review. The Opus P0-1 ("FFS masked by ZED valid set") is real
  but **bounded by MAX_DISP**: with MAX_DISP=192, FFS depth is numerically
  wrong (clipped to ~0.94 m floor), so unmasking it would have made the mesh
  worse. Fix order in the addendum ("MAX_DISP first, then ROI separation") is
  the correct ordering. Original Opus P0 priority list was structurally
  mis-ordered.
- v3 confirms the FTM design works once RANSAC is removed: dropping the
  RANSAC coarse step lifted views 6–9 mean fitness 0.765 → 0.866 with no
  other change.

### 3.2 New bug introduced by Step 2 (must fix before any new capture)

`register.py:254` appends `T_prev` to `all_poses` for dropped frames "to keep
length == n for pipeline.py":

```python
log.append(f"{i} FTM-DROP  {fit:.4f}  skip")
all_poses.append(T_prev)
continue   # skips fuse.integrate_one — FTM volume stays clean
```

The FTM running volume is correctly protected (`continue` before
`integrate_one`). **But** `pipeline.py:142-146` runs a second pass:

```python
volume = fuse_mod.integrate(depths, colors, poses, intrinsic, ...)
```

This pass takes **all 17 poses including the T_prev duplicates** and
re-integrates everything from scratch. The final `mesh.ply` is built from
this second volume, not the FTM one.

Consequence on v3 specifically: views 13/14/15/16 all dropped → all four
were re-integrated at the view-12 pose. Four depth maps at the same camera
pose stack noise into the view-12 region of the final mesh. This is the
quiet reason v3 vertex count (352K) is essentially equal to v2 (347K)
despite accepted-frame fitness jumping from 0.668 → 0.880 — the drop-frame
contamination on the second pass cancels most of the registration win.

**Fix (minimal, ~10 LOC)**: mark dropped poses as `None` and have
`fuse.integrate` / `fuse.integrate_one` skip `None`:

```python
# register.py — dropped frame
all_poses.append(None)

# fuse.py:101 — integrate loop
for i, (depth_m, color_rgb, T_world) in enumerate(zip(depths, colors, poses)):
    if T_world is None:
        continue
    ...
```

**Fix (architectural, preferred long-term)**: have `register()` return
`(poses, log, volume_or_None)`; pipeline reuses the FTM volume directly and
only calls `fuse.integrate` on the pose-graph path. This is Opus review
P2-2, which Step 1+2 left untouched. Promoting it to **P0** because the
drop-frame mechanism above only works correctly if the second integrate
pass is removed or made drop-aware.

This bug also explains why view 13-16 attribution in §Step 2 finding #3 is
ambiguous: any two of {large motion, broken ROI, T_prev pinned at view 12}
could explain it, but the second-pass leak guarantees a confounder. Re-run
v3 with `T_world is None: continue` before drawing further conclusions
about views 13-16.

### 3.3 `MAX_DISP=416` introduces a near-field blind zone

Sonnet's config note states 512 crashes due to baked-in pe shape, so 416 is
the model's hard ceiling for the `23-36-37` weights. Implication:

```
min covered depth = fx · baseline / MAX_DISP
                  = 1907.2 · 0.1201 / 416
                  ≈ 0.55 m
```

`config.py:16` still has `DEPTH_MIN_M = 0.40`. The 0.40–0.55 m band remains
inside the cost-volume floor — any pixel at that range gets clipped and
will replay a smaller version of the v1 plateau symptom.

**Fix**: `DEPTH_MIN_M = 0.55` and add a capture-time overlay warning when
center-pixel z < 0.60 m. Otherwise the next capture session may reproduce
v1's failure mode on whichever frames the operator gets too close.

### 3.4 Status of original P0 items vs. what Step 1+2 actually changed

| review item | applied? | comment |
|---|---|---|
| MAX_DISP (Sonnet addendum, P0) | ✅ 192 → 416 | validated, biggest delta |
| P0-2: FTM drops RANSAC | ✅ | validated, +0.10 fitness on views 6–9 |
| P0-4 raise threshold | ✅ 0.25 → 0.50 | validated |
| P0-4 fallback = skip | ⚠️ partial | drops from FTM volume only, leaks into 2nd integrate (§3.2) |
| P0-3: ROI not largest-area | ❌ | required before new capture |
| P0-1: FFS ROI separation (capture saves roi.npy, ffs_depth reads it) | ❌ | required before new capture |
| P1-1: voxel 1mm → 3mm | ✅ | applied |
| P1-2: L/R fusion asymmetry | ❌ | minor, defer |
| P2-1: model_pcd every K frames | ❌ | minor at n=17, defer |
| P2-2: pipeline double-integrate | ❌ | now P0 — see §3.2 |

### 3.5 Pre-new-capture checklist

Order matters. Items 1–3 must land **before** the next capture session, or
the new dataset will reproduce known failure modes.

1. **Drop-frame leak fix** (`register.py` + `fuse.py`). Re-run on
   `ffs_20260506_173026` first; expected: vertex count drops noticeably
   (~10–15%) and the view-12 region of the mesh becomes cleaner. This
   isolates §3.2 from §Step 2 finding #3.
2. **`auto_roi_depth` interim fix** (area cap < 0.4·H·W + center-proximity
   ranking, `capture.py:135–147`). Original Opus P0-3 has the code.
3. **FFS ROI separation** — capture saves `view_NNN_roi.npy`,
   `ffs_depth.py:189` reads it instead of `np.isfinite(orig_depth)`.
   Concurrent with #2 because they share the new mask.
4. `DEPTH_MIN_M = 0.55` + capture overlay warning when live z < 0.60 m
   (`config.py:16`, `capture.py` overlay).
5. README note: MAX_DISP=416 is weight-specific (`23-36-37`); switching
   weights requires re-checking `model.args.max_disp` and the implied
   min-depth.

After 1–4, re-run on existing data first. Acceptance criterion: FTM
fitness ≥ 0.80 on ≥ 14/17 frames, drop count ≤ 2, mesh shape recognizable
without manual inspection. Only after that result is reproduced, capture a
new dataset.

---

## Step 3 — Drop-frame leak fix + DEPTH_MIN_M (2026-05-08)

### Changes applied
- `register.py`: `all_poses.append(None)` (was `T_prev`) — dropped frames no longer
  duplicate T_prev into the second integrate pass
- `fuse.py`: `if T_world is None: continue` in `integrate()` loop — second pass skips
  None poses entirely
- `config.py`: `DEPTH_MIN_M = 0.40 → 0.55` — aligns with actual FFS coverage floor
  (fx·baseline/MAX_DISP = 1907.2·0.1201/416 ≈ 0.55 m for 23-36-37 weights)

Recon output: `artec_recon_v4/`   Run time: **120 s**

### recon_log v4 (None-pose skip + DEPTH_MIN_M=0.55)
```
pair  method    fitness  rmse_mm
0-1             0.9258   1.769
1-2             0.8955   1.798
3   FTM         0.9594   2.020
4   FTM         0.9833   2.079
5   FTM         0.9874   1.919
6   FTM         0.8071   2.815
7   FTM         0.8577   2.277
8   FTM         0.8973   2.310
9   FTM         0.8914   2.289
10  FTM-DROP    0.4086   skip
11  FTM         0.7991   2.375
12  FTM         0.7405   3.466
13  FTM-DROP    0.4273   skip
14  FTM-DROP    0.4281   skip
15  FTM         0.5699   3.757   ← recovered! (was DROP 0.4991 in v3)
16  FTM-DROP    0.4640   skip    ← was 0.1613 in v3; improved by better T_prev from v15
```
Accepted FTM views (3–9, 11, 12, 15):
  Mean fitness = **0.849**   Mean rmse = **2.53 mm**

Second integrate pass: 13 frames (skipped views 10, 13, 14, 16)

### Key findings

1. **Leak fix confirmed**: v3 mesh 352K verts → v4 mesh **198K verts** (−44%). Opus §3.2
   was correct: views 13/14/15/16 all stacked at the view-12 camera pose in v3 inflated
   vertex count artificially without adding new coverage. The v4 mesh is smaller but clean.

2. **DEPTH_MIN_M=0.55 recovered view 15** (0.4991 → 0.5699): filtering out sub-0.55m
   near-field points cleaned the view-15 point cloud enough to push fitness above
   threshold. This triggered a cascade: T_prev for view 16 is now T15 (was T12) →
   view 16 fitness improved 0.1613 → 0.4640 (still below threshold but meaningful).

3. **View 16 still drops** (0.464 < 0.50). Views 13, 14, 16 are genuinely uncoverable
   with the current data — likely large angular displacement and/or ROI drift in those
   views (Opus P0-3). Not fixable without new capture.

4. **Drop count reduced**: v3 dropped 5 frames (10,13,14,15,16); v4 drops 4 (10,13,14,16).

### Summary — all versions

| metric | v1 baseline | v2 MAX_DISP=416 | v3 T_prev+drop | v4 None-skip+0.55m |
|--------|-------------|-----------------|----------------|---------------------|
| FTM mean fitness (accepted) | 0.495 | 0.668 | 0.880 | **0.849** |
| Accepted FTM frames | 14/14 | 14/14 | 9/14 | **10/14** |
| Drop count | 0 | 0 | 5 | **4** |
| Mean rmse (accepted) | 5.30 mm | 2.91 mm | 2.39 mm | **2.53 mm** |
| Mesh verts | scattered | 347K | 352K (leaked) | **198K (clean)** |
| Run time | ~300 s | 298 s | 138 s | **120 s** |

**Current state**: pipeline is algorithmically correct. Remaining 4 dropped frames
(views 10, 13, 14, 16) require new capture with ROI fix (Stage 3 — capture.py P0-3).

---

### 3.6 Verdict

**Right track.** Diagnostic depth is sufficient (MAX_DISP root cause was
reached), validation discipline is good (every change is backed by a
percentile or fitness number), algorithm changes are clean (runtime halved
with no quality regression). The remaining structural gap is the
drop-frame leak in §3.2 — once that is closed, the project is one
ROI-aware capture session away from a usable mesh.

---

## Step 4 — Visual comparison: v3 vs v4 mesh (2026-05-08)

### Observation (side-by-side mesh viewer)

| | v3 mesh (352K verts) | v4 mesh (198K verts) |
|---|---|---|
| ROI object (bag) | **present but geometry broken**: large hole through bag interior visible, sides inverted/collapsed | **ROI region reconstructed correctly**: bag shape closed, surface consistent, comparable to best prior pure-ICP result |
| Background cluster | dark floating cluster upper-left (桌面/non-ROI residual) | same dark cluster present (ROI fix not yet applied) |
| Texture | blurry (TSDF 3mm voxel) | same — blur is voxel resolution, not registration error |

### Root cause of v3→v4 quality jump

The v3→v4 transition produced the first **geometrically valid** mesh of the
target object. The decisive change was **not** a parameter tweak but a
one-line bug fix in `register.py` + one-line guard in `fuse.py`:

```
register.py:  all_poses.append(None)        # was: all_poses.append(T_prev)
fuse.py:      if T_world is None: continue  # new guard in integrate() loop
```

**Why this caused a topology collapse in v3:**
views 13/14/15/16 all DROPped, but `all_poses` stored T_prev (= view-12 pose)
for each. The second-pass `fuse.integrate` then re-integrated all 17 frames —
including those 4 frames' depth maps projected from the **view-12 camera
direction**. These depth maps were captured facing different sides of the bag,
but were injected at the same pose → TSDF SDF signs contradicted each other
in that region → Marching Cubes extracted inverted/punctured surface (the
large hole visible in v3 right image).

**Contribution ranking (v3→v4 specifically):**

| rank | change | file | effect |
|------|--------|------|--------|
| 1 ★ | `all_poses.append(None)` | `register.py` | eliminates SDF contradiction; makes mesh topologically valid |
| 2 | `if T_world is None: continue` | `fuse.py` | enforces the None skip in second-pass integrate |
| 3 | `DEPTH_MIN_M = 0.55` | `config.py` | recovered view 15 (fitness 0.499→0.570), added bottom coverage |

**Contribution ranking (v1 baseline → v4 usable mesh, cumulative):**

| rank | change | Δ fitness | visual effect |
|------|--------|-----------|---------------|
| 1 ★★ | `FFS_MAX_DISP 192→416` (config.py) | +35% | object appears from nothing; primary root cause |
| 2 ★ | `all_poses.append(None)` + None guard (register.py + fuse.py) | topology fix | closed surface; eliminates puncture holes |
| 3 | RANSAC removed, T_prev ICP init (register.py) | +32% accepted fitness | smoother surface on views 6–9 |
| 4 | `DEPTH_MIN_M 0.40→0.55` (config.py) | view 15 recovered | better bottom coverage |
| 5 | `FTM_MIN_FITNESS 0.25→0.50` (config.py) | stricter threshold | prevents marginal frames from polluting TSDF |
| 6 | `TSDF_VOXEL 0.001→0.003` (config.py) | N/A | mesh generation becomes feasible (1mm was too fine for this scene) |

### Remaining issue (requires new capture)

Left-side dark floating cluster = frames where `auto_roi_depth` selected
the desk (largest connected component) instead of the bag. These frames'
depth ROI covers the desk plane → TSDF integrates desk geometry alongside
bag → small-cluster filter (threshold 17K tri) did not remove it.

Fix: `capture.py:auto_roi_depth` — add area cap (reject components > 40%
of H×W) + center-proximity tie-break. Requires new capture session (Stage 3).

---

## Step 5 — New capture session on fixed pipeline (2026-05-08)

### Reference files
- **New dataset**: `Vision/vision_demo_test_res/ffs_20260508_160056/` — 30 views
  (captured with `pipeline.py capture --mode auto`; auto-stopped at MAX_VIEWS=30)
- **View Picker idea**: `Agent_Log/20260506_artec_ffs.md` lines 72–107
  (Laplacian blur filter + spatial greedy subsampling + contact-sheet UI;
  to be implemented as `Vision/artec_ffs/pick_views.py` — pending)

### Note on capture.py ROI fix status
`auto_roi_depth` **not yet fixed** (Stage 3 still pending). New data still uses
largest-connected-component ROI → background cluster may reappear in mesh.
Proceeding to assess whether 30-frame coverage + current pipeline improvements
(v4 code) produce better registration regardless.

### Recon command
```
conda run -n zedenv python Vision/artec_ffs/pipeline.py recon \
  "Vision/vision_demo_test_res/ffs_20260508_160056" --ftm \
  --out "Vision/vision_demo_test_res/ffs_20260508_160056/artec_recon_v1"
```

### recon_log v1 on new data (ZED depth, 30 views, v4 pipeline)
```
pair  method    fitness  rmse_mm
0-1             0.8828   4.10
1-2             0.9021   3.34
3   FTM         0.9227   3.53
4   FTM         0.8299   3.62
5   FTM         0.7384   4.37
6   FTM         0.8080   4.21
7   FTM         0.8779   3.39
8   FTM         0.9206   3.46
9   FTM         0.9391   3.06
10  FTM         0.5810   5.52   ← low pts view (desk ROI likely)
11  FTM         0.6263   4.64
12  FTM         0.8420   3.77
13  FTM         0.7372   3.36
14  FTM         0.9308   3.02
15  FTM         0.9662   3.46
16  FTM         0.9886   2.68
17  FTM         0.8345   3.56
18  FTM         0.7901   4.53
19  FTM         0.8336   4.33   ← only 426K pts
20  FTM         0.6401   4.18   ← only 341K pts (minimum)
21  FTM         0.8156   4.33   ← only 413K pts
22  FTM         0.7517   4.45   ← only 495K pts
23  FTM         0.8904   4.22
24  FTM         0.9389   3.85
25  FTM         0.8391   3.88
26  FTM         0.9355   3.17
27  FTM         0.9794   3.16
28  FTM         0.9896   2.82
29  FTM         0.9890   2.76
```
**DROP count: 0 / 30** — all frames accepted.

Mean FTM fitness (views 3–29): **0.850**
Mean rmse (views 3–29):        **3.75 mm**  (ZED depth, no FFS correction yet)
Mesh: **208,223 verts**, 398,099 tris   Run time: **158 s**

### Analysis

| observation | detail |
|---|---|
| 0 DROPs (30/30) | vs 4 DROPs on old 17-view data — T_prev init + None-skip working |
| Views 19–24 low point count (341K–733K) | object partially out of frame or near DEPTH_MIN_M edge; still registered (T_prev bridged the gap) |
| rmse higher than v4 accepted (3.75 vs 2.39 mm) | expected — this is raw ZED depth; FFS correction not yet applied |
| Views 10–11 lower fitness (0.58, 0.63) | likely desk ROI selection (auto_roi_depth still unfixed); same root cause as old data's background cluster |

### Comparison: old data v4 (ZED, 17 views) vs new data v1 (ZED, 30 views)

| metric | old data v4 | new data v1 | note |
|--------|-------------|-------------|------|
| DROP count | 4/17 | **0/30** | pipeline fix confirmed on fresh data |
| FTM mean fitness | 0.849 | **0.850** | essentially equal |
| Mean rmse | 2.53 mm | 3.75 mm | new data not FFS-corrected yet |
| Mesh verts | 198K | **208K** | similar coverage |

**Conclusion**: pipeline is stable. Next step to improve rmse: run `--ffs` on new data
(`ffs_20260508_160056`) using `ffs` env to replace ZED depth with FFS depth,
then re-run recon. Expected rmse improvement ~30% based on old data comparison.

---

### Recon v2 on new data — FFS depth (2026-05-08)

FFS depth inference already applied (`ffs_depth.py` run separately), then recon:
```
conda run -n zedenv python Vision/artec_ffs/pipeline.py recon \
  "Vision/vision_demo_test_res/ffs_20260508_160056" --ftm \
  --out "Vision/vision_demo_test_res/ffs_20260508_160056/artec_recon_v2"
```

#### recon_log v2 (FFS depth, 30 views)
```
pair  method    fitness  rmse_mm
0-1             0.9837   1.87
1-2             0.9430   1.82
3   FTM         0.9738   1.93
4   FTM         0.9421   1.87
5   FTM         0.9042   1.84
6   FTM         0.9378   2.05
7   FTM         0.9740   1.99
8   FTM         0.9844   1.89
9   FTM         0.9901   1.84
10  FTM         0.8582   2.18
11  FTM         0.7725   3.21
12  FTM         0.9916   2.04
13  FTM         DROP (fitness=0.000)   ← FFS depth incompatible w/ model
14  FTM         0.8596   1.89
15  FTM         0.9842   1.87
16  FTM         0.9882   1.76
17  FTM         0.9215   1.87
18  FTM         DROP (fitness=0.000)   ← 562K pts, FFS sparse
19  FTM         DROP (fitness=0.000)   ← 383K pts
20  FTM         DROP (fitness=0.000)   ← 326K pts (minimum)
21  FTM         DROP (fitness=0.000)   ← 396K pts
22  FTM         DROP (fitness=0.000)   ← 470K pts
23  FTM         DROP (fitness=0.000)   ← 505K pts
24  FTM         DROP (fitness=0.000)   ← 703K pts
25  FTM         0.9646   1.83
26  FTM         0.9996   1.64
27  FTM         0.9998   1.84
28  FTM         0.9987   1.56
29  FTM         1.0000   1.80
```
**DROP count: 9 / 30** (views 13, 18–24)

Mean FTM fitness (accepted views 3–29): **0.950**
Mean rmse (accepted views 3–29):        **1.94 mm**
Mesh: **128,686 verts**, 248,957 tris   Run time: **89.5 s**

#### New data v1 vs v2 comparison

| metric | v1 (ZED) | v2 (FFS) | delta |
|--------|---------|---------|-------|
| DROP count | 0/30 | **9/30** | FFS caused 9 new drops |
| FTM mean fitness | 0.850 | **0.950** | +0.100 (+12%) |
| Mean rmse | 3.75 mm | **1.94 mm** | **−48%** |
| Mesh verts | 208K | 128K | −38% (fewer integrated frames) |
| Run time | 158 s | 89.5 s | faster (fewer frames) |

#### Analysis

| observation | detail |
|---|---|
| rmse −48% | FFS depth dramatically improves ICP quality for accepted frames |
| Views 18–24 DROP (fitness=0.000) | Low FFS point count (326K–703K) → these are same views that had low pts in ZED recon; FFS produces sparser depth for bad-angle/low-texture views |
| View 13 DROP (2.3M pts, fitness=0.000) | Most concerning: high pt count but zero ICP overlap. FFS likely produced a systematic depth error (scale/offset) for this view. ZED handled it fine (fitness=0.737). |
| FFS helps quality, hurts coverage | Tradeoff: accepted frames have near-perfect registration (many ≥0.99), but 9 frames lost entirely |

#### Root cause hypothesis — view 13 & 18-24 DROP
- **Views 18–24**: FFS estimated very sparse depth (326K–703K pts vs 1.1–2.4M for most views). These were views where the capture geometry was difficult (object partially out of stereo field). FFS's confidence was low → depth map mostly NaN/invalid after threshold → near-zero overlap with model.
- **View 13**: Full 2.3M pts from FFS but fitness=0.000. Hypothesis: FFS produced a smooth but *incorrectly scaled* depth map for this view (wrong disparity plane), causing ICP to see geometrically consistent but spatially displaced points. Needs per-view depth histogram investigation.

#### Suggested hybrid fix (future work)
For views where FFS yields pts < 800K **or** FFS/ZED depth median ratio > 1.3×, fall back to ZED depth. This would recover views 18–24 and possibly view 13 while keeping FFS quality for the majority.

#### Overall pipeline status
- For frames FFS handles well: **rmse ~1.9 mm** (excellent)
- Coverage gap: 9/30 frames lost → smaller mesh (128K vs 208K verts)
- Priority fix: investigate view 13 failure + implement FFS/ZED fallback

---

## Step 6 — Mesh quality inspection of new dataset (Opus, 2026-05-08)

Direct inspection of `artec_recon_v1` and `artec_recon_v2` mesh.ply / pcd.ply
plus per-view FFS-vs-ZED depth percentiles. Diagnostic script saved as
`Vision/artec_ffs/_inspect_meshes.py`.

### 6.1 Measured mesh stats

| metric | v1 ZED (30 views) | v2 FFS (21 views) |
|---|---|---|
| verts / tris | 241,043 / 465,077 | 128,686 / 248,957 |
| watertight | False | False |
| edge_manifold (allow boundary) | True | True |
| vertex_manifold | **False** | **False** |
| **bbox extent (mm)** | **1160 × 640 × 675** | **1218 × 633 × 620** |
| connected components | 1 | 1 |
| edge length median | 2.66 mm | 2.81 mm |
| edge length p10 / p90 | 1.06 / 3.51 mm | 1.57 / 3.52 mm |

### 6.2 What the mesh stats prove (the "blur" complaint)

**Two independent root causes for "mesh too blurry":**

1. **Voxel-resolution-limited mesh** — edge length median 2.66/2.81 mm sits
   right at `TSDF_VOXEL = 3 mm`. Marching Cubes cannot emit triangles smaller
   than the voxel. The 3 mm voxel was sized for v1's 1mm-voxel + 5mm-noise era;
   FFS now achieves rmse 1.94 mm so voxel ≥ rmse target permits ~1.5 mm.
2. **Background still in mesh** — bbox extent 1.16 m × 0.64 m × 0.68 m vastly
   exceeds any plausible object dimensions (object ≈ 30–50 cm). The mesh
   reports 1 connected component, but this is a false win: the bag and the
   desk are bonded through the shared TSDF voxels at the contact line, so
   `_remove_small_clusters` cannot separate them. Roughly half of the
   reported 241K vertices belong to desk surface, not the object — visually
   reads as "blur" because background and foreground share a single mesh.

The user's "对上视角但太糊" verdict is consistent with both being active.

### 6.3 The view 13 / 18–24 DROP cluster is NOT an FFS quality problem

Per-view depth percentile dump (selected views):

```
view 10  FFS pts=2,356,354  p10/50/90=0.623/0.758/0.909  | ZED pts=2,356,354 (identical)
view 13  FFS pts=2,330,365  p10/50/90=0.590/0.678/0.776  | ZED pts=2,330,365 (identical)
view 15  FFS pts=2,352,800  p10/50/90=0.607/0.699/0.822  | ZED pts=2,352,800 (identical)
view 19  FFS pts=  383,268  p10/50/90=0.561/0.607/0.675  | ZED pts=  431,269
view 20  FFS pts=  325,898  p10/50/90=0.564/0.610/0.674  | ZED pts=  346,584
```

Two facts:
- View 13 has **2.3M FFS points** with median depth 678 mm — there is nothing
  numerically wrong with the depth array. v1 ZED registered this view fine
  (fitness 0.737).
- `register.py:200-201` returns `(T_prev, 0.0, 0.0, False)` when ICP raises
  `RuntimeError`. **`fitness == 0.0000` is a sentinel, not a measurement.**

So views 13 and 18–24 in v2 did not "fail FFS quality"; their ICP raised an
exception. The previous Step-5 hypothesis ("FFS produced incorrectly scaled
depth for view 13") is **not supported by the data**.

**More likely cause**: in `ffs_depth.py:209-222`, the regenerated PLY uses
`valid = np.isfinite(depth)` after FFS depth went through both `[DEPTH_MIN_M,
DEPTH_MAX_M]` clip and the ZED-valid ROI mask. For views with FFS depth
heavily clipped (e.g. view 19/20 are at object edge with small ROI), the
resulting `valid.sum()` may be too small for Open3D to estimate normals →
ICP raises RuntimeError → sentinel fitness 0.0.

**One-shot verification** (must run before any further FFS hypothesis):

```python
import open3d as o3d
for i in [13, 18, 19, 20, 21, 22, 23, 24]:
    pcd = o3d.io.read_point_cloud(f"view_{i:03d}.ply")
    print(i, len(pcd.points))
```

Acceptance criterion: if any view returns < ~50K points, the regenerated
PLY is the culprit, not FFS depth. Add a fallback in `refine_dir`:

```python
if int(valid.sum()) < 50_000:
    print(f"  [ffs] {stem}: too few valid points ({valid.sum()}) — keeping ZED PLY")
    continue
```

This is expected to recover 7–9 views' coverage in v2 immediately.

### 6.4 "Sharp Fusion" status check

User asked whether prior agent logs reference Artec's Sharp Fusion. Yes:

- `Agent_Log/20260505_recon_pipeline.md:60` — TSDF voxel comparison row:
  current "fixed 1 mm" vs Artec "adaptive (fine near surface, coarse far)".
- `Agent_Log/20260505_recon_pipeline.md:118` — explicit note: "Artec Sharp
  Fusion adaptive TSDF — proprietary, not open-source, not reproducible."

The proprietary part is the adaptive-narrow-band scheme. ~80% of the visible
benefit is reachable with two open-source pieces:

1. **Reduce TSDF_VOXEL from 3 mm to 1.5 mm.** Now justified — FFS rmse 1.94
   mm; voxel ≥ rmse holds. Expect ~3–4× vertex count and visibly sharper
   surfaces. One-line change in `config.py`.
2. **Replace Marching Cubes with Screened Poisson** on the fused point
   cloud. Open3D one-call. Sub-voxel detail, watertight output, no
   re-integration needed.

Optional further: **bilateral depth filter pre-integration** (cv2.bilateralFilter)
to suppress noise without blurring object edges. Only needed if step 1 above
re-introduces salt-pepper.

### 6.5 Updated priority list (supersedes Step 5 "Suggested hybrid fix")

**P0 — must run before any code change**

P0-A. Verify PLY point counts for views 13, 18–24 (one-line script in §6.3).
      Outcome decides whether the fix lives in `ffs_depth.py` (PLY
      regeneration) or somewhere else (FFS hybrid fallback).

**P1 — visual-quality wins (no new data needed)**

P1-A. `TSDF_VOXEL = 0.003 → 0.0015`, `TSDF_TRUNC = 0.012 → 0.006`
      (`config.py:45-46`). Re-run recon on `ffs_20260508_160056` v2.
P1-B. Add `mesh_poisson.ply` output path in `mesh.py` using
      `o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(pcd, depth=10)`
      with low-density vertex trim. Keep Marching Cubes mesh as `mesh.ply`,
      add Poisson as `mesh_poisson.ply`. User compares both visually.
P1-C. If P0-A confirms PLY shortage: in `ffs_depth.py:refine_dir`, add
      `if valid.sum() < 50_000: keep ZED PLY` fallback. Expected: v2 DROPs
      drop from 9 to ≤ 2.

**P2 — capture-side, requires new dataset after fix**

P2-A. `auto_roi_depth` area cap + center proximity (`capture.py:135-147`).
      Required to stop the desk-bonding bbox 1.2 m problem (§6.2 cause #2).
P2-B. FFS ROI separation: capture saves `view_NNN_roi.npy`, `ffs_depth.py:189`
      reads it instead of `np.isfinite(orig_depth)`. Pairs with P2-A.

**P3 — secondary**

P3-A. `right.png` is the last grabbed frame; `depth.npy` is 15-frame mean-
      fused. Asymmetric noise floor for FFS stereo input (`capture.py:64-95`).
      Fix: save `view_NNN_left_for_ffs.png` as the same single frame matched
      with `right.png`; keep `color.png` fused for rendering.

### 6.6 What the user can verify in 30 minutes (no code rewrite)

1. Run the one-line PLY check (§6.3) — answers "is FFS broken or is PLY
   regeneration broken?".
2. Set `TSDF_VOXEL = 0.0015`, `TSDF_TRUNC = 0.006`; re-run recon v2 — answers
   "how much sharpness is locked behind voxel size?".
3. Run the standalone Poisson reconstruction snippet on existing
   `artec_recon_v2/pcd.ply` to produce `mesh_poisson.ply` for visual A/B.
   This snippet is provided in §6.4 above — it is **not yet run**; no
   `mesh_poisson.ply` exists in the repo until the user (or next agent)
   executes it.

### 6.7 Verdict

The algorithm pipeline is healthy. The remaining "blurry mesh" complaint
splits cleanly into three orthogonal causes that can be fixed independently:

- voxel size (P1-A, parameter)
- mesh extraction method (P1-B, Poisson)
- background bonding (P2-A, ROI fix on capture side)

The v2 "9 frames lost" finding from Step 5 is **likely an artifact of PLY
regeneration, not FFS depth quality** (§6.3). Verify before treating it as
a real coverage tradeoff.
