# Code Review — `artec_ffs`

**Audience**: future agent picking up this project.
**Scope**: 7 source files under `vision/artec_ffs/` + capture log `Agent_Log/20260506_artec_ffs.md`.
**Verdict**: structure is clean (centralized config, clear module split, README is good), but the bad mesh on `ffs_20260506_173026` is **not a single bug** — four design defects compound. FFS is effectively neutralized, FTM silently regresses to RANSAC, ROI is wrong, and the fitness gate is too loose.

Fix order is P0 → P3. Each item gives file:line, defect, and concrete fix.

---

## P0 — direct root causes of the bad mesh

### P0-1. FFS is masked by ZED's own valid-pixel set, defeating its purpose
**File**: `ffs_depth.py:189` and `ffs_depth.py:199`

```python
roi_mask = np.isfinite(orig_depth)   # ZED-valid pixels only
...
depth[~roi_mask] = np.nan            # FFS depth discarded wherever ZED failed
```

FFS exists to fill ZED SGM holes (low-texture, specular). The current code throws FFS depth away exactly where ZED has no value — i.e. exactly where FFS would help. Net effect: FFS only "refreshes" pixels ZED already solved.

**Fix**: separate two concepts.
- *Geometric ROI* (object boundary): take the outer contour of the connected component or a morphology-dilated version of it, persist as `roi_geom_mask` at capture time (or recompute from `orig_depth` finite mask + morphological closing). Apply this.
- *ZED valid-pixel set*: do **not** use as a mask on FFS output.

```python
# capture.py: save geometric mask once
np.save(out_dir / f"view_{idx:03d}_roi.npy", roi_mask)   # boolean

# ffs_depth.py: read roi_geom (not orig_depth finite-set)
roi_geom = np.load(data_dir / f"{stem}_roi.npy")
depth[~roi_geom] = np.nan
```

Without this fix, every other improvement is bounded by FFS being a no-op.

---

### P0-2. FTM falls back to FPFH+RANSAC, re-introducing the symmetry trap it was designed to avoid
**File**: `register.py:196-207` (`_ftm_register`)

The module docstring (`register.py:13-15`) claims FTM avoids 4-fold rotational-symmetry traps because the model is multi-view fused. But `_ftm_register` re-runs FPFH+RANSAC on every frame and **discards `T_prev`**:

```python
T_coarse = _ransac(s_d, m_d, s_f, m_f, VOXEL_COARSE)   # ignores T_prev
res_g    = _icp_geom(new_pcd, model_pcd, T_init=T_coarse, max_dist=0.05)
res_c    = _icp_colored(new_pcd, model_pcd, T_init=res_g.transformation)
```

Auto-trigger thresholds are 40 mm / 8° (`config.py:21-22`) — small motion. `T_prev` is a high-quality init; RANSAC throws it away and rolls dice on a symmetric object.

**Fix**: drop RANSAC from FTM entirely. Keep only ICP refinement seeded by `T_prev`:

```python
def _ftm_register(new_pcd, model_pcd, T_prev):
    res_g = _icp_geom(new_pcd, model_pcd, T_init=T_prev, max_dist=0.05)
    res_c = _icp_colored(new_pcd, model_pcd, T_init=res_g.transformation)
    ok = res_c.fitness >= FTM_MIN_FITNESS
    return res_c.transformation, res_c.fitness, res_c.inlier_rmse, ok
```

This is the actual reason the log shows FTM fitness 0.25–0.65 — it is not "low overlap", it is RANSAC mis-seating each frame.

---

### P0-3. Auto-ROI picks largest connected component → frames the desk
**File**: `capture.py:135-147` (`auto_roi_depth`)

```python
largest = 1 + np.argmax(stats[1:, cv2.CC_STAT_AREA])
return (labels == largest)
```

Once any desk surface falls in `[DEPTH_MIN_M, DEPTH_MAX_M] = [0.40, 1.20]` m, its connected component is larger than the object, and ROI flips to the desk. This is the "ROI inaccurate" symptom called out in `Agent_Log/20260506_artec_ffs.md` and the dominant cause of bad capture data.

**Fix** (preferred order):
1. Implement the **MobileSAM click-ROI** (Plan B in the log). One click per first frame; propagate via the previous frame's mask center; save mask alongside depth.
2. As a transitional cheap fix: filter components by **center proximity** to image center and an **area cap** (object expected ≪ frame area), e.g.

```python
H, W = depth_m.shape
img_center = np.array([W/2, H/2])
cands = []
for k in range(1, n_labels):
    cx_, cy_ = stats[k, cv2.CC_STAT_LEFT] + stats[k, cv2.CC_STAT_WIDTH]/2, \
               stats[k, cv2.CC_STAT_TOP]  + stats[k, cv2.CC_STAT_HEIGHT]/2
    area = stats[k, cv2.CC_STAT_AREA]
    if area > 0.4 * H * W:        # too big → likely background
        continue
    dist = np.linalg.norm([cx_ - img_center[0], cy_ - img_center[1]])
    cands.append((dist, k))
if not cands:
    return None
return labels == min(cands)[1]
```

This is the single highest-leverage capture-side fix.

---

### P0-4. `FTM_MIN_FITNESS = 0.25` is far too lax and the fallback path makes things worse
**File**: `config.py:37`, `register.py:256-265`

The fallback when FTM fitness is below threshold runs *adjacent-frame RANSAC* (`pairwise(pcds[i-1], pcds[i])`) — exactly the symmetry-trap-prone path the FTM design was meant to escape. So a low-confidence frame is replaced with an even less reliable estimate, and that pose is then **integrated into the model**, poisoning every subsequent FTM target.

In the log, view 4 fitness=0.153 → fallback → all later views drift.

**Fix**:
- `FTM_MIN_FITNESS = 0.5` (raise floor).
- Fallback should **skip the frame** (do not append to `all_poses`, do not integrate). Continue with the last good model. Log it as `FTM-DROP`.

```python
if not ok:
    log.append(f"{i} FTM-DROP  {fit:.4f}  {rmse*1000:.3f}mm")
    continue   # do not extend all_poses, do not integrate
T_prev = T
all_poses.append(T)
fuse.integrate_one(volume, depths[i], colors[i], T, intrinsic, ...)
```

Skipping a frame is strictly safer than fusing a bad pose.

---

## P1 — parameter / data alignment

### P1-1. TSDF voxel is finer than depth noise
**File**: `config.py:45` (`TSDF_VOXEL = 0.001`)

ZED depth noise at ~800 mm is ~2–5 mm; FFS at FFS_VALID_ITERS=8 is comparable. A 1 mm voxel etches noise into the surface.

**Fix**: `TSDF_VOXEL = 0.003`, `TSDF_TRUNC = 0.012` (keep ~4× ratio).

### P1-2. Asymmetric L/R fusion before FFS
**File**: `capture.py:64-95` (`grab_fused`)

Left depth is mean-fused over N frames; right image is the **last grabbed frame** (`right_bgr` is overwritten in the loop). FFS then runs on `last-left.png` + `last-right.png`, so the stereo pair itself is internally consistent — but `view_NNN_color.png` is `last-left` while `view_NNN_depth.npy` (ZED) is fused. Color/ZED-depth are slightly misaligned in time.

**Fix** (pick one):
- Save *fused* left as `color.png` for the rendering pipeline, *and additionally* save the matching last-left as `left_for_ffs.png` paired with `right.png`. FFS uses the latter pair.
- Or: don't fuse depth in `grab_fused` (single-frame everywhere). Simpler, removes the asymmetry, slightly noisier.

Recommend the first — keep fused depth's noise reduction; just stop reusing `color.png` for FFS input.

---

## P2 — performance / redundancy

### P2-1. FTM re-extracts the entire TSDF point cloud every frame
**File**: `register.py:278` (`model_pcd = _extract_model_pcd(volume)`)

Comment already flags this ("or every k frames for speed"). 17 frames × full extract is the dominant cost.

**Fix**: re-extract every K=3 frames; reuse `model_pcd` in between.

```python
if (i - FTM_WARMUP_FRAMES) % 3 == 0:
    model_pcd = _extract_model_pcd(volume)
```

### P2-2. Pipeline integrates twice
**File**: `pipeline.py:142-146` after `register.py:242-275`

`register_frame_to_model` already integrates every frame into a TSDF volume. Then `pipeline.reconstruct` calls `fuse.integrate(...)` on all frames again from scratch and discards the FTM volume.

**Fix**: have `register()` return `(poses, log, volume_or_None)`. In pipeline:

```python
poses, log_lines, volume = reg_mod.register(...)
if volume is None:    # pose-graph path
    volume = fuse_mod.integrate(depths, colors, poses, intrinsic, ...)
```

Cuts wall-time roughly in half on the FTM path.

---

## P3 — documentation / minor

### P3-1. Loop-closure comment lies
**File**: `register.py:147-150`

```python
# Only add if there is enough overlap
info  = _info_matrix(pcds[0], pcds[i], T_0i)
pg.edges.append(o3d.pipelines.registration.PoseGraphEdge(0, i, T_0i, info, uncertain=True))
```

No overlap check happens. Either implement it (e.g. fitness from a quick ICP, gate on `LOOP_OVERLAP_MIN` from `config.py:42`) or delete the misleading comment.

### P3-2. `cv2.resize(..., dsize=None, fx=, fy=)`
**File**: `ffs_depth.py:116`

`dsize` should be `(0, 0)` when using `fx`/`fy`. Tolerated by OpenCV 4.x today; not guaranteed.

```python
img_left_rgb  = cv2.resize(img_left_rgb,  (0, 0), fx=FFS_SCALE, fy=FFS_SCALE)
```

### P3-3. timm shim is brittle and semi-redundant
**File**: `ffs_depth.py:28-54`

Hard-codes timm submodule names. README already recommends running in `ffs` env where `timm.layers` exists. Replace the shim with a clear error:

```python
try:
    import timm.layers  # noqa
except ImportError:
    raise ImportError("Run inside the 'ffs' conda env (needs timm>=0.9).")
```

### P3-4. `import shutil` inside loop
**File**: `ffs_depth.py:203`

Move to module top.

---

## Suggested execution order

1. P0-3 (auto-ROI) — without this, capture data itself is poisoned, every recon stays bad.
2. P0-2 (FTM drops RANSAC) — gives a real frame-to-model, not a disguised pairwise.
3. P0-4 (raise threshold + skip on fail) — stops single-frame poisoning.
4. P0-1 (FFS ROI separation) — only now does FFS actually contribute.
5. P1-1 (voxel size) — visible smoothness improvement.
6. P1-2, P2-x, P3-x — quality-of-life and speed.

After P0-1..P0-4 + P1-1, expect FTM fitness to move from 0.4–0.6 into ≥0.8 territory on the same `ffs_20260506_173026` data. If it does not, suspect FFS depth itself (run `ffs_depth.py` standalone, eyeball one disparity map vs. one ZED depth — the priority-1 task in the log).

---

## What is fine / leave alone

- Module split (`config / capture / ffs_depth / register / fuse / mesh / pipeline`) is the right shape; do not refactor.
- `config.py` centralization is good; new params should keep landing here.
- `mesh.py` post-processing chain (degenerate → small clusters → Laplacian) is standard and fine.
- Pose convention (`c2w` for nodes, `inv(T)` for `volume.integrate`) is consistent across `register`/`fuse`/`pipeline`. Do not "clean up".
- Fallback math at `register.py:262` (`T = all_poses[-1] @ np.linalg.inv(T_fb)`) is correct given `pairwise` returns source→target. If P0-4 changes fallback to skip, this code is dead and can be removed.

---

## Cross-Review Addendum (Sonnet 4.6 vs Opus — 2026-05-08)

This section was written after comparing the above Opus review with an independent Sonnet review of the same codebase and the same agent log. It is addressed to whichever agent picks this up next.

### Points both reviews agree on

- Auto-ROI (capture.py `auto_roi_depth`) picks the largest connected component, which is the desk. Opus gives a more complete interim fix (area cap + center proximity). Use Opus's version.
- `FTM_MIN_FITNESS = 0.25` is too low. Both reviews recommend raising it; Opus additionally identifies that the fallback writes the bad pose into the TSDF model and poisons all subsequent FTM targets. Opus's "skip the frame" conclusion is stronger than "raise the threshold alone". Use Opus's fix.
- Loop-closure edges are added unconditionally despite the comment claiming overlap gating. Both agree: either gate properly or delete the comment.

### Point Sonnet found that Opus missed — treat as P0 blocker

**`FFS_MAX_DISP = 192` is far too small for ZED 2i at the actual working distance (`config.py:67`).**

ZED 2i HD2K calibrated intrinsics: fx ≈ 1577 px, baseline = 120 mm.
Required disparity = fx × baseline / depth:

| depth | required disparity | covered by MAX_DISP=192? |
|-------|--------------------|--------------------------|
| 400 mm | 473 px | ❌ 2.5× over limit |
| 600 mm | 315 px | ❌ 1.6× over limit |
| 800 mm | 237 px | ❌ over limit |
| 940 mm | 201 px | ≈ boundary |

FFS builds a cost volume over [0, MAX_DISP] disparity bins. Any scene point closer than ~940 mm falls outside the searchable range; the network either clips output to 192 px (→ depth ≈ 940 mm, a flat plane artifact) or produces undefined values. The entire working volume of the scanner (0.4–0.9 m) is outside the cost-volume range.

**This is the single root cause of the bad mesh on `ffs_20260506_173026`.** Every other fix is bounded by this: P0-1 in the Opus review (FFS ROI separation) cannot improve results if FFS depth is numerically wrong everywhere. Fixing MAX_DISP must precede P0-1.

**Fix**: set `FFS_MAX_DISP = 512` in `config.py`. Alternatively set `FFS_SCALE = 0.5` (halves image resolution, halves required disparity range) combined with `FFS_MAX_DISP = 256`. The scale=0.5 path is faster but loses some fine-detail accuracy. Prefer `MAX_DISP=512` at `FFS_SCALE=1.0` if VRAM allows; fall back to scale=0.5 if OOM.

**Validation before any other fix**: run `ffs_depth.py` standalone on one view, print `np.nanpercentile(depth, [10,50,90])`. Expected output if correct: values in [0.4, 1.2] m range. If output is clustered around 0.94 m or is mostly NaN, MAX_DISP is confirmed as the problem.

### Conflict on "what neutralizes FFS"

- **Opus P0-1** claims FFS is neutralized because its output is masked to ZED's valid-pixel set, so it only refreshes pixels ZED already solved.
- **Sonnet** claims FFS is neutralized because MAX_DISP=192 makes the depth values numerically wrong regardless of masking.

**These are not mutually exclusive.** However, the fix order matters:

1. Fix MAX_DISP first. This makes FFS produce correct disparity.
2. Then fix the ROI masking (Opus P0-1). This lets FFS fill ZED holes.

Applying Opus P0-1 before fixing MAX_DISP would expose *wrong* FFS depth in ZED-hole regions, making the mesh worse. Do not reorder.

### Point Opus found that Sonnet missed

**`_ftm_register` discards `T_prev` and runs FPFH+RANSAC from scratch on every frame (`register.py:196–207`).**

The auto-trigger thresholds are 40 mm / 8° — small inter-frame motion. `T_prev` is a high-quality initialization. RANSAC ignores it and rolls dice. This is a second independent cause of the 0.25–0.65 fitness range (in addition to wrong FFS depth). Even with correct FFS depth, this RANSAC re-initialization would degrade FTM on any object with rotational symmetry.

Fix: remove RANSAC from `_ftm_register`, initialize both ICP steps from `T_prev` directly (Opus P0-2 code is correct).

### Point Opus found that Sonnet missed — performance

`TSDF_VOXEL = 0.001` (1 mm) is finer than ZED/FFS depth noise (~2–5 mm at 800 mm). Noise gets voxelized into the surface. Change to `TSDF_VOXEL = 0.003`, `TSDF_TRUNC = 0.012`.

### Revised fix priority (supersedes Opus's "Suggested execution order")

Validation first — do not edit code until these two checks are done:

```
# Check 1: FFS depth value distribution (10 min)
conda activate ffs
python Vision/artec_ffs/ffs_depth.py "Vision/vision_demo_test_res/ffs_20260506_173026"
python -c "
import numpy as np, glob
for f in sorted(glob.glob('Vision/vision_demo_test_res/ffs_20260506_173026/view_*_depth.npy'))[:3]:
    d = np.load(f); print(f, np.nanpercentile(d,[10,50,90]))
"
# Expected if FFS correct: [0.4..0.9] m range
# If clustered near 0.94 m: MAX_DISP is the problem (confirmed)

# Check 2: ROI mask sanity — add one debug line in auto_roi_depth, capture one frame
# or inspect existing view_*_depth.npy NaN pattern vs color.png manually
```

Code changes in order:

| # | change | file | what changes |
|---|--------|------|--------------|
| 1 | `FFS_MAX_DISP = 192 → 512` | `config.py:67` | FFS cost volume covers actual depth range |
| 2 | Auto-ROI: add area cap + center proximity (Opus P0-3 code) | `capture.py:135–147` | Capture data ROI targets object, not desk |
| 3 | `_ftm_register`: remove RANSAC, init ICP from `T_prev` (Opus P0-2 code) | `register.py:196–207` | FTM is actually frame-to-model, not disguised pairwise |
| 4 | `FTM_MIN_FITNESS = 0.25 → 0.50`; fallback = skip frame (Opus P0-4 code) | `config.py:37`, `register.py:256–265` | Bad pose no longer poisons TSDF model |
| 5 | FFS ROI: use geometric mask from `view_NNN_roi.npy`, not ZED valid-pixel set (Opus P0-1 code) | `ffs_depth.py:189–199`, `capture.py:save_keyframe` | FFS fills holes ZED cannot solve |
| 6 | `TSDF_VOXEL = 0.001 → 0.003`, `TSDF_TRUNC = 0.004 → 0.012` | `config.py:45–46` | Noise not voxelized into surface |
| 7 | `cv2.resize(dsize=None → (0,0))` | `ffs_depth.py:116` | OpenCV API correctness |
| 8 | `import shutil` to module top | `ffs_depth.py:203` | Minor cleanliness |

After changes 1–4 (no new capture needed, rerun on existing `ffs_20260506_173026`):
- Expected: FTM fitness ≥ 0.75 on most frames, mesh shows recognizable object shape.
- If fitness still low after 1–4: run Check 1 again, confirm FFS depth is now in range before proceeding to change 5.

Changes 5–8 require a fresh capture session (new `right.png` files with saved `roi.npy`). Do not apply change 5 to existing data directories.
