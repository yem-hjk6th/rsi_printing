# ffs_poisson_detail — work log

> Status & direction notes for any agent (incl. on other machines) picking this
> up. Complements the architected memory at
> `~/.claude/projects/g--UVA-Research-Update-rsi-printing/memory/`
> (`project_ffs_poisson_detail.md`, `reference_reconstruction_research.md`).
> See `README.md` for usage. Keep this file current when direction changes.

## What this package is

Detail-recovery fork of `Vision/artec_ffs/`. Same capture / registration / FFS
stages (copied verbatim); the fusion + meshing stages are swapped for a
**Screened Poisson** path, plus a fast post-capture **data-quality gate**
(`quality_check.py`). Created 2026-05-14 because artec_ffs's 3 mm TSDF +
Marching Cubes turned a scanned face into a featureless blob.

Naming convention (user-defined): reconstruction variants are siblings of
`artec_ffs/`, named `ffs_<technical-route>_<core-change>`. Harder paradigms get
their own packages — see "Upcoming" below.

## Current direction

Stay on the **volumetric depth-fusion** route (KinectFusion lineage) and squeeze
the detail-recovery ladder in `config.py`:
  Rung 1 finer TSDF voxels (FUSE_VOXEL 1.5 mm)
  Rung 2 Screened Poisson meshing (MESH_BACKEND="poisson", default)
  Rung 3 Poisson octree depth + low-density vertex trimming
This is the "easy→medium" tier. It is intentionally NOT trying to be neural.

## Verified working (2026-05-14, YE-SERVER / RTX 5080, `ffs` conda env)

- Full workflow: `capture --mode auto` → `qc` → move bad views to `_skipped/`
  → `recon`. On a 30-view auto capture, after dropping QC-flagged views
  (0-2 jumpy start, 3 had a 175° rotation pair, 19-21 sparse dropout), recon on
  the remaining 23 views registered **23/23 with zero FTM drops** →
  290k verts / 575k tris. Visibly more facial detail than artec_ffs's 67k-vert
  Marching Cubes output.
- `qc` on 30 views: ~47 s (parallelised across 8 workers).

## Known issues / TO-DO  (roughly priority order)

1. **ROI segmentation pulls in background.** `capture.py auto_roi_depth` takes
   the largest connected depth component in [DEPTH_MIN_M, DEPTH_MAX_M]. When the
   subject is close to walls / pillows / furniture, the background is connected
   and gets reconstructed too (the "dark blob + scene" artefact). Fix candidates:
   tighten the depth band per-frame around the subject median, add a
   largest-component-by-volume filter, or a simple foreground depth gate.
   This is a pipeline bug, NOT a capture-physics limit.
2. **register.py pose-graph warmup is fragile.** It always uses the first 3
   views; if any of them is bad, ColoredICP throws "No correspondences found"
   and the whole run dies. Fix candidates: catch + fall back to geometric ICP,
   or auto-pick the best-connected consecutive triple for warmup. NOTE:
   register.py is copied verbatim from artec_ffs — fixing it here forks it.
3. **Non-rigid subject motion is a hard limit, not a bug.** Multi-view fusion
   assumes a rigid scene. A person moving over a ~30-frame handheld scan causes
   broken faces / misalignment. For people: capture faster, fewer frames, or
   move to a route that tolerates it. Document, don't "fix".
4. **qc speed.** ~47 s for 30 views is acceptable but the per-pair FPFH+RANSAC
   probe still dominates. Could drop `QC_PROBE_RANSAC_ITER` further or coarsen
   `QC_PROBE_VOXEL`, at some cost to WARN-list stability (already a bit noisy
   run-to-run because RANSAC is stochastic).
5. **Capture warnings (cosmetic).** ZED SDK logs `ULTRA is deprecated, use
   NEURAL` and `depth_minimum_distance 300mm clamped to 400mm`. Harmless; tidy
   `capture.py open_zed()` when convenient.

## Upcoming technical routes (separate packages, not here)

If the Poisson ladder is not enough, the research dossier in `doc/refs/`
points to the next routes. Each should be its own `ffs_*` package:
- `ffs_gs_*` — 2D Gaussian Splatting / SuGaR via nerfstudio, fed the ZED poses
  this pipeline already produces. Fast to train, mesh-extractable. Best
  near-term quality bet on a 16 GB GPU.
- `ffs_neural_*` — Neuralangelo (NeuS + hash grid). Highest fidelity, but
  VRAM-marginal on RTX 5080 16 GB.
See `doc/refs/reconstruction_intro_summary.md` and `reconstruction_tools_survey.md`.

## Pointers

- Research dossier (verified citations + tool survey): `doc/refs/`
- Machine setup / FFS_REPO_ROOT side-channel: see artec_ffs README + memory
  `project_artec_ffs_localization.md`, `reference_local_config_pattern.md`
- `config_local.py` is per-machine and gitignored — recreate it on a new box.
