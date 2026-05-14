# Reconstruction Tools Survey for ZED 2i + Open3D Pipeline

Context: current pipeline (ZED SGM depth -> Open3D TSDF -> marching cubes) produces over-smoothed surfaces (face -> blob). All entries below were verified by fetching their public GitHub pages on 2026-05-14. Rework estimates assume the existing Python script that already pulls rectified ZED stereo + intrinsics + extrinsics.

## 1. Photogrammetry / MVS pipelines

### COLMAP — https://github.com/colmap/colmap
- **Status:** Alive. Latest release v4.0.4 (2026-04-27), 11.7k stars, BSD-3-Clause. ETH/UNC team.
- **Workflow:** `feature_extractor` -> `exhaustive_matcher` -> `mapper` (SfM sparse) -> `image_undistorter` -> `patch_match_stereo` -> `stereo_fusion` -> `poisson_mesher`/`delaunay_mesher`. CLI or `pycolmap` (PyPI, CUDA wheels available) for Python-native control.
- **ZED ingest:** Feed the rectified left+right images as separate views, or skip SfM and import known intrinsics+poses via `cameras.txt`/`images.txt` to go straight to PatchMatch MVS. With 10–30 views and known poses, expect 5–20 min on RTX 5080.
- **Quality vs ZED-SGM:** Substantially sharper on textured regions; will still smear on bare skin/uniform surfaces.
- **Python integration:** Native (pycolmap) for SfM/MVS objects; mesher is CLI subprocess.
- **Rework:** Light — write a `cameras.txt`/`images.txt` exporter from your existing extrinsics; ~1 day.

### AliceVision Meshroom — https://github.com/alicevision/Meshroom
- **Status:** Alive. Release 2025.1.0 (2025-08-19), 12.7k stars, MPL-2.0.
- **Workflow:** Node graph in GUI; also runs headless via `meshroom_batch` CLI on a saved `.mg` graph. ~50% Python codebase, plugin nodes are Python.
- **Quality:** Comparable to COLMAP+OpenMVS; default texturing is better. Same SfM weakness on textureless skin.
- **Python integration:** CLI subprocess; Python plugin SDK exists but adds overhead.
- **Rework:** Medium — easiest path is dump images to disk, call `meshroom_batch`, re-load `.obj` into Open3D. ~1–2 days.

### OpenMVG + OpenMVS — https://github.com/openMVG/openMVG, https://github.com/cdcseacave/openMVS
- **Status:** OpenMVG last tagged v2.1 (2023-12-28), MPL-2.0 — slower but issues still tracked. OpenMVS v2.4.0 (2026-01-20), AGPL-3.0 — active.
- **AGPL caveat:** OpenMVS infects any linked code; OK for research, not for closed redistribution.
- **Workflow:** OpenMVG produces sparse + camera poses; OpenMVS does `DensifyPointCloud` -> `ReconstructMesh` -> `RefineMesh` -> `TextureMesh`. Both CLI-driven; `scripts/python` has glue.
- **Python integration:** CLI subprocess only.
- **Rework:** Medium — same as Meshroom path. AGPL likely disqualifies it for the RSI printing repo if you plan to publish.

## 2. Neural surface / radiance-field tools

### nerfstudio — https://github.com/nerfstudio-project/nerfstudio
- **Status:** Alive. v1.1.5 (2024-11-11), 850 open issues, Apache-2.0.
- **Models:** `nerfacto` (general), `splatfacto` (3DGS), `instant-ngp`, `tensorf`, `neus-facto` (SDF/mesh-friendly).
- **Feeding your poses:** Write a `transforms.json` (NeRF-Blender schema) with `fl_x, fl_y, cx, cy, w, h` plus per-frame `transform_matrix` and `file_path`. `ns-process-data` is bypassed entirely when poses are known. Mesh export: `ns-export poisson` or `tsdf`.
- **Hardware:** RTX 5080 16 GB fine for nerfacto/splatfacto at default resolutions.
- **Python integration:** Native Python (PyTorch). Can be imported as a library.
- **Rework:** Low — `transforms.json` writer ~100 lines.

### SDFStudio — https://github.com/autonomousvision/sdfstudio
- **Status:** Stagnant. Last documented features mid-2023, 2.1k stars, Apache-2.0. Forked from an old nerfstudio version. Use only if you specifically need NeuS/VolSDF/BakedSDF baselines; otherwise prefer nerfstudio's `neus-facto`.
- **Rework:** Medium and risky (env conflicts with current PyTorch/CUDA).

### Neuralangelo — https://github.com/NVlabs/neuralangelo
- **Status:** Official NVlabs release, active issue tracking, MIT-style NVIDIA Source Code License (research/non-commercial).
- **Hardware:** Default config needs >=24 GB VRAM; 16 GB configs exist with quality loss. **Your RTX 5080 16 GB is borderline.**
- **Output:** High-quality SDF -> mesh. Best detail-recovery candidate for faces if VRAM is workable.
- **Python integration:** Native (built on Imaginaire).
- **Rework:** Medium — needs a COLMAP run upstream for poses (or your own pose file converted).

### 3D Gaussian Splatting (graphdeco-inria) — https://github.com/graphdeco-inria/gaussian-splatting
- **Status:** Maintained but explicitly low-bandwidth; CC-BY-NC-SA-style INRIA non-commercial license. Recommends 24 GB VRAM for paper-quality; RTX 5080 16 GB works with `--densify_grad_threshold` tweaks at slightly reduced quality.
- **Output:** Splats, not a mesh. Use SuGaR or 2DGS for surface.

### 2D Gaussian Splatting — https://github.com/hbb1/2d-gaussian-splatting
- **Status:** Alive, recent updates through late 2025. Produces oriented disks; built-in TSDF-based **bounded mesh extraction** and unbounded extraction. Strong choice for getting a clean mesh out of a splat representation.
- **Python integration:** Native PyTorch.
- **Rework:** Low if COLMAP/transforms.json already produced.

### SuGaR — https://github.com/Anttwo/SuGaR
- **Status:** Active (last documented updates 2024-09), MIT, CVPR 2024.
- **Function:** Extracts an editable triangle mesh from a 3DGS reconstruction, optionally Blender-ready.
- **Python integration:** Native PyTorch; entry scripts `train_full_pipeline.py`.
- **Rework:** Low — chains after `gaussian-splatting`.

## 3. Mesh / point cloud editing GUIs

### CloudCompare — https://github.com/CloudCompare/CloudCompare
- **Status:** Alive, v2.13.2 (2024-07-11), GPL-3.0. CLI mode for batch ops. Polygon-draw segmentation, manual cleanup, ICP — best free tool for hand-cleanup of raw point clouds. **CloudComPy** is an unofficial sibling repo providing Python bindings (separate project, status varies).
- **Python integration:** External GUI or CloudComPy bindings.
- **Rework:** None — used as a manual step.

### MeshLab + PyMeshLab — https://github.com/cnr-isti-vclab/meshlab, https://github.com/cnr-isti-vclab/PyMeshLab
- **Status:** Both alive. MeshLab 2025.07 (2025-07-22, GPL-3.0); PyMeshLab v2025.7.post1 (2026-01-30, GPL-3.0).
- **PyMeshLab:** Full filter set scriptable in Python (Poisson recon, screened Poisson, HC Laplacian smoothing, quadric decimation, ambient-occlusion masking, etc.). Drop-in replacement for several Open3D mesh ops.
- **Python integration:** Native bindings (pip).
- **Rework:** Minimal — `import pymeshlab` and chain filters on your existing mesh.

### Polyscope — https://github.com/nmwsharp/polyscope
- **Status:** Alive, v2.6.1 (2026-02-26), MIT. Pip-installable.
- **Function:** Embeddable C++/Python viewer with picking. Add a few lines after your TSDF mesh build to inspect and pick vertices/points. Lighter than Open3D's GUI for ad-hoc inspection.
- **Rework:** Trivial.

### Open3D O3DVisualizer — https://github.com/isl-org/Open3D
- **Status:** Alive, v0.19 (2025-01-08). `geometry.PointCloud.crop` with `SelectionPolygonVolume` (loaded from JSON) is documented; for **interactive** polygon-draw selection use `draw_geometries_with_editing` / `VisualizerWithEditing` (key `K` to lock view, then drag-rect or polygon to select, `C` to crop, `S` to save). Programmatic "drag to delete" beyond this requires writing a custom widget on top of `O3DVisualizer` (gui module) — non-trivial, easier to off-load that step to CloudCompare or Polyscope.

### Blender
- Standard sculpt/cleanup/retopo. Useful as the final manual stage; scriptable via `bpy`. Not a programmatic pipeline component for you.

## 4. Detail-preserving fusion / Sharp-Fusion-style

### VDBFusion — https://github.com/PRBonn/vdbfusion
- **Status:** Quiet. Last release v0.1.6 (2022-03), 5 open issues, MIT. Code is stable and pip-installable; not actively developed but works. Sparse OpenVDB grids let you use higher resolution at the same RAM as Open3D's dense TSDF — useful for face-scale detail.
- **Python integration:** Native bindings (`pip install vdbfusion`).
- **Rework:** Low — swap your `o3d.pipelines.integration.ScalableTSDFVolume` for `vdbfusion.VDBVolume`. Half-day.

### Voxblox — https://github.com/ethz-asl/voxblox
- **Status:** ROS-coupled, C++ only, BSD-3-Clause, 1.6k stars. No first-party Python bindings. Not recommended for your stack.

### BundleFusion
- Original Niessner implementation (`niessner/BundleFusion`) is research code from 2017, Windows/CUDA only, not maintained. Skip.

### Open3D RGBD odometry / integration
- Already what you use; the issue is dense TSDF averaging, not the odometry. Switching to VDBFusion at higher resolution, or running screened Poisson via PyMeshLab on a high-density point cloud, gives more detail than tweaking TSDF voxel size alone.

### Sharp-feature-preserving fusion (anisotropic TSDF, edge-aware Poisson)
- No actively maintained open implementation found for anisotropic/sharp-feature TSDF as of this check. Best practical substitutes: (a) **2DGS bounded mesh extraction** (its TSDF step is detail-preserving on splats); (b) **PyMeshLab screened Poisson** with high octree depth on a fused point cloud; (c) **Neuralangelo** if VRAM allows.

## Recommended next step
For faces specifically, the highest-leverage swap is: keep ZED for poses, run **splatfacto in nerfstudio** (or `gaussian-splatting`) on the RGB frames, then **SuGaR** or **2DGS** for mesh. Fall back to **PyMeshLab screened Poisson** on a cleaned ZED point cloud as a low-risk baseline. **Neuralangelo** is the highest-quality option but VRAM-marginal on the 5080 16 GB.
