# Surface Reconstruction Literature — Grouped Summary

Pipeline context: ZED 2i → FoundationStereo depth → ColoredICP pose graph → Open3D TSDF (3 mm) → marching cubes → Laplacian smooth. Symptom: smooth blob, missing eyes/mouth.

---

## Topic A — Foundational paradigms

**COLMAP SfM** [Schonberger & Frahm, CVPR 2016] + **MVS** [Schonberger et al., ECCV 2016]: globally-consistent camera poses + dense depth from images — the offline alternative to your ICP pose graph.
**MVS Survey** [Furukawa & Hernandez, 2015]: explains why TSDF averaging erases thin-surface detail.
**KinectFusion** [Newcombe et al., ISMAR 2011]: defines the TSDF truncation model your Open3D volume inherits; truncation band + voxel size directly set the feature-size floor.
**BundleFusion** [Dai et al., ToG 2017]: adds online loop-closure and re-integration — shows how to fix ICP drift that compounds blob artefacts over a full torso scan.

**Takeaway**: your detail loss starts here — 3 mm voxels + noisy depth = 6–9 mm effective blur, larger than eyelid ridges.

---

## Topic B — Detail-preserving meshing

**Poisson** [Kazhdan et al., SGP 2006] + **Screened Poisson** [Kazhdan & Hoppe, ToG 2013]: fit a global implicit to oriented normals; screened variant forces surface through your points instead of shrinking away from them.

**Takeaway**: replace marching cubes on TSDF with Screened Poisson on your fused point cloud — immediate, zero-training upgrade available inside Open3D.

---

## Topic C — Neural surface reconstruction

**NeRF** [Mildenhall et al., ECCV 2020]: volume-rendered MLP, photorealistic but no clean surface.
**NeuS** [Wang et al., NeurIPS 2021] + **VolSDF** [Yariv et al., NeurIPS 2021]: model density via SDF → surface is a sharp zero-crossing → recovers fine features that TSDF washes out.
**Instant-NGP** [Muller et al., ToG 2022]: hash-grid encoding cuts optimisation to minutes, making NeuS-style runs feasible per scan session.
**Neuralangelo** [Li et al., CVPR 2023]: NeuS + hash grid = current SOTA for high-fidelity geometry from multi-view RGB, directly comparable to your use case.
**BakedSDF** [Yariv et al., SIGGRAPH 2023]: trains neural SDF then exports a triangle mesh — the full neural-to-mesh pipeline.

**Takeaway**: Neuralangelo from your ZED frames would likely recover the facial detail your TSDF pipeline loses, at the cost of an offline optimisation step (~20 min/scene on RTX 5070).

---

## Topic D — 3D Gaussian Splatting and mesh extraction

**3DGS** [Kerbl et al., SIGGRAPH 2023]: real-time rasterisation, but Gaussians are not surfaces.
**2DGS** [Huang et al., SIGGRAPH 2024]: surface-aligned disks give view-consistent normals/depth — mesh-extractable directly.
**SuGaR** [Guedon & Lepetit, CVPR 2024]: regularises 3DGS to lie on surfaces, exports textured mesh — practical pipeline output for RSI printing.
**Gaussian Surfels** [Dai et al., SIGGRAPH 2024]: GS disks + Screened Poisson = highest-quality GS surface currently published.

**Takeaway**: GS-based methods train fast and export meshes; Gaussian Surfels is the current best choice if you want a mesh from a ZED capture session in under 30 min.

---

## Topic E — Depth quality

**RAFT-Stereo** [Lipson et al., 3DV 2021]: shows why iterative update handles edge regions better than single-pass SGM.
**FoundationStereo** [Wen et al., CVPR 2025]: the model you run; cite it. Zero-shot generalisation to skin/hair is the key capability.
**DPT** [Ranftl et al., ICCV 2021] + **Depth Anything V2** [Yang et al., NeurIPS 2024]: monocular fallbacks; useful as cross-check for stereo confidence maps.

**Takeaway**: FoundationStereo already gives you near-optimal stereo depth; the blob problem is downstream (TSDF resolution / ICP drift), not in the depth estimator.

---

## Topic F — Commercial scanner benchmarks

**Schipper et al., Scientific Reports 2024**: quantified Artec Eva vs Space Spider vs 3dMD on faces — Artec Space Spider achieves ~0.05 mm accuracy. Sets the accuracy bar your ZED 2i pipeline must approach to be competitive.

**Takeaway**: a structured-light scanner is ~60x more accurate per point than your current pipeline at 3 mm voxels; closing that gap requires either sub-mm voxels + better depth, or a neural approach.
