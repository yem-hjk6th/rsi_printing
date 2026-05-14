# Sharp Fusion and Stereo Detail: Research Summary

## (a) What is publicly known about Artec Sharp Fusion

Sharp Fusion is one of several fusion modes in Artec Studio (alongside Smart Fusion, introduced in AS 18 as default). Artec's documentation describes it as the mode that "perfectly reconstructs fine features" and is "the only mode that unlocks all capabilities of an Artec Spider scanner." It takes a resolution parameter (mm) defining mean inter-point distance: lower values preserve sharper geometry.

No peer-reviewed paper describing the Sharp Fusion algorithm exists in DBLP or CrossRef. The only Artec patent traceable via Google Patents is **WO2009035890A3 / US20090067706A1** (Lapa, 2009), which covers multi-frame surface measurement by merging images in a common reference frame — this is a capture-level patent, not a meshing algorithm. No patent specifically named "Sharp Fusion" or describing its volumetric fusion math was found in USPTO, EPO, or Google Patents searches. Artec does not appear to have published the algorithmic basis of Sharp Fusion in open literature.

**What can be inferred:** Based on the available context, Sharp Fusion is likely a modified TSDF or oriented-implicit-surface reconstruction that applies tighter, anisotropic distance truncation near detected feature edges, rather than the isotropic Gaussian-smoothed TSDF used in standard fusion. The closest open equivalents (below) implement the same idea.

## (b) Open alternatives closest to Sharp Fusion's goal

**Directional TSDF** (Splietker & Behnke, IROS 2019, DOI: 10.1109/iros40897.2019.8968264) is the closest structural analogue: it stores opposing surfaces separately and uses surface-gradient-based ray casting, preventing the classic TSDF problem of merging nearby opposing faces. Open source at github.com/AIS-Bonn/DirectionalTSDF.

**Screened Poisson Reconstruction** (Kazhdan & Hoppe, ACM TOG 2013, DOI: 10.1145/2487228.2487237) adds a data-term screen that pins the reconstructed surface to input samples, recovering fine detail lost in the original 2006 Poisson formulation. This is what the open-source PoissonRecon executable implements and is likely the post-processing step inside Artec Studio for non-Sharp modes.

**Sharp feature consolidation via displacement learning** (Zhao et al., CAGD 2023, DOI: 10.1016/j.cagd.2023.102204) provides a learned pre-processing step that snaps noisy scan points onto their correct sharp-feature skeleton before fusion — a complementary approach to the meshing algorithm itself.

Pauly et al. 2003 (DOI: 10.1111/1467-8659.00675) and Lai et al. 2007 (DOI: 10.1109/tvcg.2007.19) establish the feature-classification geometry that any sharp-preserving pipeline must respect.

**Note on missing references:** No public paper was found for "Robust feature classification and editing" by Pauly et al. (TVCG 2003) — the 2003 Pauly paper is the Eurographics multi-scale feature extraction; the TVCG paper on robust feature classification is Lai et al. 2007. No paper specifically on "anisotropic TSDF" with a verified DOI (distinct from the directional TSDF) was found in literature prior to 2022.

## (c) What helps stereo capture pick up more detail

**Sub-pixel disparity accuracy is the primary lever.** Gehrig et al. (CVIU 2012, DOI: 10.1016/j.cviu.2011.07.008) show that four targeted improvements — fractional disparity sampling, discontinuity-preserving smoothing, a weak stereo constraint, and multi-frame temporal averaging — substantially reduce systematic sub-pixel error. At 1-2 m range, a half-pixel disparity error on a 175 mm baseline (ZED) corresponds to ~3-5 mm depth error; driving this below 0.1 pixel recovers sub-millimetre geometry.

**Resolution selection matters.** Abdelsalam et al. (RAS 2024, DOI: 10.1016/j.robot.2024.104753) show the ZED 2i at HD2K achieves the lowest depth error at 1-2 m; at that range, depth RMS is sub-centimetre. The Middlebury 2014 benchmark (Scharstein et al., LNCS 2014, DOI: 10.1007/978-3-319-11752-2_3) provides the sub-pixel ground truth needed to rigorously evaluate any stereo algorithm's fine-geometry contribution.

**Multi-frame fusion before meshing** is the architectural approach: DTAM (Newcombe et al., ICCV 2011) and KinectFusion (Newcombe et al., ISMAR 2011) both show that fusing many depth frames into a TSDF before surface extraction smooths noise without blurring edges — the same principle Artec Studio applies across scan frames. Voxel hashing (Niessner et al., SIGGRAPH Asia 2013) enables dynamic resolution allocation, concentrating detail budget near detected surfaces.

No peer-reviewed paper specifically characterising the optimal capture distance/overlap/resolution tradeoff for stereo scanning of human faces at 1-2 m was found. The ZED-specific papers (Ortiz 2018, Abdelsalam 2024) provide the closest empirical data.
