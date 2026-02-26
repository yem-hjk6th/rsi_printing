# R7 Debug Log

## Capture Setup
- **Camera:** ZED 2i, HD1080 (1920×1080)
- **ArUco:** DICT_6X6_250, 100mm marker, ID=1
- **Sub-pixel refinement:** ON — `CORNER_REFINE_SUBPIX`, winSize=5, maxIter=50, minAcc=0.01
- **Sync:** time.time() with 0.12s freshness threshold

## Data
- Raw: 37 rows → 31 unique poses
- Pairwise filter: 31 → 31 (no removal)
- Outlier removal: 31 → 28

## Key Finding
Sub-pixel refinement worsened all metrics:
- Mean error: 2.9mm → 9.1mm
- Max error: 5.1mm → 15.9mm
- Method consensus: 2.1mm → 24.6mm

## Root Cause Analysis
1. 100mm marker at ~1m → ~100px edge length in HD1080
2. Default integer-pixel corner detection already achieves <0.5px accuracy at this scale
3. Sub-pixel gradient fitting picks up print noise (ink texture, paper grain, micro-reflections)
4. Systematic corner displacement → biased solvePnP → biased calibration

## Worst Poses (LOO)
- Pose 3 (850,40,480) B=65°: 28.0mm — extreme tilt amplifies corner error
- Pose 8 (850,40,460) B=82°, C=18°: 20.1mm
- Pose 27 (868,40,439): 19.8mm

## Verdict
Sub-pixel refinement should NOT be used with printed markers at this distance/resolution.
R6 (no sub-pixel) remains the gold standard result.
