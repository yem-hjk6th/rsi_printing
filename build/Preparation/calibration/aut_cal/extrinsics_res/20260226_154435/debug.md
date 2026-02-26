# R8 Debug Log

## Capture Setup
- **Camera:** ZED 2i, HD1080 (1920×1080)
- **ArUco:** DICT_6X6_250, 100mm marker, ID=1
- **Sub-pixel refinement:** OFF (same as R6)
- **Marker position:** Moved to robot foot area (lower, closer to base)
- **Marker distance:** 1.12–1.27m (vs R6's 1.02–1.14m)

## Data
- Raw: 32 rows → 26 unique poses
- Pairwise filter: 26 → 26 (no removal)
- Outlier removal: 26 → 22

## Key Observations

### More "flipped" poses detected
R6 only captured forward-facing poses (B=65~90°). R8 captured additional flipped orientations (A~170°, C~170°), indicating the new marker position is visible from more angles. This is a positive sign for diversity.

### Severe outlier at Pose 13
- Robot: (836, 15, 446), ABC=(10.5, 76.0, 6.5)
- Error: 56.6mm — 8× larger than the next worst
- LOO: 61.7mm
- Likely cause: marginal marker detection at edge of frame or motion blur
- Successfully removed by iterative outlier removal

### T_cam2gripper stability
Tx between R6 and R8 differs by only 0.4mm despite completely different marker positions. This cross-validates both results. Ty and Tz differ by 12-8mm respectively, likely due to R8's higher marker distance reducing corner precision.

## Per-Pose Errors (sorted)
```
Rank  Pose  Error    Robot XYZ
  1    13   56.6mm   (836, 15, 446)   ← outlier
  2     4   18.0mm   (850, 40, 468)
  3    26   10.2mm   (827, 40, 462)
  4    23    9.0mm   (801, 40, 458)
  5    10    6.6mm   (918, 137, 441)
  6    18    6.6mm   (780, -114, 450)
  7    12    6.4mm   (920, 94, 440)
  8    17    6.4mm   (780, -49, 450)
```

## Verdict
Moving marker to robot foot area provided more detections from flipped poses but at greater distance. Net result is comparable to R6 (3.1mm vs 2.9mm after outlier removal) but with worse stability metrics. R6 remains best.
