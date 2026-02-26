# R6 Calibration Debug Log

## Capture Setup
- **Camera:** ZED 2i
- **Resolution:** HD1080 (1920×1080) — upgraded from R5's HD720
- **ArUco:** DICT_6X6_250, 100mm marker, ID=1 (was 50mm ID=0 in R5)
- **Sub-pixel refinement:** OFF (default detector parameters)
- **Sync method:** time.time() with 0.12s freshness threshold
- **SVD orthogonalization:** Applied in calibrate.py

## Data Statistics
- Raw CSV rows: 37
- After dedup: 31 unique poses
- Pairwise filter: 31 → 31 (no removal — good consistency)
- After outlier removal (calibrate.py iterative): 31 → 23

## Pose Diversity
| Axis | Range | Assessment |
|------|-------|------------|
| X | 760–920 mm (160mm) | OK |
| Y | -140–140 mm (280mm) | OK |
| Z | 430–480 mm (50mm) | OK |
| A | -174–172° (346°) | OK (includes 180° flips) |
| **B** | **65–90° (25°)** | **Narrow — limited by marker visibility** |
| C | -167–175° (342°) | OK (includes 180° flips) |

## Per-Pose Back-Substitution Errors
```
 #   Err     World XYZ (mm)           Robot XYZ
 1   1.4    (506.4, -316.8,  11.7)   (850, 40, 460)
 2   1.9    (507.9, -318.4,  11.0)   (850, 40, 460)
 3   8.2    (512.3, -321.9,   7.0)   (850, 40, 480)  ← worst
 4   1.6    (507.5, -316.8,  11.1)   (850, 40, 460)
 5   1.0    (506.8, -317.0,  11.2)   (850, 40, 470)
 6   1.9    (507.6, -318.1,  11.8)   (850, 40, 460)
 7   2.5    (506.7, -315.2,  11.1)   (850, 40, 460)
 8   6.1    (506.0, -314.2,   5.7)   (850, 40, 460)
 9   5.7    (509.9, -316.9,  15.0)   (850, 40, 470)
10   7.7    (499.9, -321.0,  13.5)   (850, 40, 470)
11   8.6    (502.1, -318.0,   3.1)   (893, 101, 452) ← 2nd worst
12   3.8    (509.9, -317.9,  11.7)   (920, 140, 440)
13   4.8    (510.6, -319.6,  11.3)   (920, 138, 440)
14   7.0    (507.5, -323.5,   7.2)   (920, 74, 440)
15   3.0    (505.3, -320.1,   9.2)   (920, -19, 440)
16   3.3    (505.1, -320.4,   9.2)   (920, -84, 440)
17   2.2    (504.5, -316.6,  11.7)   (920, -140, 440)
18   4.3    (503.2, -314.6,  10.2)   (892, -88, 442)
19   4.3    (505.0, -313.6,  11.4)   (861, -30, 444)
20   3.3    (505.6, -314.4,  10.8)   (830, 27, 446)
21   2.6    (507.6, -315.5,  11.1)   (799, 84, 449)
22   4.6    (509.7, -316.4,  13.4)   (780, 120, 450)
23   3.4    (505.2, -319.4,  13.4)   (780, 112, 450)
24   6.9    (500.5, -321.3,  11.7)   (780, 46, 450)
25   2.6    (507.5, -315.7,   9.5)   (889, 26, 432)
26   2.7    (506.4, -315.0,  11.1)   (900, 40, 430)
27   5.9    (504.5, -315.3,   5.5)   (868, 40, 439)
28   4.6    (505.0, -315.6,   6.7)   (801, 40, 458)
29   1.8    (506.2, -318.7,  12.1)   (760, 40, 470)
30   6.1    (506.3, -319.1,  16.6)   (763, 40, 470)
31   4.7    (504.8, -319.9,  14.5)   (827, 40, 462)
```

Marker world coords: mean=(506.2, -317.6, 10.7) mm, std=(2.7, 2.5, 2.9) mm

## LOO Cross-Validation
- Mean: 4.2mm, Max: 9.8mm, Std: 2.0mm
- T stability: (0.9, 1.3, 0.8) mm

Worst LOO poses:
- Pose 3: 9.8mm (850,40,480) — B=65° (extreme tilt)
- Pose 11: 7.7mm (893,101,452) — flipped orientation
- Pose 10: 7.4mm (850,40,470) — A=8°,C=15° combination
- Pose 24: 7.2mm (780,46,450) — A=25°,B=82°,C=23°
- Pose 14: 6.4mm (920,74,440) — A=148° (near-flip)

## Issues Encountered During Capture
1. **MARKER_ID mismatch:** aruco_gen2.py assigns ID=1 to 100mm markers (not ID=0). 
   This caused empty CSVs until fixed.
2. **B axis narrow range (25°):** Marker only visible when roughly facing camera.
   B<65° or B>90° → marker at extreme angle or facing away.
3. **~50% of robot poses had no ArUco detection** — expected for eye-in-hand 
   setup where marker is fixed on table.

## Next Steps
- [ ] Re-capture with sub-pixel refinement enabled (already added to capture.py)
- [ ] Try larger marker (150mm/200mm) for better corner detection at steep angles
- [ ] Attempt to extend B axis range by adjusting marker placement
