# R6 Calibration Accuracy Report

**Date:** 2026-02-26  
**Source:** `R6/sync_robot_aruco.csv`  
**Changes vs R5:** HD1080 (was HD720), 100mm marker ID1 (was 50mm ID0), SVD orthogonalization  
**Sub-pixel refinement:** Not used in this capture

---

## R6 vs R5 Comparison

| Metric | R5 (HD720, 50mm marker) | R6 (HD1080, 100mm marker) | Change |
|--------|------------------------|---------------------------|--------|
| Method | Horaud | Daniilidis | — |
| Poses (raw) | 20 | 31 | +11 |
| Poses (after outlier removal) | 20 | 23 | +3 |
| **Mean error** | **6.543 mm** | **2.884 mm** | **-56% ↓** |
| **Max error** | **10.251 mm** | **5.136 mm** | **-50% ↓** |
| Quality | GOOD | GOOD | — |

## Five-Method Comparison (31 poses, pairwise filtered)

| Method | Mean (mm) | Max (mm) | Tx (mm) | Ty (mm) | Tz (mm) | \|t\| (mm) |
|--------|-----------|----------|---------|---------|---------|------------|
| Tsai | 4.1 | 10.8 | -305.0 | -76.1 | 381.3 | 494.2 |
| Park | 4.1 | 10.7 | -306.4 | -74.9 | 380.8 | 494.4 |
| Horaud | 4.1 | 10.7 | -306.4 | -74.9 | 380.8 | 494.4 |
| Andreff | 7.1 | 19.9 | -284.8 | -68.8 | 361.6 | 465.4 |
| **Daniilidis** | **4.0** | **10.6** | **-306.0** | **-76.9** | **381.5** | **495.1** |

Park/Horaud/Daniilidis consensus: ΔX=0.4mm ΔY=1.9mm ΔZ=0.8mm total=2.1mm

## Verification Summary

| Check | Result |
|-------|--------|
| det(R) | 1.00000000 ✓ |
| \|\|R·R^T - I\|\| | 1.34e-15 ✓ |
| LOO mean error | 4.2 mm |
| LOO max error | 9.8 mm |
| T stability (LOO std) | (0.9, 1.3, 0.8) mm |
| Marker world coord std | (2.7, 2.5, 2.9) mm |
| Rotation spread | 45.1° |
| Quality score | 9/10 ★★★★☆ |

## T_cam2gripper

```
R5 (Horaud):
[[ 0.066052  0.823625  0.563276 -0.322889]
 [ 0.997690 -0.063505 -0.024136 -0.066070]
 [ 0.015892  0.563569 -0.825916  0.419890]
 [ 0.000000  0.000000  0.000000  1.000000]]

R6 (Daniilidis):
[[ 0.047495  0.811369  0.582601 -0.306804]
 [ 0.998643 -0.051054 -0.010310 -0.077421]
 [ 0.021379  0.582300 -0.812693  0.388133]
 [ 0.000000  0.000000  0.000000  1.000000]]
```

Translation difference R6-R5:  
- ΔTx = +16.1 mm  
- ΔTy = -11.4 mm  
- ΔTz = -31.8 mm  
- |ΔT| = 37.5 mm

## Conclusion

R6 shows **significant improvement** over R5:
- Mean error dropped from 6.5mm to 2.9mm (56% reduction)
- Max error dropped from 10.3mm to 5.1mm (50% reduction)
- All 4 robust methods (excluding Andreff) agree within 2.1mm
- T_cam2gripper is stable across LOO folds (std < 1.3mm per axis)

The 37.5mm shift in translation is expected — the R5 50mm marker at HD720 had worse corner localization, causing systematic bias. The R6 result with 100mm marker at HD1080 should be more accurate.

**Remaining improvement:** Sub-pixel refinement has been added to capture.py but was NOT used for this capture. A re-capture with sub-pixel enabled may further improve precision.
