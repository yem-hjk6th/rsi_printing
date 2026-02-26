# R8 Calibration Accuracy Report

**Date:** 2026-02-26  
**Source:** `R8/sync_robot_aruco.csv`  
**Changes vs R6:** ArUco marker moved to robot foot area (~1.15-1.27m distance vs R6's ~1.02-1.14m)  
**Same as R6:** HD1080, 100mm marker ID1, no sub-pixel refinement

---

## R8 vs R6 vs R5 Comparison

| Metric | R5 (HD720, 50mm) | R6 (HD1080, 100mm) | R7 (sub-pixel) | R8 (moved marker) |
|--------|-----------------|--------------------|-----------------|--------------------|
| Method | Horaud | Daniilidis | Daniilidis | Horaud |
| Poses (raw → filtered) | 20 | 31 → 23 | 31 → 28 | 26 → 22 |
| **Mean error** | 6.543 mm | **2.884 mm** | 9.099 mm | 3.056 mm |
| **Max error** | 10.251 mm | **5.136 mm** | 15.940 mm | 6.857 mm |
| LOO mean | — | 4.2 mm | 11.4 mm | 9.3 mm |
| Method consensus | — | 2.1 mm | 24.6 mm | 18.1 mm |
| Marker coord std | — | (2.7, 2.5, 2.9) | (7.4, 5.5, 7.2) | (8.5, 2.4, 9.0) |
| Quality score | — | 9/10 ★★★★☆ | 6/10 ★★★ | 8/10 ★★★★ |

## T_cam2gripper Convergence (R6 vs R8)

| Component | R6 | R8 | Δ |
|-----------|-----|-----|---|
| Tx | -306.8 mm | -306.4 mm | **0.4 mm** |
| Ty | -77.4 mm | -65.3 mm | 12.1 mm |
| Tz | 388.1 mm | 380.6 mm | 7.6 mm |
| \|ΔT\| | — | — | **14.3 mm** |

## Five-Method Comparison (26 poses)

| Method | Mean (mm) | Max (mm) | Tx (mm) | Ty (mm) | Tz (mm) |
|--------|-----------|----------|---------|---------|---------|
| Tsai | 9.5 | 56.8 | -314.1 | -43.5 | 391.4 |
| Park | 8.7 | 56.5 | -307.6 | -43.4 | 395.5 |
| **Horaud** | **8.7** | **56.5** | **-307.6** | **-43.4** | **395.5** |
| Andreff | 16.9 | 59.2 | -256.2 | -32.2 | 342.1 |
| Daniilidis | 10.5 | 53.8 | -311.3 | -59.4 | 403.3 |

Park/Horaud/Daniilidis consensus: ΔTotal = 18.1mm (vs R6's 2.1mm)

## Outlier Analysis

Pose 13 (robot 836, 15, 446) had 56.6mm error — a severe outlier likely due to marker at extreme viewing angle. After outlier removal (26→22), mean error dropped to 3.1mm.

## Conclusion

R8 with moved marker position achieved mean error 3.1mm (after outlier removal), very close to R6's 2.9mm. However:
- Method consensus is worse (18.1mm vs 2.1mm)
- LOO error is higher (9.3mm vs 4.2mm)
- Marker distance increased (~1.2m vs ~1.1m), slightly reducing corner precision
- Tx converged remarkably (0.4mm difference from R6)

**R6 remains the best calibration result.**
