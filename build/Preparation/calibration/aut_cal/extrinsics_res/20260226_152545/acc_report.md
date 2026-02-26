# R7 Calibration Accuracy Report

**Date:** 2026-02-26  
**Source:** `R7/sync_robot_aruco.csv`  
**Changes vs R6:** Sub-pixel corner refinement (`CORNER_REFINE_SUBPIX`, winSize=5, maxIter=50, minAcc=0.01)  
**Resolution / Marker:** HD1080, 100mm marker ID1 (same as R6)

---

## R7 vs R6 Comparison

| Metric | R6 (no sub-pixel) | R7 (sub-pixel) | Change |
|--------|-------------------|----------------|--------|
| Method | Daniilidis | Daniilidis | — |
| Poses (raw → filtered) | 31 → 23 | 31 → 28 | — |
| **Mean error** | **2.884 mm** | **9.099 mm** | **+215% ↑ worse** |
| **Max error** | **5.136 mm** | **15.940 mm** | **+210% ↑ worse** |
| LOO mean | 4.2 mm | 11.4 mm | +171% worse |
| Method consensus | 2.1 mm | 24.6 mm | +1071% worse |
| Marker coord std | (2.7, 2.5, 2.9) mm | (7.4, 5.5, 7.2) mm | ~2.5× worse |
| Quality score | 9/10 ★★★★☆ | 6/10 ★★★ | -3 |

## Five-Method Comparison (31 poses)

| Method | Mean (mm) | Max (mm) | Tx (mm) | Ty (mm) | Tz (mm) |
|--------|-----------|----------|---------|---------|---------|
| Tsai | 10.6 | 24.7 | -288.1 | -72.9 | 415.0 |
| Park | 10.7 | 24.4 | -281.8 | -69.4 | 421.3 |
| Horaud | 10.7 | 24.4 | -281.7 | -69.4 | 421.3 |
| Andreff | 15.3 | 28.1 | -235.2 | -54.8 | 364.8 |
| **Daniilidis** | **10.3** | **18.2** | **-282.3** | **-60.2** | **398.6** |

Park/Horaud/Daniilidis consensus: ΔTotal = 24.6mm (vs R6's 2.1mm)

## Conclusion

Sub-pixel corner refinement **degraded** calibration accuracy by ~3× on printed ArUco markers at ~1m distance. The printed marker's edge quality is insufficient for sub-pixel gradient fitting, causing systematic corner displacement.

**R6 result remains the best calibration.**
