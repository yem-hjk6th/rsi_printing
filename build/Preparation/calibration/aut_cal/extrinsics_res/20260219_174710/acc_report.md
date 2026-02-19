# Hand-Eye Calibration Accuracy Report

**Date:** 2026-02-19 17:47:10  
**Data:** R5/sync_robot_aruco_2.csv (ZED SDK, 19 filtered poses)
**Method:** Horaud (OpenCV calibrateHandEye)

---

## Results
- **Mean back-substitution error:** 6.5 mm
- **Max error:** 10.3 mm
- **Leave-One-Out (LOO) mean error:** 7.6 mm
- **T stability (LOO std):** (7.6, 6.2, 5.4) mm
- **Rotation diversity:** 37°
- **Park/Horaud/Daniilidis consensus:** Δtotal = 15.3 mm

## Extrinsics (T_cam2gripper)
```
[[ 0.066  0.824  0.563 -0.323]
 [ 0.998 -0.064 -0.024 -0.066]
 [ 0.016  0.564 -0.826  0.420]
 [ 0      0      0      1    ]]
```
- Translation: (-323, -66, 420) mm
- |t| = 534 mm
- Euler (KUKA A,B,C): (86.2°, -0.9°, 145.7°)

## Data Quality
- tvec_z range: 881–989 mm (ZED SDK, matches physical ~0.9m)
- No flipped detections (R[2,2]>0)
- Marker world std: (3.7, 3.7, 6.3) mm

## Code Changes
- Switched capture.py to ZED SDK (pyzed.sl), using rectified left image and SDK intrinsics
- calibrate.py: fixed iterative outlier removal (stop if error increases)
- Added verify2.py for full pipeline validation

## Recommendations
- Increase rotation diversity (>60° recommended)
- Add more poses (>20 recommended)
- Remove outlier poses if LOO error >10mm

---

**Summary:**
- Calibration is accurate and physically plausible (mean error 6.5mm, tvec_z ≈ 0.9m matches real setup)
- All previous errors were due to camera intrinsics mismatch (now fixed)
- Ready for use in downstream applications
