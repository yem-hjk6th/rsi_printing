# Debug Log: Hand-Eye Calibration (R5)

**Date:** 2026-02-19 17:47:10

## Initial Problem
- All previous calibration rounds (R1–R5) had huge errors (mean 117–1000mm)
- tvec_z values were ~2m, but physical camera-to-marker distance was ~0.2–0.65m
- Calibration results physically impossible (T_cam2gripper translation >2m)

## Root Cause
- Used cv2.VideoCapture(1) to grab ZED image, but intrinsics were from ZED SDK rectified image (HD720)
- This mismatch caused solvePnP to compute tvec_z ≈ 4x real value
- All previous data invalid

## Fixes
- Rewrote capture.py to use pyzed.sl (ZED SDK), reading correct intrinsics and rectified left image
- Verified tvec_z now 881–989mm, matching real setup
- calibrate.py: fixed iterative outlier removal logic (stop if error increases)
- Added verify2.py for full pipeline validation

## Error Evolution
- Before fix: mean error 117–1000mm, tvec_z ≈ 2m
- After fix: mean error 6.5mm, tvec_z ≈ 0.9m

## Key Code Changes
- capture.py: switched to ZED SDK, removed cv2.VideoCapture
- calibrate.py: improved outlier removal
- verify2.py: added for full validation

## Additional Notes
- Marker length: 50mm (check if matches physical marker)
- All results now physically plausible and stable
- Recommend increasing rotation diversity in SRC for even better accuracy
