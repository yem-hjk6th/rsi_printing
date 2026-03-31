# Revision Log

## 2026-03-31 — Calibration Toolchain + Recorder Upgrade

### New Files
- `build/calib/aut_cal_capture.py` — RSI + ArUco synchronized auto-capture for hand-eye calibration
- `build/calib/extrinsic_extraction.py` — hand-eye solver (Daniilidis/Park/Horaud) with timestamped output
- `build/calib/verify_extrinsic.py` — 7-section verification report generator
- `build/calib/aut_cal4.src` — 35-pose KUKA trajectory for new bed position (760, -410, -12.5)
- `build/calib/res/` — calibration results (6 runs, best mean=1.566mm)
- `build/Roboter/src/Ye_RSI_t4.src` / `Ye_RSI_t4_fixed.src` / `Ye_RSI_t5_fixed.src` — new print trajectories
- `build/DataCollection/cam_adjust/compare_depth_frangi.py` — depth + Frangi vessel filter comparison
- `build/DataCollection/cam_adjust/run_sam2_t3.py` — SAM2 bead segmentation on t3 recording
- `rsi_data/layer_detect.py` — RSI layer boundary detection from CSV
- `svo_extract/depth_layer_detect.py` — depth-based layer detection from SVO

### Modified Files
- `build/Preparation/test/init_recorder_svo_csv.py` — added `session_meta.json`, `K.txt`, `svo_frame_idx`, `zed_timestamp_ns` columns; HD2K 15fps; stereo preview; median averaging; DLL fix
- `build/Roboter/src/src_header_replace.py` — minor updates
