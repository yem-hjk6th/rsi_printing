# Revision Log

## 2026-04-23 — Desktop Environment Setup + CUDA Auto-Detect Refactor

### Context
Migrating workflow from Win Laptop (RTX 5070, CUDA 12.8) to Desktop (RTX 5080, CUDA 13.0).
All CUDA DLL path handling converted from hardcoded `v12.8` to runtime auto-detection via `glob`.

### New Files
- `Vision/zed_setup.py` — ZED SDK DLL setup helper; uses `_find_cuda_dir()` with `glob` to auto-pick highest installed CUDA version
- `Vision/zed_stereo_calc.py` — stereo geometry utilities
- `Vision/cam_port.py` — camera port enumeration
- `Vision/svo_extract/` — SVO file processing scripts (moved from root `svo_extract/`)
- `Vision/zed_prin_exp_demo/` — ZED + printing experiment demo scripts
- `Vision/micro_scope_2/`, `Vision/Micro_scope_exp_demo/` — microscope capture experiments
- `build/feature_extraction/` — full mask extraction pipeline (SAM-2, depth, layer separation)
  - `extract_masks.py`, `extract_masks2.py`, `depth1.py`, `depth2_ffs.py`, `depth3_disp_layers.py`
  - `make_grids.py`, `layer_sep_masks.py`
- `build/DataCollection/labeling/post_labeling/` — post-labeling analysis tools
- `build/Roboter/src/Ye_RSI_t6_fixed.src`, `Ye_micro_t1_fixed.src` — new KUKA trajectories
- `build/calib/calibration_summary_20260331.md` — calibration results summary
- `doc/` — new documentation folder (cal_flow/, roboter_ori/ subfolders)
- `log/` — new log folder (this file, zed_dll_fix.md, SEARCH_RESULTS_SUMMARY.md, printbed_setup.md)
- `rsi_setup/` — reorganized from root-level `RSI_logistics/`, `RSI_mindset/`, `RSI_play/`, `RSI_set_ver/`, `RSI_visual/`
- `.github/copilot-instructions.md` — agent auto-read: machine config, CUDA policy, env names

### Modified Files (CUDA auto-detect refactor — 13 files)
- `Vision/zed_setup.py` — `_find_cuda_dir()` replaces hardcoded `CUDA_DIR = v12.8`
- `build/DataCollection/cam_adjust/zed_res_benchmark.py` — glob-based CUDA bin detection
- `build/DataCollection/cam_adjust/zed_res_bench_quick.py` — glob-based CUDA bin detection
- `build/Preparation/test/init_recorder_svo_csv.py` — glob-based CUDA bin detection
- `build/Preparation/test/ori_recorder_svo_csv.py` — glob-based CUDA bin detection
- `build/calib/aut_cal_capture.py` — glob-based CUDA bin detection
- `build/feature_extraction/mask_extract/*.py` (7 files) — glob-based CUDA bin detection

### Reorganized (old path → new path)
- `REVISIONS.md` → `log/REVISIONS.md`
- `SEARCH_RESULTS_SUMMARY.md` → `log/SEARCH_RESULTS_SUMMARY.md`
- `svo_extract/` → `Vision/svo_extract/`
- `roboter_ori/` → `doc/roboter_ori/`
- `cal/MDPH2_Line_Width_Analysis_20260326.md` → `doc/cal_flow/`
- `RSI_logistics/`, `RSI_mindset/`, `RSI_play/`, `RSI_set_ver/`, `RSI_visual/` → `rsi_setup/`
- `build/Roboter/src/src_header_replace.py` → `src/src_header_replace.py`

### Environments Configured (Desktop)
- `zedenv` (Python 3.10): pyzed 5.2, torch nightly cu128, SAM-2, tensorflow, open3d, pycolmap
- `ffs` (Python 3.12): Fast-FoundationStereo, pybullet (conda-forge), tensorflow, open3d, timm

---

## 2026-04-16 — ZED SDK DLL Fix

See `log/zed_dll_fix.md` for full details.

- Created `Vision/zed_setup.py` as single-import DLL path solution
- ZED SDK 5.2.2 → upgraded to 5.2.3 (CUDA 13 build) on desktop

---

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
