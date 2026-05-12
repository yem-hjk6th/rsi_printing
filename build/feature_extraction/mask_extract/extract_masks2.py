"""
extract_masks2.py — SAM2 overlay for specific frame list → 10×3 grid
  与 make_grids.py 使用相同帧号, 生成对齐的 SAM2 overlay 网格.
  环境: conda activate zedenv
"""

import sys, os, time
import cv2
import numpy as np
import torch
from pathlib import Path

# ── ZED SDK DLL ──
if os.name == "nt":
    for p in [
        r"C:\Program Files (x86)\ZED SDK\bin",
        r"C:\Program Files (x86)\ZED SDK\dependencies\bin",
        r"C:\Program Files\NVIDIA GPU Computing Toolkit\CUDA\v12.8\bin",
    ]:
        if os.path.isdir(p):
            os.add_dll_directory(p)

import pyzed.sl as sl
from sam2.build_sam import build_sam2
from sam2.automatic_mask_generator import SAM2AutomaticMaskGenerator

# ═══════════════════════════════════════════════════════════════════════════════
#  CONFIG
# ═══════════════════════════════════════════════════════════════════════════════

SVO_PATH = Path(
    r"C:\Users\888y9\Desktop\rsi_printing\recorded_data"
    r"\20260331_202433\recording_20260331_202433_001_20260331_202433.svo2"
)
OUT_DIR = Path(
    r"C:\Users\888y9\Desktop\rsi_printing\recorded_data\20260331_202433"
)

ROI = [1200, 600, 400, 400]

# 与 make_grids.py 完全一致的 30 帧
FRAME_LIST = [
    150, 450, 850, 1250, 1650, 2050, 2450, 2850, 3250, 3650,
    4050, 4450, 4850, 5250, 5650, 6050, 6450, 6850, 7250, 7650,
    8050, 8450, 8850, 9250, 9650, 10050, 10450, 10850, 11250, 11650,
]

# SAM2
CKPT_PATH = Path(
    r"C:\Users\888y9\Desktop\rsi_printing\build\DataCollection"
    r"\cam_adjust\t2\sam2_checkpoints\sam2.1_hiera_small.pt"
)
MODEL_CFG = "configs/sam2.1/sam2.1_hiera_s.yaml"

POINTS_PER_SIDE        = 16
POINTS_PER_BATCH       = 64
PRED_IOU_THRESH        = 0.85
STABILITY_SCORE_THRESH = 0.85
MIN_MASK_REGION_AREA   = 20
OVERLAY_ALPHA          = 0.5

COLS, ROWS = 10, 3


def roi_crop(bgr, roi):
    x, y, w, h = roi
    return bgr[y:y + h, x:x + w].copy()


def make_grid(images, cols, rows):
    h, w = images[0].shape[:2]
    canvas = np.zeros((rows * h, cols * w, 3), dtype=np.uint8)
    for i, img in enumerate(images):
        if i >= cols * rows:
            break
        r, c = divmod(i, cols)
        canvas[r * h:(r + 1) * h, c * w:(c + 1) * w] = img
    return canvas


def main():
    device = "cuda" if torch.cuda.is_available() else "cpu"

    # ── Open SVO ──
    zed = sl.Camera()
    ip = sl.InitParameters()
    ip.set_from_svo_file(str(SVO_PATH))
    ip.svo_real_time_mode = False
    ip.depth_mode = sl.DEPTH_MODE.NONE
    if zed.open(ip) != sl.ERROR_CODE.SUCCESS:
        print("[ERROR] SVO open failed"); sys.exit(1)

    # ── Load SAM2 ──
    print("Loading SAM2...", end="", flush=True)
    t0 = time.perf_counter()
    sam2 = build_sam2(MODEL_CFG, str(CKPT_PATH), device=device,
                      apply_postprocessing=False)
    mask_gen = SAM2AutomaticMaskGenerator(
        sam2,
        points_per_side=POINTS_PER_SIDE,
        points_per_batch=POINTS_PER_BATCH,
        pred_iou_thresh=PRED_IOU_THRESH,
        stability_score_thresh=STABILITY_SCORE_THRESH,
        min_mask_region_area=MIN_MASK_REGION_AREA,
    )
    print(f" {time.perf_counter() - t0:.1f}s")

    img_mat = sl.Mat()
    overlays = []
    t_total = time.perf_counter()

    for i, fidx in enumerate(FRAME_LIST):
        t0 = time.perf_counter()

        zed.set_svo_position(fidx)
        if zed.grab() != sl.ERROR_CODE.SUCCESS:
            ph = np.zeros((ROI[3], ROI[2], 3), dtype=np.uint8)
            cv2.putText(ph, f"F{fidx} SKIP", (5, 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
            overlays.append(ph)
            print(f"  [{i+1:2d}/{len(FRAME_LIST)}] F{fidx} SKIP")
            continue

        zed.retrieve_image(img_mat, sl.VIEW.LEFT)
        bgr = img_mat.get_data()[:, :, :3].copy()
        bgr = cv2.cvtColor(bgr, cv2.COLOR_RGB2BGR)
        roi_bgr = roi_crop(bgr, ROI)
        rgb = cv2.cvtColor(roi_bgr, cv2.COLOR_BGR2RGB)

        # SAM2 inference
        with torch.inference_mode():
            masks = mask_gen.generate(rgb)
        masks.sort(key=lambda m: m['area'], reverse=True)

        # Overlay
        overlay = roi_bgr.copy()
        rng = np.random.RandomState(42)
        for m in masks:
            color = rng.randint(0, 255, 3).tolist()
            overlay[m['segmentation']] = color
        blend = cv2.addWeighted(roi_bgr, 1 - OVERLAY_ALPHA,
                                overlay, OVERLAY_ALPHA, 0)
        cv2.putText(blend, f"F{fidx}", (5, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        overlays.append(blend)

        dt = time.perf_counter() - t0
        print(f"  [{i+1:2d}/{len(FRAME_LIST)}] F{fidx}  "
              f"masks={len(masks)}  {dt:.2f}s")

    zed.close()

    # ── Grid ──
    grid = make_grid(overlays, COLS, ROWS)
    out_path = OUT_DIR / "grid_sam2_overlay_10x3.jpg"
    cv2.imwrite(str(out_path), grid, [cv2.IMWRITE_JPEG_QUALITY, 92])

    print(f"\nGrid: {grid.shape[1]}x{grid.shape[0]} -> {out_path.name}")
    print(f"Done in {time.perf_counter() - t_total:.1f}s")


if __name__ == "__main__":
    main()
