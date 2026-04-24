"""
extract_masks.py — SAM2 mask extraction from SVO2 files
  从指定 SVO2 文件的指定帧范围内提取 SAM2 分割掩膜。
  输出: 每帧 mask PNG + 计时 CSV, 存入 recorded_data 对应文件夹的 sam2mask/ 下。
"""

import sys, os, csv, cv2, time, json
import numpy as np
import torch
from pathlib import Path

# ZED SDK DLL paths (must be added before importing pyzed)
if os.name == "nt":
    import glob as _g
    _cuda_bin = next(iter(sorted(
        _g.glob(r"C:\Program Files\NVIDIA GPU Computing Toolkit\CUDA\v*\bin"),
        reverse=True)), "")
    for p in [
        r"C:\Program Files (x86)\ZED SDK\bin",
        r"C:\Program Files (x86)\ZED SDK\dependencies\bin",
        _cuda_bin,
    ]:
        if p and os.path.isdir(p):
            os.add_dll_directory(p)

import pyzed.sl as sl
from sam2.build_sam import build_sam2
from sam2.automatic_mask_generator import SAM2AutomaticMaskGenerator

# ═══════════════════════════════════════════════════════════════════════════════
#  USER CONFIG — 在此处设置 ROI、帧范围、以及其它参数
# ═══════════════════════════════════════════════════════════════════════════════

# ── SVO2 文件路径 ──
SVO_PATH = Path(
    r"C:\Users\888y9\Desktop\rsi_printing\recorded_data"
    r"\20260331_202433\recording_20260331_202433_001_20260331_202433.svo2"
)

# ── 帧范围 (None = 从头/到尾) ──
FRAME_START = 150          # e.g. 0, 500, None
FRAME_END   = 750          # e.g. 3000, None
FRAME_STEP  = 20             # 每隔 N 帧取一帧, 1 = 每帧都读, init = 1

# ── ROI 区域 [x, y, w, h] — None = 全图 ──
#    坐标相对于原始分辨率 (通常 1920×1080 或 1280×720)
ROI = [1200, 600, 400, 400]  # [x, y, w, h] nozzle + bead region

# ── SAM2 模型 ──
CKPT_PATH = Path(
    r"C:\Users\888y9\Desktop\rsi_printing\build\DataCollection"
    r"\cam_adjust\t2\sam2_checkpoints\sam2.1_hiera_small.pt"
)
MODEL_CFG = "configs/sam2.1/sam2.1_hiera_s.yaml"

# ── SAM2 AMG 参数 (影响计算速度和分割质量) ──
POINTS_PER_SIDE       = 16    # 网格采样密度, 越大越慢但越细致 (16/32/64), init = 32
POINTS_PER_BATCH      = 64    # GPU batch size, 越大越快但占更多 VRAM
PRED_IOU_THRESH       = 0.85   # IoU 置信度阈值, 越低保留越多 mask, init = 0.7
STABILITY_SCORE_THRESH = 0.85 # 稳定性阈值, 越低保留越多 mask
MIN_MASK_REGION_AREA  = 20    # 最小 mask 面积 (px), 过滤碎片

# ── 输出控制 ──
SAVE_OVERLAY  = True          # 是否同时保存 mask 叠加到原图的可视化
OVERLAY_ALPHA = 0.5           # 叠加透明度

# ═══════════════════════════════════════════════════════════════════════════════
#  END USER CONFIG
# ═══════════════════════════════════════════════════════════════════════════════


def build_output_dir(svo_path: Path) -> Path:
    """在 SVO 所在的 recorded_data 子目录下创建 sam2_mask_YYYYMMDD_HHMMSS/"""
    from datetime import datetime
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    out = svo_path.parent / f"sam2_mask_{ts}"
    out.mkdir(parents=True, exist_ok=True)
    return out


def open_svo(svo_path: Path):
    zed = sl.Camera()
    p = sl.InitParameters()
    p.set_from_svo_file(str(svo_path))
    p.svo_real_time_mode = False
    p.depth_mode = sl.DEPTH_MODE.NONE   # 不需要深度，加速
    status = zed.open(p)
    if status != sl.ERROR_CODE.SUCCESS:
        print(f"[ERROR] ZED open failed: {status}")
        sys.exit(1)
    return zed


def resolve_frame_range(zed, start, end, step):
    total = zed.get_svo_number_of_frames()
    s = max(0, start if start is not None else 0)
    e = min(total, end if end is not None else total)
    return list(range(s, e, step)), total


def extract_roi(bgr, roi):
    if roi is None:
        return bgr.copy(), 0, 0
    x, y, w, h = roi
    H, W = bgr.shape[:2]
    x = max(0, min(x, W - 1))
    y = max(0, min(y, H - 1))
    w = min(w, W - x)
    h = min(h, H - y)
    return bgr[y:y+h, x:x+w].copy(), x, y


def main():
    # ── Validate ──
    if not SVO_PATH.exists():
        print(f"[ERROR] SVO not found: {SVO_PATH}")
        sys.exit(1)
    if not CKPT_PATH.exists():
        print(f"[ERROR] Checkpoint not found: {CKPT_PATH}")
        sys.exit(1)

    device = "cuda" if torch.cuda.is_available() else "cpu"
    out_dir = build_output_dir(SVO_PATH)

    print(f"{'='*60}")
    print(f"  SAM2 Mask Extraction")
    print(f"{'='*60}")
    print(f"  SVO:    {SVO_PATH.name}")
    print(f"  Device: {device}")
    print(f"  ROI:    {ROI if ROI else 'full frame'}")
    print(f"  Output: {out_dir}")

    # ── Open SVO ──
    zed = open_svo(SVO_PATH)
    frame_list, total_frames = resolve_frame_range(
        zed, FRAME_START, FRAME_END, FRAME_STEP
    )
    n_frames = len(frame_list)

    res = zed.get_camera_information().camera_configuration.resolution
    print(f"  Resolution: {res.width} x {res.height}")
    print(f"  Total SVO frames: {total_frames}")
    print(f"  Frames to process: {n_frames}  "
          f"(range {frame_list[0]}..{frame_list[-1]}, step {FRAME_STEP})")
    print(f"{'='*60}\n")

    # ── Load SAM2 ──
    print("Loading SAM2 model...", end="", flush=True)
    t_load_start = time.perf_counter()
    sam2 = build_sam2(
        MODEL_CFG, str(CKPT_PATH),
        device=device, apply_postprocessing=False,
    )
    mask_gen = SAM2AutomaticMaskGenerator(
        sam2,
        points_per_side=POINTS_PER_SIDE,
        points_per_batch=POINTS_PER_BATCH,
        pred_iou_thresh=PRED_IOU_THRESH,
        stability_score_thresh=STABILITY_SCORE_THRESH,
        min_mask_region_area=MIN_MASK_REGION_AREA,
    )
    t_load = time.perf_counter() - t_load_start
    print(f" done ({t_load:.2f}s)")

    # ── Process frames ──
    img_mat = sl.Mat()
    timing_rows = []
    t_total_start = time.perf_counter()

    for i, fidx in enumerate(frame_list):
        t_frame_start = time.perf_counter()

        zed.set_svo_position(fidx)
        if zed.grab() != sl.ERROR_CODE.SUCCESS:
            print(f"  [SKIP] Frame {fidx}: grab failed")
            continue

        zed.retrieve_image(img_mat, sl.VIEW.LEFT)
        bgr = img_mat.get_data()[:, :, :3].copy()
        bgr = cv2.cvtColor(bgr, cv2.COLOR_RGB2BGR)

        # ROI crop
        roi_bgr, rx, ry = extract_roi(bgr, ROI)
        rgb = cv2.cvtColor(roi_bgr, cv2.COLOR_BGR2RGB)

        # SAM2 inference
        if device == "cuda":
            torch.cuda.synchronize()
        t_infer_start = time.perf_counter()

        with torch.inference_mode():
            masks = mask_gen.generate(rgb)

        if device == "cuda":
            torch.cuda.synchronize()
        t_infer = time.perf_counter() - t_infer_start

        # Sort masks by area descending
        masks.sort(key=lambda m: m['area'], reverse=True)

        # Save combined mask (all masks merged into single binary)
        h_roi, w_roi = rgb.shape[:2]
        combined_mask = np.zeros((h_roi, w_roi), dtype=np.uint8)
        for mi, m in enumerate(masks):
            combined_mask[m['segmentation']] = mi + 1  # instance ID

        mask_path = out_dir / f"mask_{fidx:06d}.png"
        cv2.imwrite(str(mask_path), combined_mask)

        # Save individual mask data as npz
        seg_list = [m['segmentation'].astype(np.uint8) for m in masks]
        areas = [m['area'] for m in masks]
        ious = [m['predicted_iou'] for m in masks]
        stabilities = [m['stability_score'] for m in masks]

        npz_path = out_dir / f"masks_{fidx:06d}.npz"
        np.savez_compressed(
            str(npz_path),
            masks=np.stack(seg_list) if seg_list else np.zeros((0, h_roi, w_roi), dtype=np.uint8),
            areas=np.array(areas),
            ious=np.array(ious),
            stabilities=np.array(stabilities),
            roi_origin=np.array([rx, ry]),
        )

        # Optional overlay visualization
        if SAVE_OVERLAY and masks:
            overlay = roi_bgr.copy()
            rng = np.random.RandomState(42)
            for m in masks:
                color = rng.randint(0, 255, 3).tolist()
                overlay[m['segmentation']] = color
            blend = cv2.addWeighted(roi_bgr, 1 - OVERLAY_ALPHA, overlay, OVERLAY_ALPHA, 0)
            cv2.imwrite(
                str(out_dir / f"overlay_{fidx:06d}.jpg"),
                blend, [cv2.IMWRITE_JPEG_QUALITY, 90],
            )

        t_frame = time.perf_counter() - t_frame_start
        timing_rows.append({
            'frame': fidx,
            'n_masks': len(masks),
            'infer_sec': round(t_infer, 4),
            'total_sec': round(t_frame, 4),
        })

        print(f"  [{i+1:4d}/{n_frames}]  frame {fidx:6d}  "
              f"masks={len(masks):3d}  "
              f"infer={t_infer:.3f}s  total={t_frame:.3f}s")

    t_total = time.perf_counter() - t_total_start
    zed.close()

    # ── Save timing CSV ──
    csv_path = out_dir / "timing.csv"
    with open(csv_path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=["frame", "n_masks", "infer_sec", "total_sec"])
        w.writeheader()
        w.writerows(timing_rows)

    # ── Save run config ──
    config_path = out_dir / "run_config.json"
    config = {
        "svo_path": str(SVO_PATH),
        "frame_start": FRAME_START,
        "frame_end": FRAME_END,
        "frame_step": FRAME_STEP,
        "roi": ROI,
        "points_per_side": POINTS_PER_SIDE,
        "points_per_batch": POINTS_PER_BATCH,
        "pred_iou_thresh": PRED_IOU_THRESH,
        "stability_score_thresh": STABILITY_SCORE_THRESH,
        "min_mask_region_area": MIN_MASK_REGION_AREA,
        "device": device,
        "model_cfg": MODEL_CFG,
        "total_svo_frames": total_frames,
        "frames_processed": len(timing_rows),
    }
    with open(config_path, "w") as f:
        json.dump(config, f, indent=2, ensure_ascii=False)

    # ── Summary ──
    print(f"\n{'='*60}")
    print(f"  SUMMARY")
    print(f"{'='*60}")
    print(f"  Total frames processed: {len(timing_rows)}")
    print(f"  Total wall time:        {t_total:.2f}s")
    if timing_rows:
        infer_times = [r['infer_sec'] for r in timing_rows]
        print(f"  Avg infer per frame:    {np.mean(infer_times):.3f}s")
        print(f"  Min / Max infer:        {min(infer_times):.3f}s / {max(infer_times):.3f}s")
    print(f"  Model load time:        {t_load:.2f}s")
    if device == "cuda":
        vram = torch.cuda.max_memory_allocated() / 1024**2
        print(f"  VRAM peak:              {vram:.0f} MB")
    print(f"  Output dir:             {out_dir}")
    print(f"{'='*60}")


if __name__ == "__main__":
    main()
