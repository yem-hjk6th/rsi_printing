"""
sam2_bead_test.py — SAM 2.1 自动分割 bead 检测 + 宽度测量

两种模式:
  1. Automatic Mask Generator — 全图自动分割所有物体
  2. Point Prompt — 手动指定 bead 上的点引导分割

对比 Meijering vesselness 滤波器的效果
"""

import sys, os, cv2, numpy as np
import pyzed.sl as sl
from pathlib import Path
import torch

from sam2.build_sam import build_sam2
from sam2.automatic_mask_generator import SAM2AutomaticMaskGenerator
from sam2.sam2_image_predictor import SAM2ImagePredictor

# ─── Config ──────────────────────────────────────────
SVO_PATH   = r"C:\Users\dell\Desktop\RSI\recorded_data\20260310_183338\recording_20260310_183338.svo2"
OUT_DIR    = Path(__file__).parent / "sam2_output"
CKPT_PATH  = Path(__file__).parent / "sam2_checkpoints" / "sam2.1_hiera_small.pt"
MODEL_CFG  = "configs/sam2.1/sam2.1_hiera_s.yaml"
TEST_FRAMES = [1200, 2000, 2800, 3600]

# ROI — 跟 vesselness 测试一致
USE_ROI  = True
ROI_Y    = (300, 950)
ROI_X    = (100, 1800)

# Auto mask generator 参数 — 调小以检测细结构
AMG_PARAMS = dict(
    points_per_side=32,           # 点网格密度 (32 for 6GB VRAM)
    points_per_batch=64,
    pred_iou_thresh=0.7,          # IoU 阈值 (降低以保留更多候选)
    stability_score_thresh=0.85,  # 稳定性阈值
    min_mask_region_area=50,      # 最小 mask 面积
)

# bead 过滤参数
MIN_ASPECT  = 3.0     # 最小纵横比
MIN_AREA    = 100     # 最小面积
MAX_AREA    = 50000   # 最大面积 (排除大块背景)
MAX_SOLIDITY = 0.8    # 最大 solidity (bead 是细长结构, solidity 低)

# 宽度测量
WIDTH_MIN = 3
WIDTH_MAX = 40
N_SCANLINES = 30


# ─── Helpers ─────────────────────────────────────────

def open_svo(path):
    zed = sl.Camera()
    p   = sl.InitParameters()
    p.set_from_svo_file(path)
    p.svo_real_time_mode = False
    p.depth_mode = sl.DEPTH_MODE.NONE
    if zed.open(p) != sl.ERROR_CODE.SUCCESS:
        print("ZED open failed"); sys.exit(1)
    return zed


def grab_frame(zed, idx):
    zed.set_svo_position(idx)
    mat = sl.Mat()
    if zed.grab() != sl.ERROR_CODE.SUCCESS:
        return None
    zed.retrieve_image(mat, sl.VIEW.LEFT)
    f = mat.get_data()[:, :, :3].copy()
    return cv2.cvtColor(f, cv2.COLOR_RGB2BGR)


def get_blue_mask(bgr):
    """蓝色底板区域"""
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    blue = cv2.inRange(hsv, np.array([90, 40, 30]), np.array([140, 255, 255]))
    k = cv2.getStructuringElement(cv2.MORPH_RECT, (15, 15))
    blue = cv2.dilate(blue, k, iterations=5)
    blue = cv2.morphologyEx(blue, cv2.MORPH_CLOSE, k, iterations=3)
    return blue


def filter_bead_masks(masks_data, blue_mask=None):
    """
    从 SAM 输出的所有 mask 中筛选 bead-like 的
    标准: 高纵横比 + 面积适中 + 在蓝色区域内
    """
    bead_candidates = []

    for m in masks_data:
        mask = m['segmentation'].astype(np.uint8)
        area = m['area']

        if area < MIN_AREA or area > MAX_AREA:
            continue

        # 蓝色区域内占比
        if blue_mask is not None:
            overlap = np.sum(mask & (blue_mask > 0))
            if overlap / (area + 1e-6) < 0.3:  # 至少 30% 在蓝色区域内
                continue

        # 连通域分析: 纵横比
        contours, _ = cv2.findContours(mask * 255, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            continue

        cnt = max(contours, key=cv2.contourArea)
        if len(cnt) < 5:
            continue

        # 旋转矩形
        rect = cv2.minAreaRect(cnt)
        (cx, cy), (rw, rh), angle = rect
        aspect = max(rw, rh) / (min(rw, rh) + 1e-6)

        if aspect < MIN_ASPECT:
            continue

        # Solidity (凸包面积比)
        hull = cv2.convexHull(cnt)
        hull_area = cv2.contourArea(hull)
        solidity = area / (hull_area + 1e-6)

        bead_candidates.append({
            'mask': mask,
            'area': area,
            'aspect': aspect,
            'solidity': solidity,
            'bbox': m['bbox'],
            'iou': m['predicted_iou'],
            'stability': m['stability_score'],
            'rect': rect,
        })

    # 按纵横比排序 (越高越 bead-like)
    bead_candidates.sort(key=lambda x: x['aspect'], reverse=True)
    return bead_candidates


def measure_width_from_mask(mask_uint8, n_samples=N_SCANLINES):
    """沿 mask 的 X 方向取垂直切面测宽"""
    ys, xs = np.where(mask_uint8 > 0)
    if len(xs) < 20:
        return []

    x_min, x_max = xs.min(), xs.max()
    if x_max - x_min < 15:
        return []

    sample_xs = np.linspace(x_min + 5, x_max - 5, n_samples).astype(int)
    widths = []
    for sx in sample_xs:
        col = mask_uint8[:, sx]
        d = np.diff(col.astype(np.int16))
        starts = np.where(d > 0)[0] + 1
        ends   = np.where(d < 0)[0] + 1
        if len(starts) == 0 and col[0] > 0:
            starts = np.array([0])
        if len(ends) == 0 and col[-1] > 0:
            ends = np.array([len(col)])
        for s, e in zip(starts, ends):
            w = e - s
            if WIDTH_MIN <= w <= WIDTH_MAX:
                widths.append(w)
    return widths


def visualize_all_masks(bgr, masks_data, frame_idx, out_dir, label="all"):
    """可视化所有 SAM 检测到的 mask (随机颜色)"""
    overlay = bgr.copy()
    for i, m in enumerate(masks_data):
        mask = m['segmentation']
        color = np.random.randint(50, 255, 3).tolist()
        overlay[mask > 0] = color
    blend = cv2.addWeighted(bgr, 0.5, overlay, 0.5, 0)
    cv2.putText(blend, f"SAM2: {len(masks_data)} masks", (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
    cv2.imwrite(str(out_dir / f"f{frame_idx}_{label}_masks.png"), blend)
    return blend


def visualize_bead(bgr, bead_candidates, widths_all, frame_idx, out_dir):
    """可视化筛选后的 bead mask + 宽度"""
    overlay = bgr.copy()
    colors = [(0, 255, 0), (255, 0, 0), (0, 255, 255), (255, 0, 255), (255, 255, 0)]

    for i, cand in enumerate(bead_candidates[:5]):
        color = colors[i % len(colors)]
        overlay[cand['mask'] > 0] = color

        # 标注纵横比
        bx, by, bw, bh = cand['bbox']
        cv2.putText(overlay, f"A={cand['aspect']:.1f} S={cand['solidity']:.2f}",
                    (int(bx), int(by) - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)

    blend = cv2.addWeighted(bgr, 0.5, overlay, 0.5, 0)

    if widths_all:
        mean_w = np.mean(widths_all)
        std_w  = np.std(widths_all)
        cv2.putText(blend, f"Bead Width: {mean_w:.1f} +/- {std_w:.1f} px  (n={len(widths_all)})",
                    (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
    cv2.putText(blend, f"Bead candidates: {len(bead_candidates)}",
                (20, 75), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)

    cv2.imwrite(str(out_dir / f"f{frame_idx}_bead_overlay.png"), blend)

    # 也保存 bead mask (合并)
    combined = np.zeros(bgr.shape[:2], dtype=np.uint8)
    for cand in bead_candidates[:5]:
        combined[cand['mask'] > 0] = 255
    cv2.imwrite(str(out_dir / f"f{frame_idx}_bead_mask.png"), combined)


# ─── Main ────────────────────────────────────────────

def main():
    OUT_DIR.mkdir(parents=True, exist_ok=True)

    device = "cuda" if torch.cuda.is_available() else "cpu"
    print(f"=== SAM 2.1 Bead Detection ===")
    print(f"Device: {device} ({torch.cuda.get_device_name(0) if device == 'cuda' else 'CPU'})")
    print(f"Model: {CKPT_PATH.name}")
    print(f"Frames: {TEST_FRAMES}\n")

    # 加载模型
    print("Loading SAM 2.1 model...", end="", flush=True)
    sam2 = build_sam2(
        MODEL_CFG,
        str(CKPT_PATH),
        device=device,
        apply_postprocessing=False,
    )
    mask_generator = SAM2AutomaticMaskGenerator(sam2, **AMG_PARAMS)
    print(" Done\n")

    # 打开 SVO
    zed = open_svo(SVO_PATH)
    all_widths = []

    for fi in TEST_FRAMES:
        bgr = grab_frame(zed, fi)
        if bgr is None:
            print(f"  Frame {fi}: grab failed"); continue

        blue_mask = get_blue_mask(bgr)

        # ROI 裁切
        if USE_ROI:
            y1, y2 = ROI_Y; x1, x2 = ROI_X
            bgr_roi  = bgr[y1:y2, x1:x2]
            blue_roi = blue_mask[y1:y2, x1:x2]
        else:
            bgr_roi, blue_roi = bgr, blue_mask

        # SAM 需要 RGB 输入
        rgb_roi = cv2.cvtColor(bgr_roi, cv2.COLOR_BGR2RGB)

        print(f"  Frame {fi}: SAM2 generating masks...", end="", flush=True)
        torch.cuda.empty_cache()
        try:
            with torch.inference_mode():
                masks = mask_generator.generate(rgb_roi)
        except RuntimeError as e:
            if "out of memory" in str(e).lower():
                print(f" CUDA OOM! Skipping.")
                torch.cuda.empty_cache()
                continue
            raise
        print(f" Got {len(masks)} masks.", end="", flush=True)

        # 可视化全部 mask
        visualize_all_masks(bgr_roi, masks, fi, OUT_DIR)

        # 筛选 bead-like mask
        bead_cands = filter_bead_masks(masks, blue_roi)
        print(f" {len(bead_cands)} bead candidates.", end="", flush=True)

        # 宽度测量
        frame_widths = []
        for cand in bead_cands[:5]:  # 取前5
            w = measure_width_from_mask(cand['mask'] * 255, N_SCANLINES)
            frame_widths.extend(w)

        visualize_bead(bgr_roi, bead_cands, frame_widths, fi, OUT_DIR)

        if frame_widths:
            all_widths.extend(frame_widths)
            mean_w = np.mean(frame_widths)
            std_w  = np.std(frame_widths)
            print(f" Done")
            print(f"    Width: {mean_w:.1f} ± {std_w:.1f} px  (n={len(frame_widths)})")
            for j, cand in enumerate(bead_cands[:5]):
                print(f"      #{j}: area={cand['area']}, aspect={cand['aspect']:.1f}, "
                      f"solidity={cand['solidity']:.2f}, IoU={cand['iou']:.2f}")
        else:
            print(f" No bead-like masks found")
        print()

    zed.close()

    # ─── 全局统计 ───
    print("=" * 55)
    if all_widths:
        arr = np.array(all_widths)
        print(f"SAM2 Global width stats (n={len(arr)}):")
        print(f"  Mean:   {arr.mean():.1f} px")
        print(f"  Median: {np.median(arr):.1f} px")
        print(f"  Std:    {arr.std():.1f} px")
        print(f"  CV:     {arr.std()/arr.mean()*100:.1f}%")
        print(f"  Range:  [{arr.min()}, {arr.max()}] px")
        print(f"  IQR:    [{np.percentile(arr,25):.0f}, {np.percentile(arr,75):.0f}] px")
    else:
        print("No valid bead width measurements.")

    # ─── 对比 Meijering ───
    print()
    print("=" * 55)
    print("COMPARISON vs Meijering vesselness:")
    print("  Meijering: Median=6.0px, IQR=[5,7], CV=80.3%, n=177")
    if all_widths:
        arr = np.array(all_widths)
        print(f"  SAM 2.1:  Median={np.median(arr):.1f}px, "
              f"IQR=[{np.percentile(arr,25):.0f},{np.percentile(arr,75):.0f}], "
              f"CV={arr.std()/arr.mean()*100:.1f}%, n={len(arr)}")
    else:
        print("  SAM 2.1:  No detections")

    print(f"\nOutput: {OUT_DIR}")


if __name__ == "__main__":
    main()
