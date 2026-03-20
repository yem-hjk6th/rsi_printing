"""
sam2_bead_224.py — 224×224 ROI + SAM2 bead width 测量 (v2)

流程:
  1. SVO2 取一帧 (RGB + Depth)
  2. 检测 nozzle 位置 (蓝色 mount HSV)
  3. ROI 224×224: 以 nozzle 为锚, 向 bead 侧偏移
  4. SAM2 分割 → 排除 nozzle 筒身 → 留细长 bead
  5. 测 bead width (px) → 用深度+内参换算 mm
"""

import sys, cv2, numpy as np
import pyzed.sl as sl
from pathlib import Path
import torch

from sam2.build_sam import build_sam2
from sam2.automatic_mask_generator import SAM2AutomaticMaskGenerator

# ─── Config ──────────────────────────────────────────
SVO_PATH  = r"C:\Users\dell\Desktop\RSI\recorded_data\20260310_183338\recording_20260310_183338.svo2"
OUT_DIR   = Path(__file__).parent / "sam2_224_output"
CKPT_PATH = Path(__file__).parent / "sam2_checkpoints" / "sam2.1_hiera_small.pt"
MODEL_CFG = "configs/sam2.1/sam2.1_hiera_s.yaml"

FRAME_IDX = 2800
ROI_SIZE  = 224

# ROI 偏移: 从 nozzle 中心向 bead 方向平移
# bead 在 nozzle 旁边 (X 方向偏移), 可根据实际调
ROI_DX = -30      # 略偏左, bead 紧邻 nozzle
ROI_DY = 350      # 从 mount 底部往下 ~350px 到 bead 区域

# SAM2
AMG_PARAMS = dict(
    points_per_side=32,
    points_per_batch=64,
    pred_iou_thresh=0.7,
    stability_score_thresh=0.85,
    min_mask_region_area=20,
)

# bead 筛选 — 排除 nozzle (圆形+大面积)
BEAD_MIN_ASPECT = 2.5     # bead 是细长的, 纵横比 >= 2.5
BEAD_MIN_AREA   = 30
BEAD_MAX_AREA   = 8000    # 排除 nozzle 筒身 (大面积)

# 宽度测量
WIDTH_MIN   = 2
WIDTH_MAX   = 40
N_SCANLINES = 20


# ─── Core functions ──────────────────────────────────

def find_nozzle(bgr):
    """检测蓝色 mount → nozzle 中心位置"""
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)

    # 蓝色 mount (截图中 nozzle 上方的蓝色方块)
    mask = cv2.inRange(hsv, np.array([100, 100, 80]), np.array([130, 255, 255]))
    k = cv2.getStructuringElement(cv2.MORPH_RECT, (7, 7))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k, iterations=3)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k, iterations=2)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    valid = [(c, cv2.contourArea(c)) for c in contours if 1000 < cv2.contourArea(c) < 300000]
    if not valid:
        # fallback: 试橙色
        mask = cv2.inRange(hsv, np.array([5, 150, 180]), np.array([25, 255, 255]))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k, iterations=3)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k, iterations=2)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        valid = [(c, cv2.contourArea(c)) for c in contours if 1000 < cv2.contourArea(c) < 300000]
        if not valid:
            return None

    valid.sort(key=lambda t: t[1], reverse=True)
    c, _ = valid[0]
    x, y, w, h = cv2.boundingRect(c)
    # nozzle ~ mount 底部中心
    return (x + w // 2, y + h)


def crop_roi(bgr, cx, cy, size=224):
    """以 (cx, cy) 为中心裁 size*size, clamp 边界"""
    H, W = bgr.shape[:2]
    half = size // 2
    x1 = max(0, min(cx - half, W - size))
    y1 = max(0, min(cy - half, H - size))
    return bgr[y1:y1+size, x1:x1+size].copy(), x1, y1


def filter_bead_masks(masks_data, roi_size):
    """
    从 SAM2 mask 中筛选 bead:
      - 排除太大 (nozzle 筒身) 和太小的
      - 只留纵横比高的细长结构
    """
    candidates = []
    roi_area = roi_size * roi_size

    for m in masks_data:
        seg = m['segmentation'].astype(np.uint8)
        area = m['area']

        # 排除 ROI 面积占比 > 10% 的大块 (nozzle/背景)
        if area > roi_area * 0.10:
            continue
        if area < BEAD_MIN_AREA or area > BEAD_MAX_AREA:
            continue

        cnts, _ = cv2.findContours(seg * 255, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not cnts:
            continue
        cnt = max(cnts, key=cv2.contourArea)
        if len(cnt) < 5:
            continue

        rect = cv2.minAreaRect(cnt)
        (rcx, rcy), (rw, rh), angle = rect
        aspect = max(rw, rh) / (min(rw, rh) + 1e-6)

        if aspect < BEAD_MIN_ASPECT:
            continue

        hull = cv2.convexHull(cnt)
        hull_area = cv2.contourArea(hull)
        solidity = area / (hull_area + 1e-6)

        candidates.append({
            'mask': seg,
            'area': area,
            'aspect': aspect,
            'solidity': solidity,
            'long_px': max(rw, rh),
            'short_px': min(rw, rh),
            'angle': angle,
            'center': (rcx, rcy),
            'bbox': m['bbox'],
            'iou': m['predicted_iou'],
            'stability': m['stability_score'],
            'rect': rect,
        })

    candidates.sort(key=lambda x: x['aspect'], reverse=True)
    return candidates


def measure_bead_width(mask_uint8, n_samples=N_SCANLINES):
    """沿 bead 长轴取垂直切面, 测短轴宽度"""
    ys, xs = np.where(mask_uint8 > 0)
    if len(xs) < 5:
        return []

    x_span = xs.max() - xs.min()
    y_span = ys.max() - ys.min()
    widths = []

    if x_span >= y_span:
        # bead 沿 X 方向 → 沿 Y 测宽
        sample_pos = np.linspace(xs.min() + 2, xs.max() - 2, n_samples).astype(int)
        for sx in sample_pos:
            for s, e in _find_runs(mask_uint8[:, sx]):
                w = e - s
                if WIDTH_MIN <= w <= WIDTH_MAX:
                    widths.append({'pos': sx, 'start': s, 'end': e, 'width': w, 'axis': 'Y'})
    else:
        # bead 沿 Y 方向 → 沿 X 测宽
        sample_pos = np.linspace(ys.min() + 2, ys.max() - 2, n_samples).astype(int)
        for sy in sample_pos:
            for s, e in _find_runs(mask_uint8[sy, :]):
                w = e - s
                if WIDTH_MIN <= w <= WIDTH_MAX:
                    widths.append({'pos': sy, 'start': s, 'end': e, 'width': w, 'axis': 'X'})
    return widths


def _find_runs(arr):
    """找 1D 数组中连续非零段"""
    d = np.diff(arr.astype(np.int16))
    starts = list(np.where(d > 0)[0] + 1)
    ends = list(np.where(d < 0)[0] + 1)
    if not starts and arr[0] > 0:
        starts = [0]
    if not ends and arr[-1] > 0:
        ends = [len(arr)]
    return list(zip(starts, ends))


# ─── Visualization ───────────────────────────────────

def save_vis(bgr_full, bgr_roi, x1, y1, nozzle, bead_cands,
             width_data, mm_per_px, out_dir, fidx):
    # 1) 全图: ROI 框 + nozzle
    vis = bgr_full.copy()
    cv2.rectangle(vis, (x1, y1), (x1 + ROI_SIZE, y1 + ROI_SIZE), (0, 255, 255), 2)
    if nozzle:
        cv2.drawMarker(vis, nozzle, (0, 0, 255), cv2.MARKER_CROSS, 20, 2)
    cv2.putText(vis, f"Frame {fidx} | ROI 224x224", (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
    cv2.imwrite(str(out_dir / f"f{fidx}_full.png"), vis)

    # 2) ROI: bead overlay + width lines
    overlay = bgr_roi.copy()
    colors = [(0, 255, 0), (0, 200, 255), (255, 0, 255)]
    for i, cand in enumerate(bead_cands[:3]):
        c = colors[i % len(colors)]
        overlay[cand['mask'] > 0] = c
    blend = cv2.addWeighted(bgr_roi, 0.5, overlay, 0.5, 0)

    for wd in width_data:
        if wd['axis'] == 'Y':
            cv2.line(blend, (wd['pos'], wd['start']), (wd['pos'], wd['end']), (0,255,255), 1)
        else:
            cv2.line(blend, (wd['start'], wd['pos']), (wd['end'], wd['pos']), (0,255,255), 1)

    if width_data:
        ws = [wd['width'] for wd in width_data]
        w_mean = np.mean(ws)
        if mm_per_px is not None:
            label = f"{w_mean:.1f}px = {w_mean*mm_per_px:.2f}mm"
        else:
            label = f"{w_mean:.1f}px"
        cv2.putText(blend, label, (5, 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1)

    cv2.imwrite(str(out_dir / f"f{fidx}_roi_bead.png"), blend)
    blend_3x = cv2.resize(blend, (ROI_SIZE*3, ROI_SIZE*3), interpolation=cv2.INTER_NEAREST)
    cv2.imwrite(str(out_dir / f"f{fidx}_roi_bead_3x.png"), blend_3x)

    # 3) bead mask
    mask_all = np.zeros(bgr_roi.shape[:2], dtype=np.uint8)
    for cand in bead_cands[:3]:
        mask_all[cand['mask'] > 0] = 255
    cv2.imwrite(str(out_dir / f"f{fidx}_roi_mask.png"), mask_all)

    # 4) 原始 ROI
    cv2.imwrite(str(out_dir / f"f{fidx}_roi_raw.png"), bgr_roi)


# ─── Main ────────────────────────────────────────────

def main():
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    device = "cuda" if torch.cuda.is_available() else "cpu"

    print(f"=== SAM2 224x224 Bead Width (v2) ===")
    print(f"Device: {device}")
    print(f"Frame:  {FRAME_IDX}\n")

    # ── 取帧: RGB + Depth ──
    zed = sl.Camera()
    p = sl.InitParameters()
    p.set_from_svo_file(SVO_PATH)
    p.svo_real_time_mode = False
    p.depth_mode = sl.DEPTH_MODE.ULTRA      # Pascal GPU 不支持 NEURAL
    p.coordinate_units = sl.UNIT.METER
    if zed.open(p) != sl.ERROR_CODE.SUCCESS:
        print("ZED open failed"); sys.exit(1)

    calib = zed.get_camera_information().camera_configuration.calibration_parameters
    fx = calib.left_cam.fx
    info = zed.get_camera_information().camera_configuration
    W, H = info.resolution.width, info.resolution.height
    total = zed.get_svo_number_of_frames()
    print(f"Resolution: {W}x{H}, fx={fx:.1f}, Total: {total} frames")

    zed.set_svo_position(FRAME_IDX)
    img_mat = sl.Mat()
    depth_mat = sl.Mat()
    if zed.grab() != sl.ERROR_CODE.SUCCESS:
        print("Grab failed"); sys.exit(1)

    zed.retrieve_image(img_mat, sl.VIEW.LEFT)
    zed.retrieve_measure(depth_mat, sl.MEASURE.DEPTH)

    bgr = img_mat.get_data()[:, :, :3].copy()
    bgr = cv2.cvtColor(bgr, cv2.COLOR_RGB2BGR)
    depth = depth_mat.get_data()
    zed.close()
    print(f"Frame {FRAME_IDX} extracted\n")

    # ── 检测 nozzle ──
    nozzle = find_nozzle(bgr)
    if nozzle is None:
        print("Nozzle not found, using frame center")
        nozzle = (W // 2, H // 2)
    else:
        print(f"Nozzle at ({nozzle[0]}, {nozzle[1]})")

    # ── ROI: nozzle 旁边的 bead 区域 ──
    roi_cx = nozzle[0] + ROI_DX
    roi_cy = nozzle[1] + ROI_DY
    bgr_roi, x1, y1 = crop_roi(bgr, roi_cx, roi_cy, ROI_SIZE)
    depth_roi = depth[y1:y1+ROI_SIZE, x1:x1+ROI_SIZE].copy()

    print(f"ROI: ({x1},{y1}) -> ({x1+ROI_SIZE},{y1+ROI_SIZE})")
    print(f"ROI offset from nozzle: dx={ROI_DX}, dy={ROI_DY}\n")

    # ROI 区域中位深度 → mm/px 换算
    valid_depth = depth_roi[np.isfinite(depth_roi) & (depth_roi > 0)]
    if len(valid_depth) > 0:
        z_med = float(np.median(valid_depth))
        mm_per_px = z_med * 1000.0 / fx
        print(f"Depth (median): {z_med:.3f} m")
        print(f"Scale: 1px = {mm_per_px:.3f} mm\n")
    else:
        z_med = 0
        mm_per_px = None
        print("Depth unavailable in ROI\n")

    # ── SAM2 ──
    print("Loading SAM 2.1...", end="", flush=True)
    sam2 = build_sam2(MODEL_CFG, str(CKPT_PATH), device=device, apply_postprocessing=False)
    mask_gen = SAM2AutomaticMaskGenerator(sam2, **AMG_PARAMS)
    print(" Done")

    rgb_roi = cv2.cvtColor(bgr_roi, cv2.COLOR_BGR2RGB)
    print("Generating masks...", end="", flush=True)
    torch.cuda.empty_cache()
    with torch.inference_mode():
        masks = mask_gen.generate(rgb_roi)
    print(f" {len(masks)} masks")

    print(f"\n  All masks:")
    roi_area = ROI_SIZE * ROI_SIZE
    for i, m in enumerate(masks):
        seg = m['segmentation'].astype(np.uint8)
        area = m['area']
        cnts, _ = cv2.findContours(seg * 255, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        aspect = 0
        if cnts:
            cnt = max(cnts, key=cv2.contourArea)
            if len(cnt) >= 5:
                (_, _), (rw, rh), _ = cv2.minAreaRect(cnt)
                aspect = max(rw, rh) / (min(rw, rh) + 1e-6)
        print(f"    [{i}] area={area} ({area/roi_area*100:.1f}%), "
              f"aspect={aspect:.1f}, iou={m['predicted_iou']:.2f}")

    # ── 筛选 bead ──
    bead_cands = filter_bead_masks(masks, ROI_SIZE)
    print(f"\nBead candidates: {len(bead_cands)}")

    # ── 宽度测量 ──
    all_wd = []
    for i, cand in enumerate(bead_cands[:3]):
        wd = measure_bead_width(cand['mask'] * 255)
        all_wd.extend(wd)
        ws = [w['width'] for w in wd]
        if ws:
            w_mean = np.mean(ws)
            mm_str = f" = {w_mean * mm_per_px:.2f}mm" if mm_per_px else ""
            print(f"  #{i}: area={cand['area']}, aspect={cand['aspect']:.1f}, "
                  f"width={w_mean:.1f}+/-{np.std(ws):.1f}px{mm_str} (n={len(ws)})")
        else:
            print(f"  #{i}: area={cand['area']}, aspect={cand['aspect']:.1f}, no width")

    # ── 汇总 ──
    print(f"\n{'='*55}")
    if all_wd:
        ws = np.array([w['width'] for w in all_wd])
        print(f"Bead Width (Frame {FRAME_IDX}, 224x224 ROI):")
        print(f"  Pixels:  {ws.mean():.1f} +/- {ws.std():.1f} px  "
              f"(median={np.median(ws):.1f}, range=[{ws.min()},{ws.max()}])")
        if mm_per_px:
            ws_mm = ws * mm_per_px
            print(f"  Metric:  {ws_mm.mean():.2f} +/- {ws_mm.std():.2f} mm  "
                  f"(median={np.median(ws_mm):.2f}mm)")
            print(f"  Scale:   1px = {mm_per_px:.3f}mm  (Z={z_med:.3f}m, fx={fx:.1f})")
        print(f"  Samples: {len(ws)}")
    else:
        print("No bead detected.")

    # ── 可视化 ──
    save_vis(bgr, bgr_roi, x1, y1, nozzle, bead_cands, all_wd, mm_per_px,
             OUT_DIR, FRAME_IDX)
    print(f"\nOutput: {OUT_DIR}")


if __name__ == "__main__":
    main()
