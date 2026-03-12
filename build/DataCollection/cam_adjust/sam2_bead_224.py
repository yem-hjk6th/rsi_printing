"""
sam2_bead_224.py — 224×224 ROI + SAM2 bead width 测量 (v4)

流程:
  1. SVO2 取帧 (RGB + Depth)
  2. 检测 nozzle 位置 (蓝色 mount HSV)
  3. ROI 224×224: 以 nozzle 为锚, 向 bead 侧偏移
  4. SAM2 分割 → 排除 nozzle 筒身 + 上方瘢痕 → 留 bead
  5. 测 bead width: 排除 nozzle 遮挡的锥形段 (cone filtering)
  6. px → mm, CSV + 置信度, 每帧计时
"""

import sys, csv, cv2, time, random, numpy as np
import pyzed.sl as sl
from pathlib import Path
from datetime import datetime
import torch

from sam2.build_sam import build_sam2
from sam2.automatic_mask_generator import SAM2AutomaticMaskGenerator

# ─── Config ──────────────────────────────────────────
SVO_PATH  = r"C:\Users\dell\Desktop\RSI\recorded_data\20260310_183338\recording_20260310_183338.svo2"
CKPT_PATH = Path(__file__).parent / "t2" / "sam2_checkpoints" / "sam2.1_hiera_small.pt"
MODEL_CFG = "configs/sam2.1/sam2.1_hiera_s.yaml"

LABELING_DIR = Path(__file__).resolve().parent.parent / "labeling" / "post_labeling" / "t1"

# 从 800-4800 随机采 50 帧 (用于 anno 生成 + 下游校准)
random.seed(42)
FRAME_LIST = sorted(random.sample(range(800, 4801), 50))
ROI_SIZE   = 224

ROI_DX = -30
ROI_DY = 350

# SAM2
AMG_PARAMS = dict(
    points_per_side=32,
    points_per_batch=64,
    pred_iou_thresh=0.7,
    stability_score_thresh=0.85,
    min_mask_region_area=20,
)

# bead 筛选
BEAD_MIN_ASPECT = 2.5
BEAD_MIN_AREA   = 30
BEAD_MAX_AREA   = 8000
SCAR_ZONE_RATIO = 0.40

# 宽度测量
WIDTH_MIN   = 2
WIDTH_MAX   = 40
N_SCANLINES = 30          # 增加采样密度
CONE_RATIO  = 0.55        # width < median*CONE_RATIO → 锥形段, 排除


# ─── Core ────────────────────────────────────────────

def find_nozzle(bgr):
    """检测蓝色 mount → 返回 mount 底部中心 (nozzle 锚点)"""
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, np.array([100, 100, 80]), np.array([130, 255, 255]))
    k = cv2.getStructuringElement(cv2.MORPH_RECT, (7, 7))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k, iterations=3)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k, iterations=2)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    valid = [(c, cv2.contourArea(c)) for c in contours if 1000 < cv2.contourArea(c) < 300000]

    if not valid:
        # fallback: 橙色 nozzle
        mask = cv2.inRange(hsv, np.array([5, 150, 180]), np.array([25, 255, 255]))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k, iterations=3)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k, iterations=2)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        valid = [(c, cv2.contourArea(c)) for c in contours if 1000 < cv2.contourArea(c) < 300000]
        if not valid:
            return None

    valid.sort(key=lambda t: t[1], reverse=True)
    c = valid[0][0]
    x, y, w, h = cv2.boundingRect(c)
    return (x + w // 2, y + h)


def crop_roi(img, cx, cy, size=224):
    """以 (cx,cy) 为中心裁 size×size, clamp 边界"""
    H, W = img.shape[:2]
    half = size // 2
    x1 = max(0, min(cx - half, W - size))
    y1 = max(0, min(cy - half, H - size))
    return img[y1:y1+size, x1:x1+size].copy(), x1, y1


def filter_bead_masks(masks_data, roi_size):
    """筛选 bead mask: 排除大面积 + 排除上方瘢痕 + 要求细长"""
    candidates = []
    roi_area = roi_size * roi_size
    scar_y = roi_size * SCAR_ZONE_RATIO

    for m in masks_data:
        seg = m['segmentation'].astype(np.uint8)
        area = m['area']

        if area > roi_area * 0.10 or area < BEAD_MIN_AREA or area > BEAD_MAX_AREA:
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

        # 瘢痕过滤: mask 中心在 ROI 上部 → nozzle 表面瑕疵, 跳过
        if rcy < scar_y:
            continue

        hull = cv2.convexHull(cnt)
        solidity = area / (cv2.contourArea(hull) + 1e-6)

        candidates.append({
            'mask': seg, 'area': area, 'aspect': aspect,
            'solidity': solidity, 'long_px': max(rw, rh),
            'short_px': min(rw, rh), 'angle': angle,
            'center': (rcx, rcy), 'bbox': m['bbox'],
            'iou': m['predicted_iou'], 'stability': m['stability_score'],
        })

    candidates.sort(key=lambda x: x['aspect'], reverse=True)
    return candidates


def measure_bead_width(mask_uint8, n_samples=N_SCANLINES):
    """
    沿 bead 长轴取垂直切面, 测短轴宽度 (成对边界).
    每条 scanline 上找到 mask 的连续非零段 → 段长就是一对边界间距 = width.
    之后做 cone filtering: 排除 width < median*CONE_RATIO 的锥形段.

    返回 (kept_widths, removed_count)
    """
    ys, xs = np.where(mask_uint8 > 0)
    if len(xs) < 5:
        return [], 0

    x_span = xs.max() - xs.min()
    y_span = ys.max() - ys.min()
    raw_widths = []

    if x_span >= y_span:
        for sx in np.linspace(xs.min() + 2, xs.max() - 2, n_samples).astype(int):
            for s, e in _find_runs(mask_uint8[:, sx]):
                w = e - s
                if WIDTH_MIN <= w <= WIDTH_MAX:
                    raw_widths.append({'pos': sx, 'start': s, 'end': e, 'width': w, 'axis': 'Y'})
    else:
        for sy in np.linspace(ys.min() + 2, ys.max() - 2, n_samples).astype(int):
            for s, e in _find_runs(mask_uint8[sy, :]):
                w = e - s
                if WIDTH_MIN <= w <= WIDTH_MAX:
                    raw_widths.append({'pos': sy, 'start': s, 'end': e, 'width': w, 'axis': 'X'})

    if not raw_widths:
        return [], 0

    # Cone filtering: 锥形段的 width 远小于稳定段
    ws = np.array([w['width'] for w in raw_widths])
    med_w = np.median(ws)
    threshold = med_w * CONE_RATIO
    kept = [w for w in raw_widths if w['width'] >= threshold]
    removed = len(raw_widths) - len(kept)
    return kept, removed


def _find_runs(arr):
    """1D 数组中连续非零段 → [(start, end), ...]"""
    d = np.diff(arr.astype(np.int16))
    starts = list(np.where(d > 0)[0] + 1)
    ends   = list(np.where(d < 0)[0] + 1)
    if not starts and arr[0] > 0:
        starts = [0]
    if not ends and arr[-1] > 0:
        ends = [len(arr)]
    return list(zip(starts, ends))


# ─── GUM uncertainty constants ───
U_Z  = 0.002   # ZED 2i depth noise ~2mm at 0.45m
U_PX = 0.5     # segmentation boundary uncertainty ±0.5px
U_FX = 1.0     # factory calibration error ~1px

def compute_uncertainty(w_px_arr, mm_per_px, z_med, fx):
    """
    GUM 测量不确定度:
      - ci95_px/mm: 95% 置信区间 (t=1.96)
      - u_w_mm: 误差传播 sqrt((u_Z/Z)^2 + (u_px/w_px)^2 + (u_fx/fx)^2) * w_mm
    """
    n = len(w_px_arr)
    w_mean = w_px_arr.mean()
    w_std  = w_px_arr.std()
    ci95_px = 1.96 * w_std / np.sqrt(n) if n > 1 else w_std

    if mm_per_px and z_med > 0 and w_mean > 0:
        w_mm = w_mean * mm_per_px
        ci95_mm = ci95_px * mm_per_px
        u_w_mm = w_mm * np.sqrt(
            (U_Z / z_med)**2 + (U_PX / w_mean)**2 + (U_FX / fx)**2
        )
    else:
        w_mm = ci95_mm = u_w_mm = None

    return {
        'ci95_px': round(ci95_px, 3),
        'ci95_mm': round(ci95_mm, 4) if ci95_mm is not None else '',
        'u_w_mm':  round(u_w_mm, 4) if u_w_mm is not None else '',
    }


# ─── Visualization ───────────────────────────────────

def save_vis(bgr_full, bgr_roi, x1, y1, nozzle, bead_cands,
             width_data, mm_per_px, out_dir, fidx):
    # 全图 + ROI 框
    vis = bgr_full.copy()
    cv2.rectangle(vis, (x1, y1), (x1 + ROI_SIZE, y1 + ROI_SIZE), (0, 255, 255), 2)
    if nozzle:
        cv2.drawMarker(vis, nozzle, (0, 0, 255), cv2.MARKER_CROSS, 20, 2)
    cv2.putText(vis, f"Frame {fidx}", (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
    cv2.imwrite(str(out_dir / f"f{fidx}_full.png"), vis)

    # ROI overlay
    overlay = bgr_roi.copy()
    colors = [(0, 255, 0), (0, 200, 255), (255, 0, 255)]
    for i, cand in enumerate(bead_cands[:1]):
        overlay[cand['mask'] > 0] = colors[i % len(colors)]
    blend = cv2.addWeighted(bgr_roi, 0.5, overlay, 0.5, 0)

    # 瘢痕分界线
    sy = int(ROI_SIZE * SCAR_ZONE_RATIO)
    cv2.line(blend, (0, sy), (ROI_SIZE, sy), (0, 0, 255), 1)
    cv2.putText(blend, "scar zone", (2, sy - 4),
                cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0, 0, 255), 1)

    for wd in width_data:
        if wd['axis'] == 'Y':
            cv2.line(blend, (wd['pos'], wd['start']), (wd['pos'], wd['end']), (0,255,255), 1)
        else:
            cv2.line(blend, (wd['start'], wd['pos']), (wd['end'], wd['pos']), (0,255,255), 1)

    if width_data:
        w_mean = np.mean([wd['width'] for wd in width_data])
        label = f"{w_mean:.1f}px"
        if mm_per_px is not None:
            label += f" = {w_mean*mm_per_px:.2f}mm"
        cv2.putText(blend, label, (5, 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1)

    cv2.imwrite(str(out_dir / f"f{fidx}_roi_bead.png"), blend)
    cv2.imwrite(str(out_dir / f"f{fidx}_roi_bead_3x.png"),
                cv2.resize(blend, (ROI_SIZE*3, ROI_SIZE*3), interpolation=cv2.INTER_NEAREST))

    # mask
    mask_all = np.zeros(bgr_roi.shape[:2], dtype=np.uint8)
    for cand in bead_cands[:1]:
        mask_all[cand['mask'] > 0] = 255
    cv2.imwrite(str(out_dir / f"f{fidx}_roi_mask.png"), mask_all)
    cv2.imwrite(str(out_dir / f"f{fidx}_roi_raw.png"), bgr_roi)


# ─── Main ────────────────────────────────────────────

def main():
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    out_dir = LABELING_DIR / timestamp
    out_dir.mkdir(parents=True, exist_ok=True)
    device = "cuda" if torch.cuda.is_available() else "cpu"

    print(f"=== SAM2 224x224 Bead Width (v4) ===")
    print(f"Device:  {device}")
    print(f"Frames:  {FRAME_LIST}")
    print(f"Output:  {out_dir}\n")

    # ── Open SVO ──
    zed = sl.Camera()
    p = sl.InitParameters()
    p.set_from_svo_file(SVO_PATH)
    p.svo_real_time_mode = False
    p.depth_mode = sl.DEPTH_MODE.ULTRA
    p.coordinate_units = sl.UNIT.METER
    if zed.open(p) != sl.ERROR_CODE.SUCCESS:
        print("ZED open failed"); sys.exit(1)

    calib = zed.get_camera_information().camera_configuration.calibration_parameters
    fx = calib.left_cam.fx
    fy = calib.left_cam.fy
    cx_cam = calib.left_cam.cx
    cy_cam = calib.left_cam.cy
    info = zed.get_camera_information().camera_configuration
    W, H = info.resolution.width, info.resolution.height
    total = zed.get_svo_number_of_frames()

    cam_model  = zed.get_camera_information().camera_model
    cam_serial = zed.get_camera_information().serial_number
    print(f"Camera:  model={cam_model}, serial={cam_serial}")
    print(f"Resolution: {W}x{H}")
    print(f"Intrinsics (left): fx={fx:.1f} fy={fy:.1f} cx={cx_cam:.1f} cy={cy_cam:.1f}")
    print(f"Total frames: {total}\n")

    # ── Load SAM2 (一次加载, 多帧复用) ──
    print("Loading SAM 2.1...", end="", flush=True)
    sam2 = build_sam2(MODEL_CFG, str(CKPT_PATH), device=device, apply_postprocessing=False)
    mask_gen = SAM2AutomaticMaskGenerator(sam2, **AMG_PARAMS)
    print(" Done\n")

    img_mat   = sl.Mat()
    depth_mat = sl.Mat()
    csv_rows  = []
    timing_rows = []

    for fidx in FRAME_LIST:
        print(f"{'='*55}")
        print(f"Frame {fidx}")
        t_frame_start = time.perf_counter()

        if fidx >= total:
            print(f"  Skipped (only {total} frames)\n")
            continue

        zed.set_svo_position(fidx)
        if zed.grab() != sl.ERROR_CODE.SUCCESS:
            print(f"  Grab failed\n"); continue

        zed.retrieve_image(img_mat, sl.VIEW.LEFT)
        zed.retrieve_measure(depth_mat, sl.MEASURE.DEPTH)

        bgr = cv2.cvtColor(img_mat.get_data()[:, :, :3], cv2.COLOR_RGB2BGR)
        depth = depth_mat.get_data()

        # ── Nozzle ──
        nozzle = find_nozzle(bgr)
        if nozzle is None:
            print("  Nozzle not found → using center")
            nozzle = (W // 2, H // 2)
        else:
            print(f"  Nozzle: ({nozzle[0]}, {nozzle[1]})")

        # ── ROI ──
        roi_cx = nozzle[0] + ROI_DX
        roi_cy = nozzle[1] + ROI_DY
        bgr_roi, x1, y1 = crop_roi(bgr, roi_cx, roi_cy, ROI_SIZE)
        depth_roi = depth[y1:y1+ROI_SIZE, x1:x1+ROI_SIZE].copy()
        print(f"  ROI: ({x1},{y1})→({x1+ROI_SIZE},{y1+ROI_SIZE})")

        # 深度 → mm/px
        valid_d = depth_roi[np.isfinite(depth_roi) & (depth_roi > 0)]
        if len(valid_d) > 0:
            z_med = float(np.median(valid_d))
            mm_per_px = z_med * 1000.0 / fx
            print(f"  Depth: {z_med:.3f}m → 1px = {mm_per_px:.4f}mm")
        else:
            z_med, mm_per_px = 0.0, None
            print("  Depth unavailable")

        # ── SAM2 ──
        t_sam = time.perf_counter()
        rgb_roi = cv2.cvtColor(bgr_roi, cv2.COLOR_BGR2RGB)
        torch.cuda.empty_cache()
        with torch.inference_mode():
            masks = mask_gen.generate(rgb_roi)
        t_sam_ms = (time.perf_counter() - t_sam) * 1000
        print(f"  SAM2: {len(masks)} masks ({t_sam_ms:.0f}ms)")

        # mask 信息
        scar_y = ROI_SIZE * SCAR_ZONE_RATIO
        roi_area = ROI_SIZE * ROI_SIZE
        for i, m in enumerate(masks):
            seg = m['segmentation'].astype(np.uint8)
            area = m['area']
            cnts, _ = cv2.findContours(seg * 255, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            aspect, cy_m = 0, 0
            if cnts:
                cnt = max(cnts, key=cv2.contourArea)
                if len(cnt) >= 5:
                    (_, cy_m), (rw, rh), _ = cv2.minAreaRect(cnt)
                    aspect = max(rw, rh) / (min(rw, rh) + 1e-6)
            tag = ""
            if area > roi_area * 0.10:
                tag = " [TOO_LARGE]"
            elif cy_m < scar_y and aspect >= BEAD_MIN_ASPECT:
                tag = " [SCAR_ZONE]"
            print(f"    [{i}] area={area} aspect={aspect:.1f} cy={cy_m:.0f}{tag}")

        # ── 筛选 bead ──
        bead_cands = filter_bead_masks(masks, ROI_SIZE)
        print(f"  Bead candidates: {len(bead_cands)}")

        # ── 宽度测量 (只取 top-1 candidate: 最高 aspect = 最 bead-like) ──
        t_width = time.perf_counter()
        all_wd = []
        total_cone_removed = 0
        for i, cand in enumerate(bead_cands[:1]):
            wd, cone_removed = measure_bead_width(cand['mask'] * 255)
            total_cone_removed += cone_removed
            all_wd.extend(wd)
            ws = [w['width'] for w in wd]

            if ws:
                w_arr = np.array(ws, dtype=float)
                w_mean, w_std, w_med = w_arr.mean(), w_arr.std(), float(np.median(w_arr))
                w_mm = w_mean * mm_per_px if mm_per_px else None
                unc = compute_uncertainty(w_arr, mm_per_px, z_med, fx)
                print(f"    #{i}: area={cand['area']} aspect={cand['aspect']:.1f} "
                      f"w={w_mean:.1f}±{w_std:.1f}px", end="")
                if w_mm is not None:
                    print(f" = {w_mm:.2f}mm (CI95={unc['ci95_mm']}mm, "
                          f"u_w={unc['u_w_mm']}mm)", end="")
                print(f"  (n_pairs={len(ws)}, cone_cut={cone_removed})")

                # 保存 bead mask 为 NPZ (供 meijering/sobel 校准用)
                np.savez_compressed(
                    str(out_dir / f"f{fidx}_bead_mask.npz"),
                    mask=cand['mask'],
                    roi_origin=np.array([x1, y1]),
                )

                csv_rows.append({
                    'frame': fidx,
                    'candidate': i,
                    'width_mean_px': round(w_mean, 2),
                    'width_std_px': round(w_std, 2),
                    'width_median_px': round(w_med, 2),
                    'width_min_px': int(w_arr.min()),
                    'width_max_px': int(w_arr.max()),
                    'width_mean_mm': round(w_mean * mm_per_px, 3) if mm_per_px else '',
                    'width_std_mm': round(w_std * mm_per_px, 3) if mm_per_px else '',
                    'ci95_px': unc['ci95_px'],
                    'ci95_mm': unc['ci95_mm'],
                    'u_w_mm': unc['u_w_mm'],
                    'mm_per_px': round(mm_per_px, 4) if mm_per_px else '',
                    'depth_m': round(z_med, 4),
                    'aspect': round(cand['aspect'], 2),
                    'solidity': round(cand['solidity'], 3),
                    'iou': round(cand['iou'], 3),
                    'stability': round(cand['stability'], 3),
                    'n_pairs': len(ws),
                    'cone_removed': cone_removed,
                    'bead_long_px': round(cand['long_px'], 1),
                })
            else:
                print(f"    #{i}: area={cand['area']} no width")
        t_width_ms = (time.perf_counter() - t_width) * 1000

        # ── Frame 汇总 ──
        t_frame_ms = (time.perf_counter() - t_frame_start) * 1000
        if all_wd:
            ws_all = np.array([w['width'] for w in all_wd])
            summary = f"  → {ws_all.mean():.1f}±{ws_all.std():.1f}px"
            if mm_per_px:
                summary += f" = {ws_all.mean()*mm_per_px:.2f}±{ws_all.std()*mm_per_px:.2f}mm"
            print(f"{summary} "
                  f"(n_pairs={len(ws_all)}, cone_cut={total_cone_removed})")
        else:
            print("  → No bead detected")
        print(f"  Timing: SAM2={t_sam_ms:.0f}ms  width={t_width_ms:.1f}ms  "
              f"total={t_frame_ms:.0f}ms")

        timing_rows.append({
            'frame': fidx, 'sam2_ms': round(t_sam_ms, 1),
            'width_ms': round(t_width_ms, 2), 'total_ms': round(t_frame_ms, 1),
        })

        save_vis(bgr, bgr_roi, x1, y1, nozzle, bead_cands, all_wd, mm_per_px, out_dir, fidx)
        print()

    zed.close()

    # ── CSV 输出 ──
    csv_path = out_dir / "bead_width.csv"
    if csv_rows:
        with open(csv_path, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=list(csv_rows[0].keys()))
            writer.writeheader()
            writer.writerows(csv_rows)
        print(f"CSV: {csv_path}")

    # ── Timing CSV ──
    timing_path = out_dir / "timing.csv"
    if timing_rows:
        with open(timing_path, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=list(timing_rows[0].keys()))
            writer.writeheader()
            writer.writerows(timing_rows)
        print(f"Timing CSV: {timing_path}")

    # ── 摘要 ──
    print(f"\n{'='*55}")
    print(f"Camera: {cam_model} (S/N {cam_serial})")
    print(f"Intrinsics: fx={fx:.1f} fy={fy:.1f}")
    if csv_rows:
        all_mm = [r['width_mean_mm'] for r in csv_rows if r['width_mean_mm'] != '']
        if all_mm:
            print(f"Bead width across {len(all_mm)} measurements: "
                  f"{np.mean(all_mm):.2f} ± {np.std(all_mm):.2f} mm")
        all_pairs = sum(r['n_pairs'] for r in csv_rows)
        all_cone  = sum(r['cone_removed'] for r in csv_rows)
        print(f"Total: {all_pairs} width pairs kept, {all_cone} cone points removed")
    if timing_rows:
        t_avg = np.mean([t['total_ms'] for t in timing_rows])
        t_sam_avg = np.mean([t['sam2_ms'] for t in timing_rows])
        t_w_avg = np.mean([t['width_ms'] for t in timing_rows])
        print(f"Avg timing: SAM2={t_sam_avg:.0f}ms  width={t_w_avg:.1f}ms  "
              f"total={t_avg:.0f}ms/frame")
    print(f"Output: {out_dir}")


if __name__ == "__main__":
    main()
