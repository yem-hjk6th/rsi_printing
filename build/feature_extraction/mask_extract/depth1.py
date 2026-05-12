"""
depth1.py — ZED 深度图 + bead 区域分层探索
  纯深度方案: 不依赖外参, 直接用 ZED SDK 的双目深度图在 bead ROI 内做 Z 分层。

=== 深度来源说明 ===

深度数据 100% 来自 ZED SDK 官方算法 (pyzed.sl):
  1. ZED SDK 内部做左右目 rectify → 立体匹配 (SGM / Neural) → disparity → depth
  2. 调用 zed.retrieve_measure(depth_mat, sl.MEASURE.DEPTH) 直接拿到每像素的 Z 值 (米)
  3. depth_mode 选项决定精度/速度:
     - NONE:         不算深度 (最快, 无深度)
     - PERFORMANCE:  SGM, 最快有深度的模式
     - QUALITY:      SGM 优化, 平衡
     - ULTRA:        最高精度 (Neural depth), 最慢
     - NEURAL:       同 ULTRA
  4. 我们不自己做立体匹配, 完全依赖 ZED SDK

  在 ~0.45m 工作距离下:
    - ZED 2i 深度精度 ~1-2mm (ULTRA 模式)
    - 分辨率: 与 RGB 相同 (左目视角)
    - 无效值: NaN / Inf (遮挡、反光区域)

=== 分层思路 ===

  1. 取 bead ROI 内的深度值
  2. 用颜色 mask 过滤只留 bead 表面
  3. bead 表面的 Z 值从底层 (远离相机) → 顶层 (靠近相机) 递减
  4. 对 Z 值做直方图, 每个峰 = 一个层面
  5. 或: 沿 Y 轴 (图像垂直方向) 看 Z 的阶梯变化
"""

import sys, os, csv, cv2, time, json
import numpy as np
from pathlib import Path
from datetime import datetime
from scipy.signal import find_peaks
from scipy.ndimage import uniform_filter1d

# ZED SDK DLL paths
if os.name == "nt":
    for p in [
        r"C:\Program Files (x86)\ZED SDK\bin",
        r"C:\Program Files (x86)\ZED SDK\dependencies\bin",
        r"C:\Program Files\NVIDIA GPU Computing Toolkit\CUDA\v12.8\bin",
    ]:
        if os.path.isdir(p):
            os.add_dll_directory(p)

import pyzed.sl as sl

# ═══════════════════════════════════════════════════════════════════════════════
#  USER CONFIG
# ═══════════════════════════════════════════════════════════════════════════════

SVO_PATH = Path(
    r"C:\Users\888y9\Desktop\rsi_printing\recorded_data"
    r"\20260331_202433\recording_20260331_202433_001_20260331_202433.svo2"
)

# ── 帧范围 ──
FRAME_START = 150
FRAME_END   = 11750
FRAME_STEP  = 100           # 深度计算较慢, 先粗采样

# ── ROI [x, y, w, h] ──
ROI = [1200, 600, 400, 400]

# ── ZED 深度模式 ──
#    ULTRA = 最高精度 (Neural), QUALITY = SGM 优化, PERFORMANCE = 最快
DEPTH_MODE = "ULTRA"

# ── Bead 颜色 mask (同 layer_sep_masks.py) ──
BEAD_SAT_MAX     = 50
BEAD_VAL_MIN     = 140
BEAD_MORPH_KSIZE = 11
BEAD_MORPH_ITER  = 4

# ── Z 值分层参数 ──
Z_HIST_BINS      = 200      # Z 直方图 bin 数
Z_RANGE_MIN      = 0.30     # 最近深度 (米), 小于这个值视为噪声
Z_RANGE_MAX      = 0.65     # 最远深度 (米), 大于这个值视为背景
Z_PEAK_DISTANCE  = 3        # 直方图上相邻峰最小 bin 间距 (= ~层高)
Z_PEAK_PROMINENCE = 0.05    # 峰值突出度 (相对于最高峰)
Z_SMOOTH_SIZE    = 5        # 直方图平滑窗口

# ── Y 轴 Z 剖面参数 ──
Y_PROFILE_SMOOTH = 5        # Y 方向 Z 均值平滑窗口
Y_STEP_THRESH    = 0.001    # Z 阶梯跳变阈值 (米), ~1mm

# ═══════════════════════════════════════════════════════════════════════════════
#  END USER CONFIG
# ═══════════════════════════════════════════════════════════════════════════════


def build_output_dir(svo_path: Path) -> Path:
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    out = svo_path.parent / f"depth_layer_{ts}"
    out.mkdir(parents=True, exist_ok=True)
    return out


def open_svo(svo_path: Path, depth_mode_str: str):
    zed = sl.Camera()
    p = sl.InitParameters()
    p.set_from_svo_file(str(svo_path))
    p.svo_real_time_mode = False
    p.coordinate_units = sl.UNIT.METER

    dm = getattr(sl.DEPTH_MODE, depth_mode_str, sl.DEPTH_MODE.ULTRA)
    p.depth_mode = dm
    p.depth_minimum_distance = 0.2
    p.depth_maximum_distance = 1.5

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


def extract_roi(img, roi):
    if roi is None:
        return img.copy(), 0, 0
    x, y, w, h = roi
    H, W = img.shape[:2]
    x = max(0, min(x, W - 1))
    y = max(0, min(y, H - 1))
    w = min(w, W - x)
    h = min(h, H - y)
    return img[y:y+h, x:x+w].copy(), x, y


def make_bead_mask(roi_bgr):
    hsv = cv2.cvtColor(roi_bgr, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv,
                       np.array([0, 0, BEAD_VAL_MIN]),
                       np.array([180, BEAD_SAT_MAX, 255]))
    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,
                                  (BEAD_MORPH_KSIZE, BEAD_MORPH_KSIZE))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k, iterations=BEAD_MORPH_ITER)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k, iterations=1)
    return mask


def analyze_depth_histogram(z_vals):
    """Z 值直方图 → 找峰值 = 各层的深度"""
    hist, bin_edges = np.histogram(z_vals, bins=Z_HIST_BINS,
                                   range=(Z_RANGE_MIN, Z_RANGE_MAX))
    bin_centers = (bin_edges[:-1] + bin_edges[1:]) / 2

    # 平滑
    hist_smooth = uniform_filter1d(hist.astype(np.float64), size=Z_SMOOTH_SIZE)
    if hist_smooth.max() > 0:
        hist_norm = hist_smooth / hist_smooth.max()
    else:
        hist_norm = hist_smooth

    peaks, props = find_peaks(
        hist_norm,
        distance=Z_PEAK_DISTANCE,
        prominence=Z_PEAK_PROMINENCE,
    )

    peak_z_values = bin_centers[peaks]
    return hist_norm, bin_centers, peaks, peak_z_values


def analyze_y_profile(depth_roi, bead_mask):
    """沿 Y 轴 (每行) 取 bead 区域的平均 Z → 找阶梯跳变"""
    H, W = depth_roi.shape
    y_means = np.full(H, np.nan)

    for y in range(H):
        row_mask = bead_mask[y, :]
        cols = np.where(row_mask > 0)[0]
        if len(cols) < 5:
            continue
        z_row = depth_roi[y, cols]
        valid = z_row[np.isfinite(z_row) & (z_row > Z_RANGE_MIN) & (z_row < Z_RANGE_MAX)]
        if len(valid) > 3:
            y_means[y] = np.median(valid)

    # 插值 NaN
    valid_idx = np.where(np.isfinite(y_means))[0]
    if len(valid_idx) < 10:
        return y_means, np.array([])

    y_interp = np.interp(np.arange(H), valid_idx, y_means[valid_idx])
    y_smooth = uniform_filter1d(y_interp, size=Y_PROFILE_SMOOTH)

    # 找阶梯: diff 的绝对值大于阈值
    dz = np.diff(y_smooth)
    steps = np.where(np.abs(dz) > Y_STEP_THRESH)[0]

    # 合并相邻的 step 点
    if len(steps) > 1:
        merged = [steps[0]]
        for s in steps[1:]:
            if s - merged[-1] > 5:
                merged.append(s)
        steps = np.array(merged)

    return y_smooth, steps


def draw_vis(roi_bgr, depth_roi, bead_mask, hist_norm, bin_centers,
             hist_peaks, peak_z, y_profile, y_steps, fidx):
    """4 合 1 可视化"""
    H, W = roi_bgr.shape[:2]
    canvas = np.zeros((H * 2, W * 2, 3), dtype=np.uint8)

    # 左上: 原图 + Y 轴阶梯位置 (红虚线)
    panel = roi_bgr.copy()
    for si, sy in enumerate(y_steps):
        # 虚线
        x = 0
        while x < W:
            cv2.line(panel, (x, sy), (min(x + 10, W), sy), (0, 0, 255), 2)
            x += 18
        cv2.putText(panel, f"S{si}", (5, sy - 3),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 255, 255), 1)
    cv2.putText(panel, f"F{fidx} | {len(y_steps)} steps",
                (5, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)
    canvas[0:H, 0:W] = panel

    # 右上: 深度图可视化 (colormap)
    z_vis = depth_roi.copy()
    z_vis[~np.isfinite(z_vis)] = 0
    z_vis = np.clip(z_vis, Z_RANGE_MIN, Z_RANGE_MAX)
    z_norm = ((z_vis - Z_RANGE_MIN) / (Z_RANGE_MAX - Z_RANGE_MIN) * 255).astype(np.uint8)
    z_color = cv2.applyColorMap(z_norm, cv2.COLORMAP_JET)
    # 非 bead 区域灰暗
    z_color[bead_mask == 0] = z_color[bead_mask == 0] // 3
    canvas[0:H, W:W*2] = z_color

    # 左下: bead mask
    mask_bgr = cv2.cvtColor(bead_mask, cv2.COLOR_GRAY2BGR)
    canvas[H:H*2, 0:W] = mask_bgr

    # 右下: Z 直方图 (横轴=Z, 纵轴=count)
    hist_img = np.zeros((H, W, 3), dtype=np.uint8)
    if len(hist_norm) > 0 and hist_norm.max() > 0:
        n_bins = len(hist_norm)
        for i in range(n_bins):
            x_pos = int(i / n_bins * W)
            bar_h = int(hist_norm[i] * (H - 20))
            cv2.line(hist_img, (x_pos, H), (x_pos, H - bar_h), (0, 200, 0), 1)
        for pi in hist_peaks:
            x_pos = int(pi / n_bins * W)
            cv2.line(hist_img, (x_pos, 0), (x_pos, H), (0, 0, 255), 1)
            z_label = f"{bin_centers[pi]:.3f}m"
            cv2.putText(hist_img, z_label, (x_pos + 2, 15),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0, 255, 255), 1)
    cv2.putText(hist_img, f"Z hist | {len(hist_peaks)} peaks",
                (5, H - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (200, 200, 200), 1)
    canvas[H:H*2, W:W*2] = hist_img

    # 单独: 干净原图 + 虚线
    clean = roi_bgr.copy()
    for si, sy in enumerate(y_steps):
        x = 0
        while x < W:
            cv2.line(clean, (x, sy), (min(x + 10, W), sy), (0, 0, 255), 2)
            x += 18
        cv2.putText(clean, f"S{si}", (3, sy - 3),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)
    cv2.putText(clean, f"F{fidx} | {len(y_steps)} steps",
                (5, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)

    return canvas, clean


def main():
    if not SVO_PATH.exists():
        print(f"[ERROR] SVO not found: {SVO_PATH}")
        sys.exit(1)

    out_dir = build_output_dir(SVO_PATH)

    print(f"{'='*60}")
    print(f"  Depth-based Layer Detection")
    print(f"{'='*60}")
    print(f"  SVO:        {SVO_PATH.name}")
    print(f"  Depth mode: {DEPTH_MODE}")
    print(f"  ROI:        {ROI}")
    print(f"  Z range:    {Z_RANGE_MIN}~{Z_RANGE_MAX}m")
    print(f"  Output:     {out_dir}")

    zed = open_svo(SVO_PATH, DEPTH_MODE)
    frame_list, total_frames = resolve_frame_range(
        zed, FRAME_START, FRAME_END, FRAME_STEP
    )
    n_frames = len(frame_list)

    res = zed.get_camera_information().camera_configuration.resolution
    calib = zed.get_camera_information().camera_configuration.calibration_parameters
    fx = calib.left_cam.fx

    print(f"  Resolution: {res.width} x {res.height}")
    print(f"  fx:         {fx:.2f}")
    print(f"  Total SVO:  {total_frames}")
    print(f"  Frames:     {n_frames}")
    print(f"{'='*60}\n")

    img_mat = sl.Mat()
    depth_mat = sl.Mat()
    rows = []
    t_total = time.perf_counter()

    for i, fidx in enumerate(frame_list):
        t0 = time.perf_counter()

        zed.set_svo_position(fidx)
        if zed.grab() != sl.ERROR_CODE.SUCCESS:
            print(f"  [SKIP] Frame {fidx}")
            continue

        # RGB
        zed.retrieve_image(img_mat, sl.VIEW.LEFT)
        bgr = img_mat.get_data()[:, :, :3].copy()
        bgr = cv2.cvtColor(bgr, cv2.COLOR_RGB2BGR)

        # Depth
        zed.retrieve_measure(depth_mat, sl.MEASURE.DEPTH)
        depth_full = depth_mat.get_data().copy()

        # ROI
        roi_bgr, rx, ry = extract_roi(bgr, ROI)
        roi_depth, _, _ = extract_roi(depth_full, ROI)
        bead_mask = make_bead_mask(roi_bgr)

        # bead 区域的 Z 值
        bead_z = roi_depth[bead_mask > 0]
        valid_z = bead_z[np.isfinite(bead_z) & (bead_z > Z_RANGE_MIN) & (bead_z < Z_RANGE_MAX)]

        if len(valid_z) < 50:
            print(f"  [{i+1}/{n_frames}] frame {fidx}: insufficient bead depth data ({len(valid_z)} pts)")
            continue

        # Analysis
        hist_norm, bin_centers, hist_peaks, peak_z = analyze_depth_histogram(valid_z)
        y_profile, y_steps = analyze_y_profile(roi_depth, bead_mask)

        t_proc = time.perf_counter() - t0

        # Vis
        canvas, clean = draw_vis(
            roi_bgr, roi_depth, bead_mask,
            hist_norm, bin_centers, hist_peaks, peak_z,
            y_profile, y_steps, fidx,
        )
        cv2.imwrite(str(out_dir / f"depth_{fidx:06d}.jpg"),
                    canvas, [cv2.IMWRITE_JPEG_QUALITY, 92])
        cv2.imwrite(str(out_dir / f"lines_{fidx:06d}.jpg"),
                    clean, [cv2.IMWRITE_JPEG_QUALITY, 92])

        rows.append({
            'frame': fidx,
            'bead_z_median': round(float(np.median(valid_z)), 4),
            'bead_z_min': round(float(valid_z.min()), 4),
            'bead_z_max': round(float(valid_z.max()), 4),
            'bead_z_std': round(float(valid_z.std()), 5),
            'z_range_mm': round(float((valid_z.max() - valid_z.min()) * 1000), 2),
            'hist_peaks': len(hist_peaks),
            'peak_z_values': ';'.join(f"{z:.4f}" for z in peak_z),
            'y_steps': len(y_steps),
            'proc_sec': round(t_proc, 3),
        })

        print(f"  [{i+1:4d}/{n_frames}]  frame {fidx:6d}  "
              f"Z={np.median(valid_z):.3f}m  range={valid_z.max()-valid_z.min():.4f}m  "
              f"hist_peaks={len(hist_peaks)}  y_steps={len(y_steps)}  "
              f"time={t_proc:.2f}s")

    t_elapsed = time.perf_counter() - t_total
    zed.close()

    # CSV
    csv_path = out_dir / "depth_layers.csv"
    with open(csv_path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=[
            "frame", "bead_z_median", "bead_z_min", "bead_z_max",
            "bead_z_std", "z_range_mm", "hist_peaks", "peak_z_values",
            "y_steps", "proc_sec",
        ])
        w.writeheader()
        w.writerows(rows)

    # Config
    with open(out_dir / "run_config.json", "w") as f:
        json.dump({
            "svo_path": str(SVO_PATH), "depth_mode": DEPTH_MODE,
            "roi": ROI, "frame_start": FRAME_START,
            "frame_end": FRAME_END, "frame_step": FRAME_STEP,
            "z_range": [Z_RANGE_MIN, Z_RANGE_MAX],
            "z_hist_bins": Z_HIST_BINS,
        }, f, indent=2)

    print(f"\n{'='*60}")
    print(f"  SUMMARY")
    print(f"{'='*60}")
    print(f"  Frames processed: {len(rows)}")
    print(f"  Total time:       {t_elapsed:.1f}s")
    if rows:
        z_ranges = [r['z_range_mm'] for r in rows]
        print(f"  Z range (bead):   {np.mean(z_ranges):.1f} ± {np.std(z_ranges):.1f} mm")
        hist_peaks_all = [r['hist_peaks'] for r in rows]
        print(f"  Hist peaks:       {np.mean(hist_peaks_all):.1f} avg")
    print(f"  Output:           {out_dir}")
    print(f"{'='*60}")


if __name__ == "__main__":
    main()
