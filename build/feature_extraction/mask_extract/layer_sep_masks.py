"""
layer_sep_masks.py — Layer separation via contour + edge analysis on SVO2
  从 SVO2 帧中提取 bead 区域，用边缘检测+轮廓分析识别层间分界线。
  不依赖 SAM2，纯 OpenCV，速度极快。

  输出: 每帧 layer 可视化 + 层数据 CSV, 存入 recorded_data 对应文件夹的 layer_sep_HHMMSS/ 下。

  核心思路:
    1. ROI 裁剪 → 灰度 → CLAHE 增强局部对比度
    2. 高斯模糊去噪 → Canny 边缘检测
    3. 水平方向 morphology 连接断裂的层间线
    4. 沿 Y 轴统计水平边缘密度 → 峰值 = 层间分界线
    5. 根据峰值间距输出层数、层高
"""

import sys, os, csv, cv2, time, json
import numpy as np
from pathlib import Path
from datetime import datetime
from scipy.signal import find_peaks

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

# ── SVO2 文件路径 ──
SVO_PATH = Path(
    r"C:\Users\888y9\Desktop\rsi_printing\recorded_data"
    r"\20260331_202433\recording_20260331_202433_001_20260331_202433.svo2"
)

# ── 帧范围 ──
FRAME_START = 150
FRAME_END   = 13300
FRAME_STEP  = 15

# ── ROI [x, y, w, h] — 聚焦 bead 区域, None = 全图 ──
ROI = [1200, 600, 400, 400]

# ── CLAHE 自适应直方图均衡 ──
CLAHE_CLIP   = 3.0        # 对比度限制, 越大增强越强 (1.0~5.0)
CLAHE_GRID   = (8, 8)     # 网格大小, 越小局部适应越强

# ── Bead 颜色 mask (HSV 阈值, 只在白色 bead 内部做边缘检测) ──
USE_BEAD_MASK     = True
BEAD_SAT_MAX      = 50      # 白色材料饱和度低 (0~255)
BEAD_VAL_MIN      = 140     # 白色材料亮度 (0~255), 降回包含 bead 侧面
BEAD_MORPH_KSIZE  = 11      # mask 形态学清理核大小, 增大让 bead mask 更完整
BEAD_MORPH_ITER   = 4       # 闭运算迭代次数, 保证侧面连通

# ── 高斯模糊 ──
BLUR_KSIZE   = 5           # 模糊核大小 (奇数), 越大去噪越强但丢细节 (3/5/7)

# ── Canny 边缘检测 ──
CANNY_LOW    = 60          # 低阈值, 提高减少弱边缘噪声 (20~80)
CANNY_HIGH   = 180         # 高阈值, 提高只保留强边缘 (60~200)

# ── 形态学：水平线连接 ──
MORPH_HLEN   = 40          # 水平结构元素长度, 增大强调水平连续边缘 (10~50)
MORPH_VLEN   = 1           # 垂直方向, 保持 1 只强化水平
MORPH_ITER   = 2           # 闭运算迭代, 增大连接更多断裂

# ── Y 轴边缘密度峰值检测 ──
PEAK_HEIGHT_RATIO = 0.18   # 峰值最小高度, 略降多检一些层 (0.05~0.3)
PEAK_DISTANCE     = 12     # 相邻层最小像素间距 (5~30)
PEAK_PROMINENCE   = 0.12   # 峰值突出度 (0.05~0.3)

# ── Sobel 水平梯度 (可选, 替代 Canny) ──
USE_SOBEL       = False     # True = 用 Sobel 水平梯度代替 Canny
SOBEL_KSIZE     = 3         # Sobel 核大小 (3/5/7)

# ── 输出控制 ──
SAVE_DEBUG_IMGS = True      # 保存中间过程图 (CLAHE, edge, profile)

# ═══════════════════════════════════════════════════════════════════════════════
#  END USER CONFIG
# ═══════════════════════════════════════════════════════════════════════════════


def build_output_dir(svo_path: Path) -> Path:
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    out = svo_path.parent / f"layer_sep_{ts}"
    out.mkdir(parents=True, exist_ok=True)
    return out


def open_svo(svo_path: Path):
    zed = sl.Camera()
    p = sl.InitParameters()
    p.set_from_svo_file(str(svo_path))
    p.svo_real_time_mode = False
    p.depth_mode = sl.DEPTH_MODE.NONE
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


def make_bead_mask(roi_bgr):
    """用 HSV 阈值提取白色 bead 区域的二值 mask"""
    hsv = cv2.cvtColor(roi_bgr, cv2.COLOR_BGR2HSV)
    # 白色: 低饱和度 + 高亮度
    mask = cv2.inRange(hsv,
                       np.array([0, 0, BEAD_VAL_MIN]),
                       np.array([180, BEAD_SAT_MAX, 255]))
    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,
                                  (BEAD_MORPH_KSIZE, BEAD_MORPH_KSIZE))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k, iterations=BEAD_MORPH_ITER)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k, iterations=1)
    return mask


def detect_layers(gray_roi, bead_mask=None):
    """
    从灰度 ROI 中检测层间分界线。
    方法: 在 bead mask 内部取中心纵向条带的平均亮度剖面,
          层间凹凸产生的亮度周期性波动 → 谷值 = 层间分界线。
    返回: (enhanced, edge, edge_h, density_norm, peaks)
    """
    H, W = gray_roi.shape

    # 1. CLAHE 增强局部对比度
    clahe = cv2.createCLAHE(clipLimit=CLAHE_CLIP, tileGridSize=CLAHE_GRID)
    enhanced = clahe.apply(gray_roi)

    # 2. 高斯模糊
    blurred = cv2.GaussianBlur(enhanced, (BLUR_KSIZE, BLUR_KSIZE), 0)

    # === 方法 A: 亮度剖面法 (主方法) ===
    # 在 bead 中心取纵向条带, 避开左右轮廓边缘
    if bead_mask is not None:
        # 对每行找 bead 的 x 范围, 取中间 40% 作为条带
        profile = np.zeros(H, dtype=np.float32)
        valid_rows = []
        for y in range(H):
            row_mask = bead_mask[y, :]
            cols = np.where(row_mask > 0)[0]
            if len(cols) < 10:
                continue
            x_min, x_max = cols[0], cols[-1]
            span = x_max - x_min
            cx1 = x_min + int(span * 0.3)
            cx2 = x_min + int(span * 0.7)
            if cx2 <= cx1:
                continue
            profile[y] = blurred[y, cx1:cx2].mean()
            valid_rows.append(y)

        if len(valid_rows) < 10:
            # fallback: 全图中心条带
            cx1 = W // 2 - W // 8
            cx2 = W // 2 + W // 8
            profile = blurred[:, cx1:cx2].mean(axis=1).astype(np.float32)
            valid_rows = list(range(H))
    else:
        cx1 = W // 2 - W // 8
        cx2 = W // 2 + W // 8
        profile = blurred[:, cx1:cx2].mean(axis=1).astype(np.float32)
        valid_rows = list(range(H))

    # 平滑剖面, 减少像素级噪声
    from scipy.ndimage import uniform_filter1d
    profile_smooth = uniform_filter1d(profile, size=3)

    # 归一化
    if profile_smooth.max() > profile_smooth.min():
        profile_norm = (profile_smooth - profile_smooth.min()) / \
                       (profile_smooth.max() - profile_smooth.min())
    else:
        profile_norm = np.zeros_like(profile_smooth)

    # 层间分界 = 亮度剖面的局部极小值 (凹陷 = 层间阴影)
    # 反转剖面, 找 peaks = 原始的 valleys
    inverted = 1.0 - profile_norm
    peaks, props = find_peaks(
        inverted,
        height=PEAK_HEIGHT_RATIO,
        distance=PEAK_DISTANCE,
        prominence=PEAK_PROMINENCE,
    )

    # 只保留 bead 区域内的峰值
    if bead_mask is not None and len(peaks) > 0:
        valid_set = set(valid_rows)
        peaks = np.array([p for p in peaks if p in valid_set])

    # === 方法 B: 边缘检测 (保留做 debug 可视化) ===
    if USE_SOBEL:
        sobel_y = cv2.Sobel(blurred, cv2.CV_64F, 0, 1, ksize=SOBEL_KSIZE)
        edge = np.abs(sobel_y).astype(np.uint8)
    else:
        edge = cv2.Canny(blurred, CANNY_LOW, CANNY_HIGH)

    if bead_mask is not None:
        edge = cv2.bitwise_and(edge, bead_mask)

    h_kernel = cv2.getStructuringElement(
        cv2.MORPH_RECT, (MORPH_HLEN, MORPH_VLEN)
    )
    edge_h = cv2.morphologyEx(edge, cv2.MORPH_CLOSE, h_kernel, iterations=MORPH_ITER)

    return enhanced, edge, edge_h, profile_norm, peaks


def draw_dashed_line(img, pt1, pt2, color, thickness=2, dash_len=10, gap_len=8):
    """在 img 上画水平虚线"""
    x1, y = pt1
    x2, _ = pt2
    x = x1
    while x < x2:
        cv2.line(img, (x, y), (min(x + dash_len, x2), y), color, thickness)
        x += dash_len + gap_len


def draw_layer_vis(roi_bgr, edge, edge_h, density_norm, peaks, fidx):
    """
    生成 4 合 1 可视化图:
      左上: 原图 + 红色虚线层线    右上: Canny/Sobel 边缘
      左下: 形态学后边缘           右下: Y 密度曲线 + 峰值
    """
    H, W = roi_bgr.shape[:2]
    canvas_h, canvas_w = H * 2, W * 2
    canvas = np.zeros((canvas_h, canvas_w, 3), dtype=np.uint8)

    # 左上: 原图 + 红色虚线层线
    panel_tl = roi_bgr.copy()
    for yi, py in enumerate(peaks):
        draw_dashed_line(panel_tl, (0, py), (W, py), (0, 0, 255), thickness=2)
        cv2.putText(panel_tl, f"L{yi}", (5, py - 4),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 255, 255), 1)
    cv2.putText(panel_tl, f"Frame {fidx} | {len(peaks)} layers",
                (5, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)
    canvas[0:H, 0:W] = panel_tl

    # 右上: 边缘图
    edge_bgr = cv2.cvtColor(edge, cv2.COLOR_GRAY2BGR)
    canvas[0:H, W:W*2] = edge_bgr

    # 左下: 形态学后边缘
    edge_h_bgr = cv2.cvtColor(edge_h, cv2.COLOR_GRAY2BGR)
    canvas[H:H*2, 0:W] = edge_h_bgr

    # 右下: 亮度剖面曲线 (绿 = 亮度, 红线 = 检测到的层间谷值)
    profile_img = np.zeros((H, W, 3), dtype=np.uint8)
    if density_norm.max() > 0:
        for y in range(H):
            x_val = int(density_norm[y] * (W - 20))
            cv2.line(profile_img, (0, y), (x_val, y), (0, 200, 0), 1)
        for py in peaks:
            cv2.line(profile_img, (0, py), (W, py), (0, 0, 255), 1)
            cv2.circle(profile_img, (int(density_norm[py] * (W - 20)), py),
                       3, (0, 255, 255), -1)
    # threshold line
    thr_x = int(PEAK_HEIGHT_RATIO * (W - 20))
    cv2.line(profile_img, (thr_x, 0), (thr_x, H), (100, 100, 100), 1)
    canvas[H:H*2, W:W*2] = profile_img

    # 单独输出: 干净原图 + 红色虚线（方便直接查看）
    clean_vis = roi_bgr.copy()
    for yi, py in enumerate(peaks):
        draw_dashed_line(clean_vis, (0, py), (W, py), (0, 0, 255), thickness=2)
        cv2.putText(clean_vis, f"L{yi}", (3, py - 4),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)
    cv2.putText(clean_vis, f"F{fidx} | {len(peaks)}L",
                (5, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)

    return canvas, clean_vis


def main():
    if not SVO_PATH.exists():
        print(f"[ERROR] SVO not found: {SVO_PATH}")
        sys.exit(1)

    out_dir = build_output_dir(SVO_PATH)

    print(f"{'='*60}")
    print(f"  Layer Separation — Contour/Edge Analysis")
    print(f"{'='*60}")
    print(f"  SVO:    {SVO_PATH.name}")
    print(f"  ROI:    {ROI if ROI else 'full frame'}")
    print(f"  Canny:  low={CANNY_LOW} high={CANNY_HIGH}")
    print(f"  CLAHE:  clip={CLAHE_CLIP} grid={CLAHE_GRID}")
    print(f"  Peak:   height_ratio={PEAK_HEIGHT_RATIO} dist={PEAK_DISTANCE}")
    print(f"  Output: {out_dir}")

    zed = open_svo(SVO_PATH)
    frame_list, total_frames = resolve_frame_range(
        zed, FRAME_START, FRAME_END, FRAME_STEP
    )
    n_frames = len(frame_list)

    res = zed.get_camera_information().camera_configuration.resolution
    print(f"  Resolution: {res.width} x {res.height}")
    print(f"  Total SVO frames: {total_frames}")
    print(f"  Frames to process: {n_frames}")
    print(f"{'='*60}\n")

    img_mat = sl.Mat()
    timing_rows = []
    layer_rows = []
    t_total_start = time.perf_counter()

    for i, fidx in enumerate(frame_list):
        t0 = time.perf_counter()

        zed.set_svo_position(fidx)
        if zed.grab() != sl.ERROR_CODE.SUCCESS:
            print(f"  [SKIP] Frame {fidx}: grab failed")
            continue

        zed.retrieve_image(img_mat, sl.VIEW.LEFT)
        bgr = img_mat.get_data()[:, :, :3].copy()
        bgr = cv2.cvtColor(bgr, cv2.COLOR_RGB2BGR)

        roi_bgr, rx, ry = extract_roi(bgr, ROI)
        gray = cv2.cvtColor(roi_bgr, cv2.COLOR_BGR2GRAY)

        # Bead color mask: 只在白色 bead 内部做边缘检测
        bead_mask = make_bead_mask(roi_bgr) if USE_BEAD_MASK else None

        # Layer detection
        enhanced, edge, edge_h, density, peaks = detect_layers(gray, bead_mask)

        t_proc = time.perf_counter() - t0

        # Vis
        vis, clean_vis = draw_layer_vis(roi_bgr, edge, edge_h, density, peaks, fidx)
        cv2.imwrite(str(out_dir / f"layers_{fidx:06d}.jpg"),
                    vis, [cv2.IMWRITE_JPEG_QUALITY, 92])
        cv2.imwrite(str(out_dir / f"lines_{fidx:06d}.jpg"),
                    clean_vis, [cv2.IMWRITE_JPEG_QUALITY, 92])

        if SAVE_DEBUG_IMGS:
            cv2.imwrite(str(out_dir / f"clahe_{fidx:06d}.jpg"), enhanced)
            if bead_mask is not None:
                cv2.imwrite(str(out_dir / f"bead_mask_{fidx:06d}.jpg"), bead_mask)

        # Compute layer heights (pixel distances between consecutive peaks)
        layer_heights = np.diff(peaks).tolist() if len(peaks) > 1 else []

        timing_rows.append({
            'frame': fidx,
            'n_layers': len(peaks),
            'layer_y_positions': peaks.tolist(),
            'layer_heights_px': layer_heights,
            'proc_sec': round(t_proc, 4),
        })

        layer_rows.append({
            'frame': fidx,
            'n_layers': len(peaks),
            'peak_ys': ';'.join(str(y) for y in peaks),
            'layer_heights': ';'.join(str(h) for h in layer_heights),
            'mean_height_px': round(np.mean(layer_heights), 1) if layer_heights else 0,
            'proc_sec': round(t_proc, 4),
        })

        print(f"  [{i+1:4d}/{n_frames}]  frame {fidx:6d}  "
              f"layers={len(peaks):2d}  "
              f"heights={layer_heights}  "
              f"time={t_proc:.3f}s")

    t_total = time.perf_counter() - t_total_start
    zed.close()

    # Save CSV
    csv_path = out_dir / "layer_data.csv"
    with open(csv_path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=[
            "frame", "n_layers", "peak_ys", "layer_heights",
            "mean_height_px", "proc_sec",
        ])
        w.writeheader()
        w.writerows(layer_rows)

    # Save config
    config = {
        "svo_path": str(SVO_PATH),
        "frame_start": FRAME_START, "frame_end": FRAME_END,
        "frame_step": FRAME_STEP, "roi": ROI,
        "clahe_clip": CLAHE_CLIP, "clahe_grid": CLAHE_GRID,
        "blur_ksize": BLUR_KSIZE,
        "canny_low": CANNY_LOW, "canny_high": CANNY_HIGH,
        "use_sobel": USE_SOBEL, "sobel_ksize": SOBEL_KSIZE,
        "morph_hlen": MORPH_HLEN, "morph_vlen": MORPH_VLEN,
        "morph_iter": MORPH_ITER,
        "peak_height_ratio": PEAK_HEIGHT_RATIO,
        "peak_distance": PEAK_DISTANCE,
        "peak_prominence": PEAK_PROMINENCE,
        "total_svo_frames": total_frames,
        "frames_processed": len(timing_rows),
    }
    with open(out_dir / "run_config.json", "w") as f:
        json.dump(config, f, indent=2, ensure_ascii=False)

    # Summary
    print(f"\n{'='*60}")
    print(f"  SUMMARY")
    print(f"{'='*60}")
    print(f"  Total frames: {len(timing_rows)}")
    print(f"  Total time:   {t_total:.2f}s")
    if timing_rows:
        proc_times = [r['proc_sec'] for r in timing_rows]
        n_layers_all = [r['n_layers'] for r in timing_rows]
        print(f"  Avg proc/frame: {np.mean(proc_times):.3f}s")
        print(f"  Layers: min={min(n_layers_all)} max={max(n_layers_all)} "
              f"avg={np.mean(n_layers_all):.1f}")
    print(f"  Output: {out_dir}")
    print(f"{'='*60}")


if __name__ == "__main__":
    main()
