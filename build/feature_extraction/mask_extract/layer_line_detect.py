"""
layer_line_detect.py — 近距离特写图像的层间线分割
  各向异性高斯 (强 X 平滑) → 逐列谷值检测 → 跨列连接 → 强平滑 polyline

  用法: python layer_line_detect.py <image_path>
"""

import sys, cv2
import numpy as np
from pathlib import Path
from scipy.signal import find_peaks
from scipy.ndimage import gaussian_filter, gaussian_filter1d

# ═══════════════════════════════════════════════════════════════════════════════
#  CONFIG
# ═══════════════════════════════════════════════════════════════════════════════

DEFAULT_IMG = None

# ROI [x, y, w, h] — None = 全图
ROI = [0, 420, 472, 575]

# CLAHE
CLAHE_CLIP = 3.0
CLAHE_GRID = (8, 8)

# 各向异性高斯
SIGMA_X = 40            # X 方向强平滑 (越大 → 逐列检测越一致)
SIGMA_Y = 1.5           # Y 方向弱平滑

# 逐列谷值检测
COL_STEP = 2            # 列步长
MIN_LAYER_PX = 18       # 最小层间距 (px)
PEAK_PROMINENCE = 0.03  # 谷值突出度 (小 → 检出更多)

# 曲线连接
LINK_MAX_DY = 8         # 相邻列最大 Y 容差
MIN_TRACK_SPAN = 0.25   # track 最小宽度占比
MIN_TRACK_PTS = 25      # track 最少点数

# track 平滑
TRACK_SMOOTH_SIGMA = 15.0  # 最终 polyline 平滑 (大值 → 非常光滑)

# 可视化
LINE_THICK = 2


# ═══════════════════════════════════════════════════════════════════════════════
#  PROCESSING
# ═══════════════════════════════════════════════════════════════════════════════

def preprocess(gray_roi):
    """CLAHE → 各向异性高斯."""
    clahe = cv2.createCLAHE(clipLimit=CLAHE_CLIP, tileGridSize=CLAHE_GRID)
    enhanced = clahe.apply(gray_roi)
    return gaussian_filter(enhanced.astype(np.float64), sigma=(SIGMA_Y, SIGMA_X))


def detect_valleys_per_column(smoothed):
    """逐列检测灰度谷值 (暗缝 = 层间线)."""
    h, w = smoothed.shape
    col_peaks = {}
    for col in range(0, w, COL_STEP):
        profile = smoothed[:, col]
        inv = profile.max() - profile
        pmax = inv.max()
        if pmax <= 0:
            continue
        inv = inv / pmax
        peaks, _ = find_peaks(inv, distance=MIN_LAYER_PX,
                              prominence=PEAK_PROMINENCE)
        if len(peaks) > 0:
            col_peaks[col] = sorted(peaks.tolist())
    return col_peaks


def link_and_smooth(col_peaks):
    """跨列贪心连接 → 过滤 → 平滑."""
    cols = sorted(col_peaks.keys())
    if not cols:
        return []

    tracks = []
    for y in col_peaks[cols[0]]:
        tracks.append([(cols[0], y)])

    for ci in range(1, len(cols)):
        col = cols[ci]
        py_list = col_peaks[col][:]
        used = [False] * len(py_list)
        for track in tracks:
            _, last_y = track[-1]
            best_i, best_d = -1, LINK_MAX_DY + 1
            for pi, py in enumerate(py_list):
                if used[pi]:
                    continue
                d = abs(py - last_y)
                if d < best_d:
                    best_d = d
                    best_i = pi
            if best_i >= 0 and best_d <= LINK_MAX_DY:
                track.append((col, py_list[best_i]))
                used[best_i] = True
        for pi, py in enumerate(py_list):
            if not used[pi]:
                tracks.append([(col, py)])

    # 过滤短 track
    w_total = cols[-1] - cols[0]
    tracks = [t for t in tracks
              if w_total > 0 and (t[-1][0] - t[0][0]) / w_total >= MIN_TRACK_SPAN
              and len(t) >= MIN_TRACK_PTS]

    # 平滑
    for i, track in enumerate(tracks):
        xs = np.array([p[0] for p in track])
        ys = np.array([p[1] for p in track])
        ys_s = gaussian_filter1d(ys.astype(float), sigma=TRACK_SMOOTH_SIGMA)
        tracks[i] = list(zip(xs.tolist(), np.round(ys_s).astype(int).tolist()))

    tracks.sort(key=lambda t: np.mean([p[1] for p in t]))

    # 拼接断裂 track (Y 相近, x 不重叠的碎片)
    changed = True
    while changed:
        changed = False
        new_tracks = []
        used = set()
        for i in range(len(tracks)):
            if i in used:
                continue
            ti = tracks[i]
            for j in range(i + 1, len(tracks)):
                if j in used:
                    continue
                tj = tracks[j]
                yi = np.mean([p[1] for p in ti])
                yj = np.mean([p[1] for p in tj])
                if abs(yi - yj) > MIN_LAYER_PX:
                    continue
                # 判断端点能否拼接
                dy_join = min(abs(ti[-1][1] - tj[0][1]),
                              abs(tj[-1][1] - ti[0][1]))
                if dy_join > LINK_MAX_DY * 3:
                    continue
                # x overlap 检查
                xi = {p[0] for p in ti}
                xj = {p[0] for p in tj}
                if len(xi & xj) > min(len(ti), len(tj)) * 0.5:
                    continue
                # 合并
                merged = {}
                for x, y in ti:
                    merged[x] = y
                for x, y in tj:
                    merged[x] = merged.get(x, 0) + y if x in merged else y
                    if x in {p[0] for p in ti}:
                        merged[x] = (merged[x]) // 2
                ti = [(x, merged[x]) for x in sorted(merged.keys())]
                # 重新平滑
                xs = np.array([p[0] for p in ti])
                ys = np.array([p[1] for p in ti])
                ys_s = gaussian_filter1d(ys.astype(float), sigma=TRACK_SMOOTH_SIGMA)
                ti = list(zip(xs.tolist(), np.round(ys_s).astype(int).tolist()))
                used.add(j)
                changed = True
            new_tracks.append(ti)
            used.add(i)
        tracks = new_tracks
        tracks.sort(key=lambda t: np.mean([p[1] for p in t]))

    # 去重: 短 track 如果 x 范围被长 track 包含且 Y 相近, 删除短的
    dedup = []
    for i, ti in enumerate(tracks):
        yi = np.mean([p[1] for p in ti])
        xi_min, xi_max = ti[0][0], ti[-1][0]
        absorbed = False
        for j, tj in enumerate(tracks):
            if i == j or len(tj) <= len(ti):
                continue
            yj = np.mean([p[1] for p in tj])
            xj_min, xj_max = tj[0][0], tj[-1][0]
            if abs(yi - yj) < MIN_LAYER_PX * 0.8 and xi_min >= xj_min and xi_max <= xj_max:
                absorbed = True
                break
        if not absorbed:
            dedup.append(ti)
    tracks = dedup

    return tracks


def gap_fill(curves, smoothed):
    """在 Y 间距过大的位置用更低门槛检测弱谷值并补线."""
    if len(curves) < 2:
        return curves
    h, w = smoothed.shape
    ymeans = [np.mean([p[1] for p in c]) for c in curves]
    gaps = [ymeans[i + 1] - ymeans[i] for i in range(len(ymeans) - 1)]
    avg_gap = np.median(gaps)

    new_curves = list(curves)
    for gi, gap in enumerate(gaps):
        if gap < avg_gap * 1.3:
            continue
        y_lo = int(ymeans[gi]) + 3
        y_hi = int(ymeans[gi + 1]) - 3
        if y_hi - y_lo < 5:
            continue
        # 在 [y_lo, y_hi] 带状区域中搜索弱谷值
        col_peaks = {}
        for col in range(0, w, COL_STEP):
            profile = smoothed[y_lo:y_hi, col]
            if len(profile) < 5:
                continue
            inv = profile.max() - profile
            pmax = inv.max()
            if pmax <= 0:
                continue
            inv = inv / pmax
            peaks, props = find_peaks(inv, distance=5,
                                      prominence=PEAK_PROMINENCE * 0.4)
            if len(peaks) > 0:
                # 取最显著的
                best = peaks[np.argmax(props["prominences"])]
                col_peaks[col] = best + y_lo

        if len(col_peaks) < 15:
            print(f"  gap_fill: gap {gi} ({gap:.0f}px) only {len(col_peaks)} pts, skip")
            continue

        # 组装并平滑
        xs = np.array(sorted(col_peaks.keys()))
        ys = np.array([col_peaks[x] for x in xs])
        ys_s = gaussian_filter1d(ys.astype(float), sigma=TRACK_SMOOTH_SIGMA)
        new_curve = list(zip(xs.tolist(), np.round(ys_s).astype(int).tolist()))
        new_curves.append(new_curve)
        print(f"  gap_fill: filled gap {gi} ({gap:.0f}px) with {len(new_curve)} pts "
              f"around Y={int(np.mean(ys_s))}")

    new_curves.sort(key=lambda c: np.mean([p[1] for p in c]))
    return new_curves


# ═══════════════════════════════════════════════════════════════════════════════
#  VISUALIZATION
# ═══════════════════════════════════════════════════════════════════════════════

def visualize(bgr, curves, save_dir=None):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    vis = bgr.copy()
    colors = plt.colormaps.get_cmap("tab20")

    for ci, curve in enumerate(curves):
        c = colors(ci % 20)[:3]
        bgr_c = (int(c[2] * 255), int(c[1] * 255), int(c[0] * 255))
        pts = np.array([[x, y] for x, y in curve], dtype=np.int32)
        cv2.polylines(vis, [pts], isClosed=False, color=bgr_c,
                      thickness=LINE_THICK, lineType=cv2.LINE_AA)

    cv2.putText(vis, f"{len(curves)} layer lines",
                (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 8))
    ax1.imshow(cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB))
    ax1.set_title("Original (ROI)")
    ax1.axis("off")
    ax2.imshow(cv2.cvtColor(vis, cv2.COLOR_BGR2RGB))
    ax2.set_title(f"{len(curves)} layer curves")
    ax2.axis("off")
    plt.tight_layout()
    if save_dir:
        fig.savefig(str(save_dir), dpi=150, bbox_inches="tight")
        print(f"Saved profile: {save_dir}")
    plt.close(fig)
    return vis


def main():
    if len(sys.argv) > 1:
        img_path = Path(sys.argv[1])
    elif DEFAULT_IMG:
        img_path = Path(DEFAULT_IMG)
    else:
        print("Usage: python layer_line_detect.py <image_path>")
        sys.exit(1)

    if not img_path.exists():
        print(f"[ERROR] File not found: {img_path}")
        sys.exit(1)

    bgr = cv2.imread(str(img_path))
    if bgr is None:
        print(f"[ERROR] Cannot read image: {img_path}")
        sys.exit(1)

    print(f"Image: {img_path.name}  size: {bgr.shape[1]}x{bgr.shape[0]}")

    if ROI:
        x, y, w, h = ROI
        roi_bgr = bgr[y:y + h, x:x + w].copy()
    else:
        roi_bgr = bgr

    gray = cv2.cvtColor(roi_bgr, cv2.COLOR_BGR2GRAY)
    smoothed = preprocess(gray)
    col_peaks = detect_valleys_per_column(smoothed)
    total = sum(len(v) for v in col_peaks.values())
    print(f"Per-column valleys: {total} pts across {len(col_peaks)} cols")

    curves = link_and_smooth(col_peaks)

    # gap-fill: 在大间距处用更低门槛重新搜索
    curves = gap_fill(curves, smoothed)

    print(f"Result: {len(curves)} layer curves")
    for i, c in enumerate(curves):
        ys = [p[1] for p in c]
        print(f"  {i:2d}: {len(c):3d} pts, Y=[{min(ys)}-{max(ys)}], "
              f"x=[{c[0][0]}-{c[-1][0]}]")

    out_dir = img_path.parent
    vis = visualize(roi_bgr, curves,
                    save_dir=out_dir / f"{img_path.stem}_profile.png")
    out_path = out_dir / f"{img_path.stem}_layers{img_path.suffix}"
    cv2.imwrite(str(out_path), vis)
    print(f"Saved: {out_path}")


if __name__ == "__main__":
    main()
