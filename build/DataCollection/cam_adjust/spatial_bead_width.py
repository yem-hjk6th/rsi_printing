"""
spatial_bead_width.py — 按机械臂行进距离采样, Sobel Y 测宽 + 邻帧连续性验证

思路:
  1. RSI CSV → 累积行进距离 (mm)
  2. 每隔 STEP_MM (1mm) 取最近的 RSI 行 → timestamp → SVO 帧号
  3. 每帧 Sobel Y 测宽 (已校准: ksize=5, threshold=20)
  4. 每个点和前后各 2 帧比较 → |Δ| > JUMP_THRESH_MM 则标记异常

输出:
  CSV: distance_mm, frame, x, y, z, vel, width_px, width_mm, d_prev1, d_prev2, d_next1, d_next2, flag
  flag: OK / JUMP / MISS
"""

import csv, cv2, time, numpy as np
import pyzed.sl as sl
from pathlib import Path
from datetime import datetime

# ─── Config ──────────────────────────────────────────
SVO_PATH = r"C:\Users\dell\Desktop\RSI\recorded_data\20260310_183338\recording_20260310_183338.svo2"
RSI_PATH = r"C:\Users\dell\Desktop\RSI\rsi_data\rsi_data_20260310_183338.csv"
OUT_BASE = Path(__file__).resolve().parent.parent / "labeling" / "post_labeling" / "spatial"

STEP_MM = 1.0          # 每 1mm 采样一次
NEIGHBOR = 2           # 前后各比对 2 帧
JUMP_THRESH_MM = 0.5   # |Δ| > 0.5mm 标记 JUMP

ROI_SIZE = 224
ROI_DX = -30
ROI_DY = 350

# Sobel (校准最优)
SOB_KSIZE = 5
SOB_THRESH = 20

# 宽度测量
WIDTH_MIN = 2
WIDTH_MAX = 40
N_SCANLINES = 30
CONE_RATIO = 0.55
SCAR_ZONE_RATIO = 0.40


# ─── Functions ───────────────────────────────────────

def find_nozzle(bgr):
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, np.array([100, 100, 80]), np.array([130, 255, 255]))
    k = cv2.getStructuringElement(cv2.MORPH_RECT, (7, 7))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k, iterations=3)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k, iterations=2)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    valid = [(c, cv2.contourArea(c)) for c in contours if 1000 < cv2.contourArea(c) < 300000]
    if not valid:
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
    H, W = img.shape[:2]
    half = size // 2
    x1 = max(0, min(cx - half, W - size))
    y1 = max(0, min(cy - half, H - size))
    return img[y1:y1+size, x1:x1+size].copy(), x1, y1


def sobel_width(gray_roi):
    scar_y = int(ROI_SIZE * SCAR_ZONE_RATIO)
    blur = cv2.GaussianBlur(gray_roi, (3, 3), 0)
    sobel = cv2.Sobel(blur, cv2.CV_64F, 0, 1, ksize=SOB_KSIZE)
    abs_sobel = np.abs(sobel)
    abs_sobel[:scar_y, :] = 0
    mask = (abs_sobel > SOB_THRESH).astype(np.uint8) * 255
    k = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 5))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k, iterations=2)
    ys, xs = np.where(mask > 0)
    if len(xs) < 10:
        return None
    x_span = xs.max() - xs.min()
    y_span = ys.max() - ys.min()
    raw = []
    if x_span >= y_span:
        for sx in np.linspace(xs.min()+2, xs.max()-2, N_SCANLINES).astype(int):
            col = sobel[scar_y:, sx]
            pos = np.where(col > SOB_THRESH)[0]
            neg = np.where(col < -SOB_THRESH)[0]
            if len(pos) > 0 and len(neg) > 0:
                top = pos[0]
                bot_c = neg[neg > top]
                if len(bot_c) > 0:
                    w = bot_c[0] - top
                    if WIDTH_MIN <= w <= WIDTH_MAX:
                        raw.append(w)
    else:
        for sy in np.linspace(ys.min()+2, ys.max()-2, N_SCANLINES).astype(int):
            row = cv2.Sobel(blur, cv2.CV_64F, 1, 0, ksize=SOB_KSIZE)[sy, :]
            pos = np.where(row > SOB_THRESH)[0]
            neg = np.where(row < -SOB_THRESH)[0]
            if len(pos) > 0 and len(neg) > 0:
                left = pos[0]
                right_c = neg[neg > left]
                if len(right_c) > 0:
                    w = right_c[0] - left
                    if WIDTH_MIN <= w <= WIDTH_MAX:
                        raw.append(w)
    if not raw:
        return None
    med = np.median(raw)
    filtered = [w for w in raw if w >= med * CONE_RATIO]
    return np.mean(filtered) if filtered else None


# ─── Main ────────────────────────────────────────────

def main():
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    out_dir = OUT_BASE / timestamp
    out_dir.mkdir(parents=True, exist_ok=True)

    # ── 1. 读 RSI CSV, 算累积距离 ──
    print("Loading RSI data...")
    rsi_rows = list(csv.DictReader(open(RSI_PATH)))
    ts_arr = np.array([float(r['timestamp']) for r in rsi_rows])
    x_arr = np.array([float(r['x_mm']) for r in rsi_rows])
    y_arr = np.array([float(r['y_mm']) for r in rsi_rows])
    z_arr = np.array([float(r['z_mm']) for r in rsi_rows])
    vel_arr = np.array([int(r['vel']) for r in rsi_rows])

    ds = np.sqrt(np.diff(x_arr)**2 + np.diff(y_arr)**2 + np.diff(z_arr)**2)
    cum_dist = np.concatenate([[0], np.cumsum(ds)])

    # 只看运动段
    moving = np.where(vel_arr > 0)[0]
    if len(moving) == 0:
        print("No moving data"); return
    i_start, i_end = moving[0], moving[-1]
    d_start = cum_dist[i_start]
    d_end = cum_dist[i_end]
    total_travel = d_end - d_start
    print(f"  Travel: {total_travel:.1f}mm (RSI rows {i_start}–{i_end})")

    # ── 2. 每 STEP_MM 找对应 RSI 行 → timestamp ──
    sample_dists = np.arange(d_start, d_end, STEP_MM)
    sample_indices = np.searchsorted(cum_dist, sample_dists, side='left')
    sample_indices = np.clip(sample_indices, 0, len(rsi_rows) - 1)
    print(f"  Sample points: {len(sample_dists)} (every {STEP_MM}mm)")

    # ── 3. 打开 SVO, 构建 timestamp → frame 映射 ──
    print("Opening SVO...")
    zed = sl.Camera()
    p = sl.InitParameters()
    p.set_from_svo_file(SVO_PATH)
    p.svo_real_time_mode = False
    p.depth_mode = sl.DEPTH_MODE.NONE
    if zed.open(p) != sl.ERROR_CODE.SUCCESS:
        print("ZED open failed"); return

    calib = zed.get_camera_information().camera_configuration.calibration_parameters
    fx = calib.left_cam.fx
    FIXED_Z = 0.447  # 固定深度 (全程 0.432-0.455m, 变化极小)
    mm_per_px = FIXED_Z * 1000.0 / fx
    total_frames = zed.get_svo_number_of_frames()
    MAX_FRAME = total_frames - 50  # 避免触及 SVO 尾部

    # 建 SVO timestamp 表 (用锚点做线性插值, 避开尾部)
    anchors = np.linspace(0, MAX_FRAME, 20).astype(int)
    anchor_ts = []
    print("  Building timestamp map...", end="", flush=True)
    for fi in anchors:
        zed.set_svo_position(fi)
        zed.grab()
        t = zed.get_timestamp(sl.TIME_REFERENCE.IMAGE).get_nanoseconds() / 1e9
        anchor_ts.append(t)
    anchor_ts = np.array(anchor_ts)
    print(" Done")

    def ts_to_frame(t):
        return int(np.round(np.interp(t, anchor_ts, anchors)))

    # RSI sample → SVO frame
    sample_frames = []
    for idx in sample_indices:
        f = ts_to_frame(ts_arr[idx])
        f = max(0, min(f, MAX_FRAME))
        sample_frames.append(f)
    sample_frames = np.array(sample_frames)

    # 去重: 同一 SVO 帧只测一次
    unique_frames = sorted(set(sample_frames))
    print(f"  Unique SVO frames to process: {len(unique_frames)}")

    # ── 4. 批量 Sobel 测宽 ──
    print(f"Running Sobel Y (fixed z={FIXED_Z}m, mm/px={mm_per_px:.4f})...", flush=True)
    img_mat = sl.Mat()
    frame_results = {}  # frame → (width_px, width_mm)

    t0 = time.perf_counter()
    for idx_u, fi in enumerate(unique_frames):
        if (idx_u + 1) % 100 == 0 or idx_u == 0 or idx_u == len(unique_frames) - 1:
            elapsed = time.perf_counter() - t0
            print(f"  [{idx_u+1}/{len(unique_frames)}] frame {fi} ({elapsed:.1f}s)", flush=True)
        zed.set_svo_position(fi)
        err = zed.grab()
        if err != sl.ERROR_CODE.SUCCESS:
            frame_results[fi] = (None, None)
            continue

        zed.retrieve_image(img_mat, sl.VIEW.LEFT)
        bgr = cv2.cvtColor(img_mat.get_data()[:, :, :3], cv2.COLOR_RGB2BGR)

        nozzle = find_nozzle(bgr)
        if nozzle is None:
            H, W = bgr.shape[:2]
            nozzle = (W // 2, H // 2)

        roi_cx = nozzle[0] + ROI_DX
        roi_cy = nozzle[1] + ROI_DY
        bgr_roi, x1, y1 = crop_roi(bgr, roi_cx, roi_cy, ROI_SIZE)
        gray_roi = cv2.cvtColor(bgr_roi, cv2.COLOR_BGR2GRAY)

        w_px = sobel_width(gray_roi)
        w_mm = (w_px * mm_per_px) if w_px else None
        frame_results[fi] = (w_px, w_mm)

    dt = time.perf_counter() - t0
    zed.close()
    print(f"  Done: {len(unique_frames)} frames in {dt:.1f}s "
          f"({dt/len(unique_frames)*1000:.1f}ms/frame)")

    # ── 5. 组装结果 + 邻帧比对 ──
    records = []
    for i, (si, fi) in enumerate(zip(sample_indices, sample_frames)):
        w_px, w_mm = frame_results.get(fi, (None, None))
        records.append({
            'i': i,
            'dist_mm': round(sample_dists[i] - d_start, 2),
            'frame': fi,
            'rsi_idx': int(si),
            'x': round(x_arr[si], 2),
            'y': round(y_arr[si], 2),
            'z': round(z_arr[si], 2),
            'vel': int(vel_arr[si]),
            'width_px': round(w_px, 2) if w_px else '',
            'width_mm': round(w_mm, 3) if w_mm else '',
        })

    # 邻帧 delta
    widths = [r['width_mm'] if r['width_mm'] != '' else None for r in records]
    for i, rec in enumerate(records):
        deltas = {}
        for offset in [-2, -1, 1, 2]:
            j = i + offset
            key = f'd_{"prev" if offset < 0 else "next"}{abs(offset)}'
            if 0 <= j < len(widths) and widths[i] is not None and widths[j] is not None:
                deltas[key] = round(widths[i] - widths[j], 3)
            else:
                deltas[key] = ''
        rec.update(deltas)

        # flag
        if rec['width_mm'] == '':
            rec['flag'] = 'MISS'
        else:
            jumps = [abs(v) for v in deltas.values() if v != '' and abs(v) > JUMP_THRESH_MM]
            rec['flag'] = 'JUMP' if jumps else 'OK'

    # ── 6. 输出 CSV ──
    fields = ['dist_mm', 'frame', 'x', 'y', 'z', 'vel',
              'width_px', 'width_mm', 'd_prev2', 'd_prev1', 'd_next1', 'd_next2', 'flag']
    csv_path = out_dir / "spatial_bead_width.csv"
    with open(csv_path, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=fields, extrasaction='ignore')
        w.writeheader()
        w.writerows(records)

    # ── 7. 统计 ──
    ok = [r for r in records if r['flag'] == 'OK']
    jump = [r for r in records if r['flag'] == 'JUMP']
    miss = [r for r in records if r['flag'] == 'MISS']
    widths_valid = [r['width_mm'] for r in records if r['width_mm'] != '']

    print(f"\n{'='*60}")
    print("SPATIAL BEAD WIDTH REPORT")
    print(f"{'='*60}")
    print(f"  Total samples: {len(records)} (every {STEP_MM}mm)")
    print(f"  Travel: {total_travel:.1f}mm")
    if widths_valid:
        arr = np.array(widths_valid)
        print(f"  Width: {np.mean(arr):.2f} ± {np.std(arr):.2f} mm  "
              f"(range {np.min(arr):.2f}–{np.max(arr):.2f})")
    print(f"  OK: {len(ok)}  JUMP: {len(jump)}  MISS: {len(miss)}")
    if jump:
        print(f"  Jump locations (dist_mm):")
        for r in jump[:20]:
            print(f"    d={r['dist_mm']}mm f{r['frame']}: {r['width_mm']}mm "
                  f"Δ=[{r['d_prev1']}, {r['d_next1']}]")
        if len(jump) > 20:
            print(f"    ... and {len(jump)-20} more")
    print(f"\n  Output: {csv_path}")


if __name__ == "__main__":
    main()
