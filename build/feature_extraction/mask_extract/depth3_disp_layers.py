"""
depth3_disp_layers.py — FFS disparity 域层分离分析
  直接在 disparity (像素) 空间做层检测, 不转 depth, 避免绝对值偏差问题。
  disparity 越大 = 越近 = 越上层
"""

import os, sys, csv, time, json
import cv2
import numpy as np
import torch
from pathlib import Path
from datetime import datetime
from scipy.signal import find_peaks
from scipy.ndimage import uniform_filter1d

# ── ZED SDK DLL (Windows) ──
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

# ── Fast-FoundationStereo ──
FFS_REPO = Path(r"C:\Users\888y9\Desktop\Repo\Fast-FoundationStereo")
sys.path.insert(0, str(FFS_REPO))

import yaml
from core.utils.utils import InputPadder
from Utils import AMP_DTYPE, vis_disparity

# ═══════════════════════════════════════════════════════════════════════════════
#  USER CONFIG
# ═══════════════════════════════════════════════════════════════════════════════

SVO_PATH = Path(
    r"C:\Users\888y9\Desktop\rsi_printing\recorded_data"
    r"\20260331_202433\recording_20260331_202433_001_20260331_202433.svo2"
)

# ── 帧范围 (少量帧, 重点分析) ──
FRAME_START = 150
FRAME_END   = 11750
FRAME_STEP  = 500           # 粗采样 ~23 帧, 每帧 ~0.5s

# ── ROI [x, y, w, h] ──
ROI = [1200, 600, 400, 400]

# ── FFS ──
MODEL_NAME  = "23-36-37"
VALID_ITERS = 8
MAX_DISP    = 192
SCALE       = 1.0

# ── Bead 颜色 mask ──
BEAD_SAT_MAX     = 50
BEAD_VAL_MIN     = 140
BEAD_MORPH_KSIZE = 11
BEAD_MORPH_ITER  = 4

# ── Disparity 分层参数 ──
DISP_HIST_BINS     = 300
DISP_PEAK_DISTANCE = 3      # bin 间距
DISP_PEAK_PROMINENCE = 0.04
DISP_SMOOTH_SIZE   = 5

# ── Y 轴 disparity 剖面参数 ──
Y_SMOOTH_SIZE      = 7
Y_STEP_THRESH_PX   = 0.8    # disparity 阶梯跳变阈值 (像素)
Y_STEP_MERGE       = 8      # 合并距离 (像素行)

# ═══════════════════════════════════════════════════════════════════════════════


def build_output_dir(svo_path: Path) -> Path:
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    out = svo_path.parent / f"disp_layers_{ts}"
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


def load_ffs(model_name, valid_iters, max_disp):
    weight_dir = FFS_REPO / "weights" / model_name
    model_path = weight_dir / "model_best_bp2_serialize.pth"
    cfg_path = weight_dir / "cfg.yaml"
    if not model_path.exists():
        print(f"[ERROR] Model not found: {model_path}")
        sys.exit(1)
    model = torch.load(str(model_path), map_location="cpu", weights_only=False)
    model.args.valid_iters = valid_iters
    model.args.max_disp = max_disp
    model.cuda().eval()
    return model


def run_ffs(model, img_l_rgb, img_r_rgb, valid_iters, scale, cache):
    if scale != 1.0:
        img_l_rgb = cv2.resize(img_l_rgb, fx=scale, fy=scale, dsize=None)
        img_r_rgb = cv2.resize(img_r_rgb, dsize=(img_l_rgb.shape[1], img_l_rgb.shape[0]))
    H, W = img_l_rgb.shape[:2]
    t0 = torch.as_tensor(img_l_rgb).cuda().float()[None].permute(0, 3, 1, 2)
    t1 = torch.as_tensor(img_r_rgb).cuda().float()[None].permute(0, 3, 1, 2)
    if cache.get("shape") != t0.shape:
        cache["padder"] = InputPadder(t0.shape, divis_by=32, force_square=False)
        cache["shape"] = t0.shape
    t0p, t1p = cache["padder"].pad(t0, t1)
    with torch.amp.autocast("cuda", enabled=True, dtype=AMP_DTYPE):
        disp = model.forward(t0p, t1p, iters=valid_iters, test_mode=True,
                             optimize_build_volume="pytorch1")
    disp = cache["padder"].unpad(disp.float())
    return disp.data.cpu().numpy().reshape(H, W).clip(0, None)


def extract_roi(arr, roi):
    x, y, w, h = roi
    H, W = arr.shape[:2]
    x, y = max(0, min(x, W-1)), max(0, min(y, H-1))
    w, h = min(w, W-x), min(h, H-y)
    return arr[y:y+h, x:x+w].copy()


def make_bead_mask(roi_bgr):
    hsv = cv2.cvtColor(roi_bgr, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, np.array([0, 0, BEAD_VAL_MIN]),
                       np.array([180, BEAD_SAT_MAX, 255]))
    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,
                                  (BEAD_MORPH_KSIZE, BEAD_MORPH_KSIZE))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k, iterations=BEAD_MORPH_ITER)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k, iterations=1)
    return mask


def analyze_disp_histogram(disp_vals):
    """Disparity 直方图找峰"""
    d_min, d_max = disp_vals.min(), disp_vals.max()
    if d_max - d_min < 1:
        return np.array([]), np.array([]), np.array([]), np.array([])

    hist, edges = np.histogram(disp_vals, bins=DISP_HIST_BINS,
                               range=(d_min, d_max))
    centers = (edges[:-1] + edges[1:]) / 2
    hist_s = uniform_filter1d(hist.astype(np.float64), size=DISP_SMOOTH_SIZE)
    if hist_s.max() > 0:
        hist_n = hist_s / hist_s.max()
    else:
        return np.array([]), centers, np.array([]), np.array([])

    peaks, _ = find_peaks(hist_n, distance=DISP_PEAK_DISTANCE,
                          prominence=DISP_PEAK_PROMINENCE)
    return hist_n, centers, peaks, centers[peaks]


def analyze_y_disp_profile(disp_roi, bead_mask):
    """沿 Y 轴的 disparity 中值曲线 → 找阶梯"""
    H, W = disp_roi.shape
    y_vals = np.full(H, np.nan)
    for y in range(H):
        cols = np.where(bead_mask[y] > 0)[0]
        if len(cols) < 5:
            continue
        d = disp_roi[y, cols]
        valid = d[d > 1]
        if len(valid) > 3:
            y_vals[y] = np.median(valid)

    vi = np.where(np.isfinite(y_vals))[0]
    if len(vi) < 10:
        return y_vals, np.array([])

    y_interp = np.interp(np.arange(H), vi, y_vals[vi])
    y_smooth = uniform_filter1d(y_interp, size=Y_SMOOTH_SIZE)

    ddy = np.diff(y_smooth)
    # bead 从上到下: 越下面越远 → disparity 越小 → diff 多为负
    # 层界面处 disparity 跳变更大
    steps = np.where(np.abs(ddy) > Y_STEP_THRESH_PX)[0]

    if len(steps) > 1:
        merged = [steps[0]]
        for s in steps[1:]:
            if s - merged[-1] > Y_STEP_MERGE:
                merged.append(s)
        steps = np.array(merged)

    return y_smooth, steps


def draw_4panel(roi_bgr, disp_roi, bead_mask, hist_n, centers,
                hist_peaks, peak_d, y_profile, y_steps, fidx):
    H, W = roi_bgr.shape[:2]
    canvas = np.zeros((H * 2, W * 2, 3), dtype=np.uint8)

    # ── 左上: 原图 + Y 轴阶梯红虚线 ──
    panel = roi_bgr.copy()
    for si, sy in enumerate(y_steps):
        x = 0
        while x < W:
            cv2.line(panel, (x, sy), (min(x + 10, W), sy), (0, 0, 255), 2)
            x += 18
        cv2.putText(panel, f"L{si}", (5, sy - 3),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 255, 255), 1)
    cv2.putText(panel, f"F{fidx} | {len(y_steps)} layers",
                (5, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)
    canvas[0:H, 0:W] = panel

    # ── 右上: disparity 伪彩 (bead only) ──
    d_vis = vis_disparity(disp_roi, color_map=cv2.COLORMAP_TURBO)
    d_vis[bead_mask == 0] = d_vis[bead_mask == 0] // 4
    canvas[0:H, W:W*2] = d_vis

    # ── 左下: Y 轴 disparity 剖面 ──
    profile_img = np.zeros((H, W, 3), dtype=np.uint8)
    if len(y_profile) > 0 and np.any(np.isfinite(y_profile)):
        vmin = np.nanmin(y_profile)
        vmax = np.nanmax(y_profile)
        if vmax > vmin:
            for y in range(1, H):
                x1 = int((y_profile[y-1] - vmin) / (vmax - vmin) * (W - 20)) + 10
                x2 = int((y_profile[y] - vmin) / (vmax - vmin) * (W - 20)) + 10
                cv2.line(profile_img, (x1, y-1), (x2, y), (0, 200, 0), 1)
            for sy in y_steps:
                cv2.line(profile_img, (0, sy), (W, sy), (0, 0, 255), 1)
            cv2.putText(profile_img, f"disp={vmin:.1f}~{vmax:.1f}px",
                        (5, H - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (200, 200, 200), 1)
    cv2.putText(profile_img, "Y-profile (disp)", (5, 12),
                cv2.FONT_HERSHEY_SIMPLEX, 0.35, (200, 200, 200), 1)
    canvas[H:H*2, 0:W] = profile_img

    # ── 右下: disparity 直方图 ──
    hist_img = np.zeros((H, W, 3), dtype=np.uint8)
    if len(hist_n) > 0 and hist_n.max() > 0:
        n_bins = len(hist_n)
        for i in range(n_bins):
            x_pos = int(i / n_bins * W)
            bar_h = int(hist_n[i] * (H - 20))
            cv2.line(hist_img, (x_pos, H), (x_pos, H - bar_h), (0, 200, 0), 1)
        for pi in hist_peaks:
            x_pos = int(pi / n_bins * W)
            cv2.line(hist_img, (x_pos, 0), (x_pos, H), (0, 0, 255), 1)
            label = f"{centers[pi]:.1f}px"
            cv2.putText(hist_img, label, (x_pos + 2, 15),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.28, (0, 255, 255), 1)
    cv2.putText(hist_img, f"Disp hist | {len(hist_peaks)} peaks",
                (5, H - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (200, 200, 200), 1)
    canvas[H:H*2, W:W*2] = hist_img

    # ── 干净原图 + 线 ──
    clean = roi_bgr.copy()
    for si, sy in enumerate(y_steps):
        x = 0
        while x < W:
            cv2.line(clean, (x, sy), (min(x + 10, W), sy), (0, 0, 255), 2)
            x += 18
        cv2.putText(clean, f"L{si}", (3, sy - 3),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)
    cv2.putText(clean, f"F{fidx} | {len(y_steps)} layers",
                (5, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)

    return canvas, clean


def main():
    if not SVO_PATH.exists():
        print(f"[ERROR] SVO not found: {SVO_PATH}"); sys.exit(1)

    out_dir = build_output_dir(SVO_PATH)

    zed = open_svo(SVO_PATH)
    total = zed.get_svo_number_of_frames()
    frame_list = list(range(max(0, FRAME_START), min(total, FRAME_END), FRAME_STEP))
    n_frames = len(frame_list)

    print(f"{'='*60}")
    print(f"  FFS Disparity-Domain Layer Analysis")
    print(f"{'='*60}")
    print(f"  SVO:     {SVO_PATH.name}")
    print(f"  Model:   {MODEL_NAME}  iters={VALID_ITERS}")
    print(f"  ROI:     {ROI}")
    print(f"  Frames:  {n_frames}")
    print(f"  Output:  {out_dir}")
    print(f"{'='*60}\n")

    print("  Loading FFS model...")
    torch.autograd.set_grad_enabled(False)
    model = load_ffs(MODEL_NAME, VALID_ITERS, MAX_DISP)
    print("  Model ready.\n")

    left_mat, right_mat = sl.Mat(), sl.Mat()
    cache = {}
    rows = []
    t_total = time.perf_counter()

    for i, fidx in enumerate(frame_list):
        t0 = time.perf_counter()

        zed.set_svo_position(fidx)
        if zed.grab() != sl.ERROR_CODE.SUCCESS:
            print(f"  [SKIP] {fidx}"); continue

        zed.retrieve_image(left_mat, sl.VIEW.LEFT)
        zed.retrieve_image(right_mat, sl.VIEW.RIGHT)
        img_l = cv2.cvtColor(left_mat.get_data()[:, :, :3].copy(), cv2.COLOR_BGRA2RGB)
        img_r = cv2.cvtColor(right_mat.get_data()[:, :, :3].copy(), cv2.COLOR_BGRA2RGB)

        # FFS
        t_inf = time.perf_counter()
        disp = run_ffs(model, img_l, img_r, VALID_ITERS, SCALE, cache)
        t_inf = time.perf_counter() - t_inf

        # ROI
        roi_s = [int(v * SCALE) for v in ROI] if SCALE != 1.0 else ROI
        roi_bgr = extract_roi(cv2.cvtColor(img_l, cv2.COLOR_RGB2BGR), roi_s)
        roi_disp = extract_roi(disp, roi_s)
        bead_mask = make_bead_mask(roi_bgr)

        # Bead disparity
        bead_d = roi_disp[bead_mask > 0]
        bead_d = bead_d[bead_d > 1]
        if len(bead_d) < 50:
            print(f"  [{i+1}/{n_frames}] frame {fidx}: no bead data"); continue

        # Analysis
        hist_n, centers, hist_peaks, peak_d = analyze_disp_histogram(bead_d)
        y_profile, y_steps = analyze_y_disp_profile(roi_disp, bead_mask)

        t_frame = time.perf_counter() - t0

        # Vis
        canvas, clean = draw_4panel(
            roi_bgr, roi_disp, bead_mask,
            hist_n, centers, hist_peaks, peak_d,
            y_profile, y_steps, fidx,
        )
        cv2.imwrite(str(out_dir / f"disp_{fidx:06d}.jpg"), canvas,
                    [cv2.IMWRITE_JPEG_QUALITY, 92])
        cv2.imwrite(str(out_dir / f"lines_{fidx:06d}.jpg"), clean,
                    [cv2.IMWRITE_JPEG_QUALITY, 92])

        row = {
            'frame': fidx,
            'disp_median': round(float(np.median(bead_d)), 2),
            'disp_min': round(float(bead_d.min()), 2),
            'disp_max': round(float(bead_d.max()), 2),
            'disp_range': round(float(bead_d.max() - bead_d.min()), 2),
            'disp_std': round(float(bead_d.std()), 3),
            'hist_peaks': len(hist_peaks),
            'peak_disps': ';'.join(f"{d:.1f}" for d in peak_d),
            'y_steps': len(y_steps),
            't_infer_s': round(t_inf, 3),
            't_frame_s': round(t_frame, 3),
        }
        rows.append(row)

        print(f"  [{i+1:3d}/{n_frames}]  F{fidx:6d}  "
              f"disp={row['disp_median']:.1f} [{row['disp_range']:.1f}px]  "
              f"hist={len(hist_peaks)}  ystep={len(y_steps)}  "
              f"infer={t_inf:.2f}s")

    t_elapsed = time.perf_counter() - t_total
    zed.close()

    # CSV
    csv_path = out_dir / "disp_layers.csv"
    if rows:
        with open(csv_path, "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
            w.writeheader()
            w.writerows(rows)

    with open(out_dir / "run_config.json", "w") as f:
        json.dump({
            "svo_path": str(SVO_PATH), "model": MODEL_NAME,
            "valid_iters": VALID_ITERS, "scale": SCALE, "roi": ROI,
            "frame_range": [FRAME_START, FRAME_END, FRAME_STEP],
            "disp_hist_bins": DISP_HIST_BINS,
            "y_step_thresh_px": Y_STEP_THRESH_PX,
        }, f, indent=2)

    print(f"\n{'='*60}")
    print(f"  DONE — {len(rows)} frames, {t_elapsed:.1f}s total")
    if rows:
        hp = [r['hist_peaks'] for r in rows]
        ys = [r['y_steps'] for r in rows]
        dr = [r['disp_range'] for r in rows]
        print(f"  Hist peaks:  {np.mean(hp):.1f} avg, max={max(hp)}")
        print(f"  Y steps:     {np.mean(ys):.1f} avg, max={max(ys)}")
        print(f"  Disp range:  {np.mean(dr):.1f} ± {np.std(dr):.1f} px")
    print(f"  Output: {out_dir}")
    print(f"{'='*60}")


if __name__ == "__main__":
    main()
