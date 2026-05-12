"""
depth2_ffs.py — Fast-FoundationStereo 高精度深度估计
  从 SVO2 取左右 rectified 图 → FFS 亚像素立体匹配 → disparity → depth
  输出: 伪彩深度图, per-frame timing CSV, ROI 深度统计

=== 深度来源 ===
  完全绕开 ZED SDK 的深度算法。
  ZED SDK 只负责: 读 SVO2 文件 + 提供出厂标定参数 (fx, baseline)
  深度计算:
    1. pyzed 读左目/右目 rectified 图 (ZED 出厂已标定, 图像已 rectify)
    2. Fast-FoundationStereo 做亚像素立体匹配 → disparity map (float32, 精度 ~0.05px)
    3. depth = fx * baseline / disparity
  亚像素精度:
    ΔZ = Z² / (f·B) · Δd
    Z=0.45m, f=1065px, B=0.12m, Δd=0.05px → ΔZ ≈ 0.08mm  (远小于层高)

=== 环境 ===
  需要 conda activate ffs (Python 3.12 + PyTorch nightly cu128)
  不能在 zedenv 中运行
"""

import os, sys, csv, time, json
import cv2
import numpy as np
import torch
from pathlib import Path
from datetime import datetime

# ── ZED SDK DLL (Windows) ──
if os.name == "nt":
    for p in [
        r"C:\Program Files (x86)\ZED SDK\bin",
        r"C:\Program Files (x86)\ZED SDK\dependencies\bin",
        r"C:\Program Files\NVIDIA GPU Computing Toolkit\CUDA\v12.8\bin",
    ]:
        if os.path.isdir(p):
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

# ── 帧范围 ──
FRAME_START = 150
FRAME_END   = 11750
FRAME_STEP  = 100

# ── ROI [x, y, w, h] (左目图像坐标) ──
ROI = [1200, 600, 400, 400]

# ── FFS 模型 ──
#    23-36-37 = 最精确, 20-26-39 = 中等, 20-30-48 = 最快
MODEL_NAME   = "23-36-37"
VALID_ITERS  = 8           # 4=快, 8=最精确
MAX_DISP     = 192
SCALE        = 1.0         # 图像缩放 (0.5 可加速, 但降低精度)

# ── 深度范围 (米), 用于伪彩映射 ──
Z_VIS_MIN = 0.30
Z_VIS_MAX = 0.65

# ═══════════════════════════════════════════════════════════════════════════════
#  END USER CONFIG
# ═══════════════════════════════════════════════════════════════════════════════


def build_output_dir(svo_path: Path) -> Path:
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    out = svo_path.parent / f"depth_ffs_{ts}"
    out.mkdir(parents=True, exist_ok=True)
    return out


def open_svo(svo_path: Path):
    """打开 SVO, depth_mode=NONE (不用 ZED 算深度)"""
    zed = sl.Camera()
    p = sl.InitParameters()
    p.set_from_svo_file(str(svo_path))
    p.svo_real_time_mode = False
    p.depth_mode = sl.DEPTH_MODE.NONE
    p.coordinate_units = sl.UNIT.METER

    status = zed.open(p)
    if status != sl.ERROR_CODE.SUCCESS:
        print(f"[ERROR] ZED open failed: {status}")
        sys.exit(1)
    return zed


def get_calibration(zed):
    """从 ZED 标定参数中取 fx, fy, cx, cy, baseline"""
    cam_config = zed.get_camera_information().camera_configuration
    cal = cam_config.calibration_parameters
    left = cal.left_cam
    baseline_m = abs(cal.get_camera_baseline())  # already in meters when UNIT.METER
    return {
        "fx": left.fx, "fy": left.fy,
        "cx": left.cx, "cy": left.cy,
        "baseline_m": baseline_m,
        "width": cam_config.resolution.width,
        "height": cam_config.resolution.height,
    }


def load_ffs_model(model_name, valid_iters, max_disp):
    weight_dir = FFS_REPO / "weights" / model_name
    model_path = weight_dir / "model_best_bp2_serialize.pth"
    cfg_path = weight_dir / "cfg.yaml"
    if not model_path.exists():
        print(f"[ERROR] Model not found: {model_path}")
        sys.exit(1)
    with open(cfg_path) as f:
        cfg = yaml.safe_load(f)
    model = torch.load(str(model_path), map_location="cpu", weights_only=False)
    model.args.valid_iters = valid_iters
    model.args.max_disp = max_disp
    model.cuda().eval()
    return model


def run_ffs(model, img_left_rgb, img_right_rgb, valid_iters, scale, padder_cache):
    """FFS 推理, 返回 disparity (H,W) float32"""
    if scale != 1.0:
        img_left_rgb = cv2.resize(img_left_rgb, fx=scale, fy=scale, dsize=None)
        img_right_rgb = cv2.resize(img_right_rgb,
                                    dsize=(img_left_rgb.shape[1], img_left_rgb.shape[0]))

    H, W = img_left_rgb.shape[:2]
    t0 = torch.as_tensor(img_left_rgb).cuda().float()[None].permute(0, 3, 1, 2)
    t1 = torch.as_tensor(img_right_rgb).cuda().float()[None].permute(0, 3, 1, 2)

    if padder_cache.get("shape") != t0.shape:
        padder_cache["padder"] = InputPadder(t0.shape, divis_by=32, force_square=False)
        padder_cache["shape"] = t0.shape

    t0p, t1p = padder_cache["padder"].pad(t0, t1)
    with torch.amp.autocast("cuda", enabled=True, dtype=AMP_DTYPE):
        disp = model.forward(t0p, t1p, iters=valid_iters, test_mode=True,
                             optimize_build_volume="pytorch1")
    disp = padder_cache["padder"].unpad(disp.float())
    disp = disp.data.cpu().numpy().reshape(H, W).clip(0, None)
    return disp


def disp_to_depth(disp, fx, baseline_m, scale):
    """disparity → depth (米)"""
    fx_scaled = fx * scale
    depth = fx_scaled * baseline_m / np.clip(disp, 0.1, None)
    return depth


def extract_roi(arr, roi):
    if roi is None:
        return arr.copy()
    x, y, w, h = roi
    H = arr.shape[0]
    W = arr.shape[1]
    x = max(0, min(x, W - 1))
    y = max(0, min(y, H - 1))
    w = min(w, W - x)
    h = min(h, H - y)
    return arr[y:y+h, x:x+w].copy()


def make_depth_vis(depth_roi, z_min, z_max):
    """深度值 → JET 伪彩 (近=红, 远=蓝)"""
    d = depth_roi.copy()
    d[~np.isfinite(d)] = z_max
    d = np.clip(d, z_min, z_max)
    norm = ((d - z_min) / (z_max - z_min) * 255).astype(np.uint8)
    # 反转: 近处 (小值) = 暖色
    color = cv2.applyColorMap(255 - norm, cv2.COLORMAP_JET)
    return color


def main():
    if not SVO_PATH.exists():
        print(f"[ERROR] SVO not found: {SVO_PATH}")
        sys.exit(1)

    out_dir = build_output_dir(SVO_PATH)

    # ── 打开 SVO & 标定 ──
    zed = open_svo(SVO_PATH)
    cal = get_calibration(zed)
    total_frames = zed.get_svo_number_of_frames()

    frame_list = list(range(
        max(0, FRAME_START),
        min(total_frames, FRAME_END),
        FRAME_STEP,
    ))
    n_frames = len(frame_list)

    print(f"{'='*65}")
    print(f"  Fast-FoundationStereo Depth Estimation")
    print(f"{'='*65}")
    print(f"  SVO:         {SVO_PATH.name}")
    print(f"  Resolution:  {cal['width']} x {cal['height']}")
    print(f"  fx:          {cal['fx']:.2f} px")
    print(f"  baseline:    {cal['baseline_m']:.4f} m")
    print(f"  Model:       {MODEL_NAME}  iters={VALID_ITERS}  max_disp={MAX_DISP}")
    print(f"  Scale:       {SCALE}")
    print(f"  ROI:         {ROI}")
    print(f"  Frames:      {n_frames}  [{FRAME_START}:{FRAME_END}:{FRAME_STEP}]")
    print(f"  Depth range: {Z_VIS_MIN}~{Z_VIS_MAX} m (vis)")
    print(f"  Output:      {out_dir}")

    # 亚像素精度估算
    Z_typ = 0.45
    delta_d = 0.05  # FFS 典型亚像素精度
    delta_z = Z_typ**2 / (cal['fx'] * cal['baseline_m']) * delta_d
    print(f"  ΔZ @{Z_typ}m:   ~{delta_z*1000:.2f} mm  (Δd={delta_d}px)")
    print(f"{'='*65}\n")

    # ── 加载 FFS 模型 ──
    print("  Loading FFS model... (first frame will be slow due to compilation)")
    t_load = time.perf_counter()
    torch.autograd.set_grad_enabled(False)
    model = load_ffs_model(MODEL_NAME, VALID_ITERS, MAX_DISP)
    print(f"  Model loaded in {time.perf_counter() - t_load:.1f}s\n")

    # ── 逐帧处理 ──
    left_mat = sl.Mat()
    right_mat = sl.Mat()
    padder_cache = {}
    rows = []
    t_total = time.perf_counter()

    for i, fidx in enumerate(frame_list):
        t0 = time.perf_counter()

        zed.set_svo_position(fidx)
        if zed.grab() != sl.ERROR_CODE.SUCCESS:
            print(f"  [SKIP] Frame {fidx}")
            continue

        # 取左右 RGB
        zed.retrieve_image(left_mat, sl.VIEW.LEFT)
        zed.retrieve_image(right_mat, sl.VIEW.RIGHT)
        img_l = left_mat.get_data()[:, :, :3].copy()
        img_r = right_mat.get_data()[:, :, :3].copy()
        img_l = cv2.cvtColor(img_l, cv2.COLOR_BGRA2RGB)
        img_r = cv2.cvtColor(img_r, cv2.COLOR_BGRA2RGB)

        t_grab = time.perf_counter() - t0

        # FFS 推理 (全图)
        t_infer = time.perf_counter()
        disp = run_ffs(model, img_l, img_r, VALID_ITERS, SCALE, padder_cache)
        t_infer = time.perf_counter() - t_infer

        # disparity → depth
        depth = disp_to_depth(disp, cal['fx'], cal['baseline_m'], SCALE)

        # ROI 裁切
        if SCALE != 1.0:
            roi_scaled = [int(v * SCALE) for v in ROI]
        else:
            roi_scaled = ROI

        roi_bgr = extract_roi(cv2.cvtColor(img_l, cv2.COLOR_RGB2BGR), roi_scaled)
        roi_depth = extract_roi(depth, roi_scaled)
        roi_disp = extract_roi(disp, roi_scaled)

        # 有效深度统计
        valid = roi_depth[np.isfinite(roi_depth) &
                          (roi_depth > Z_VIS_MIN) & (roi_depth < Z_VIS_MAX)]
        n_valid = len(valid)

        t_frame = time.perf_counter() - t0

        # ── 伪彩可视化 ──
        depth_color = make_depth_vis(roi_depth, Z_VIS_MIN, Z_VIS_MAX)
        disp_color = vis_disparity(roi_disp, color_map=cv2.COLORMAP_TURBO)

        # 拼接: 左=原图, 中=disparity伪彩, 右=depth伪彩
        h, w = roi_bgr.shape[:2]
        canvas = np.zeros((h, w * 3, 3), dtype=np.uint8)
        canvas[:, 0:w] = roi_bgr
        canvas[:, w:w*2] = disp_color
        canvas[:, w*2:w*3] = depth_color

        # 标注
        cv2.putText(canvas, f"F{fidx} RGB", (5, 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)
        cv2.putText(canvas, "Disparity (FFS)", (w + 5, 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)
        cv2.putText(canvas, "Depth (m)", (w*2 + 5, 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)

        if n_valid > 0:
            info = (f"Z={np.median(valid):.4f}m  "
                    f"range={valid.max()-valid.min():.4f}m  "
                    f"infer={t_infer:.2f}s")
            cv2.putText(canvas, info, (5, h - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.35, (200, 200, 200), 1)

        cv2.imwrite(str(out_dir / f"depth_{fidx:06d}.jpg"),
                    canvas, [cv2.IMWRITE_JPEG_QUALITY, 92])

        # CSV row
        row = {
            'frame': fidx,
            'disp_median': round(float(np.median(roi_disp[roi_disp > 0])), 3) if np.any(roi_disp > 0) else 0,
            'disp_subpx_std': round(float(np.std(roi_disp[roi_disp > 0] % 1)), 4) if np.any(roi_disp > 0) else 0,
            'depth_median': round(float(np.median(valid)), 5) if n_valid > 0 else 0,
            'depth_min': round(float(valid.min()), 5) if n_valid > 0 else 0,
            'depth_max': round(float(valid.max()), 5) if n_valid > 0 else 0,
            'depth_std': round(float(valid.std()), 6) if n_valid > 0 else 0,
            'depth_range_mm': round(float((valid.max() - valid.min()) * 1000), 3) if n_valid > 0 else 0,
            'n_valid_px': n_valid,
            't_grab_s': round(t_grab, 4),
            't_infer_s': round(t_infer, 4),
            't_frame_s': round(t_frame, 4),
        }
        rows.append(row)

        print(f"  [{i+1:4d}/{n_frames}]  frame {fidx:6d}  "
              f"disp={row['disp_median']:.1f}px  "
              f"Z={row['depth_median']:.4f}m  "
              f"Δ={row['depth_range_mm']:.1f}mm  "
              f"infer={t_infer:.2f}s  total={t_frame:.2f}s")

    t_elapsed = time.perf_counter() - t_total
    zed.close()

    # ── CSV ──
    csv_path = out_dir / "depth_ffs.csv"
    with open(csv_path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()) if rows else [])
        w.writeheader()
        w.writerows(rows)

    # ── Config JSON ──
    with open(out_dir / "run_config.json", "w") as f:
        json.dump({
            "svo_path": str(SVO_PATH),
            "model": MODEL_NAME,
            "valid_iters": VALID_ITERS,
            "max_disp": MAX_DISP,
            "scale": SCALE,
            "roi": ROI,
            "frame_range": [FRAME_START, FRAME_END, FRAME_STEP],
            "z_vis_range": [Z_VIS_MIN, Z_VIS_MAX],
            "calibration": cal,
            "delta_z_mm_at_045m": round(delta_z * 1000, 3),
        }, f, indent=2)

    # ── Summary ──
    print(f"\n{'='*65}")
    print(f"  SUMMARY")
    print(f"{'='*65}")
    print(f"  Frames processed: {len(rows)}")
    print(f"  Total time:       {t_elapsed:.1f}s")
    if rows:
        infer_times = [r['t_infer_s'] for r in rows]
        print(f"  Infer time/frame: {np.mean(infer_times):.2f}s avg  "
              f"(first={infer_times[0]:.2f}s, rest={np.mean(infer_times[1:]):.2f}s)")
        z_ranges = [r['depth_range_mm'] for r in rows if r['depth_range_mm'] > 0]
        if z_ranges:
            print(f"  Depth range (ROI): {np.mean(z_ranges):.1f} ± {np.std(z_ranges):.1f} mm")
    print(f"  Output: {out_dir}")
    print(f"{'='*65}")


if __name__ == "__main__":
    main()
