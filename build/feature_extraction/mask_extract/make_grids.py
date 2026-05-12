"""
make_grids.py — 生成 4 种帧号对齐的 10×3 网格
  1. grid_left_10x3.jpg      左目 RGB (ROI)
  2. grid_right_10x3.jpg     右目 RGB (ROI)
  3. grid_sam2_overlay_10x3.jpg  SAM2 overlay (最近可用帧)
  4. grid_ffs_disparity_10x3.jpg FFS disparity 伪彩 (bilateral smoothed + global norm)

帧号以 depth2_ffs 输出文件为 master, 均匀选 30 帧.
disparity 重新推理, 加 bilateral filter 平滑 + 全局归一化, 解决 wrinkle 问题.

环境: conda activate ffs
"""

import os, sys, time
import cv2
import numpy as np
import torch
from pathlib import Path

# ── ZED SDK DLL ──
if os.name == "nt":
    for p in [
        r"C:\Program Files (x86)\ZED SDK\bin",
        r"C:\Program Files (x86)\ZED SDK\dependencies\bin",
        r"C:\Program Files\NVIDIA GPU Computing Toolkit\CUDA\v12.8\bin",
    ]:
        if os.path.isdir(p):
            os.add_dll_directory(p)

import pyzed.sl as sl

# ── FFS ──
FFS_REPO = Path(r"C:\Users\888y9\Desktop\Repo\Fast-FoundationStereo")
sys.path.insert(0, str(FFS_REPO))
from core.utils.utils import InputPadder
from Utils import AMP_DTYPE, vis_disparity

# ═══════════════════════════════════════════════════════════════════════════════
#  CONFIG
# ═══════════════════════════════════════════════════════════════════════════════

SVO_PATH = Path(
    r"C:\Users\888y9\Desktop\rsi_printing\recorded_data"
    r"\20260331_202433\recording_20260331_202433_001_20260331_202433.svo2"
)
SAM2_DIR = Path(
    r"C:\Users\888y9\Desktop\rsi_printing\recorded_data"
    r"\20260331_202433\sam2_mask_20260408_202232"
)
FFS_OUT_DIR = Path(
    r"C:\Users\888y9\Desktop\rsi_printing\recorded_data"
    r"\20260331_202433\depth_ffs_20260408_212541"
)
OUT_DIR = Path(
    r"C:\Users\888y9\Desktop\rsi_printing\recorded_data\20260331_202433"
)

ROI = [1200, 600, 400, 400]   # [x, y, w, h]

# FFS model
MODEL_NAME  = "23-36-37"
VALID_ITERS = 8
MAX_DISP    = 192
SCALE       = 1.0

# Bilateral filter: 平滑 disparity 噪声, 保留边缘
BILATERAL_D     = 9       # kernel size
BILATERAL_SIG_C = 5.0     # disparity tolerance (px)
BILATERAL_SIG_S = 9.0     # spatial sigma (px)

COLS, ROWS = 10, 3
N_GRID = COLS * ROWS       # 30

# ═══════════════════════════════════════════════════════════════════════════════


def uniform_sample(lst, n):
    if len(lst) <= n:
        return lst
    idx = np.linspace(0, len(lst) - 1, n, dtype=int)
    return [lst[i] for i in idx]


def make_grid(images, cols, rows):
    h, w = images[0].shape[:2]
    canvas = np.zeros((rows * h, cols * w, 3), dtype=np.uint8)
    for i, img in enumerate(images):
        if i >= cols * rows:
            break
        r, c = divmod(i, cols)
        cell = cv2.resize(img, (w, h)) if img.shape[:2] != (h, w) else img
        canvas[r * h:(r + 1) * h, c * w:(c + 1) * w] = cell
    return canvas


def roi_crop(arr, roi):
    x, y, w, h = roi
    return arr[y:y + h, x:x + w].copy()


def load_ffs():
    weight_dir = FFS_REPO / "weights" / MODEL_NAME
    model = torch.load(str(weight_dir / "model_best_bp2_serialize.pth"),
                       map_location="cpu", weights_only=False)
    model.args.valid_iters = VALID_ITERS
    model.args.max_disp = MAX_DISP
    model.cuda().eval()
    return model


def ffs_infer(model, img_l_rgb, img_r_rgb, cache):
    H, W = img_l_rgb.shape[:2]
    t0 = torch.as_tensor(img_l_rgb).cuda().float()[None].permute(0, 3, 1, 2)
    t1 = torch.as_tensor(img_r_rgb).cuda().float()[None].permute(0, 3, 1, 2)
    if cache.get("shape") != t0.shape:
        cache["padder"] = InputPadder(t0.shape, divis_by=32, force_square=False)
        cache["shape"] = t0.shape
    t0p, t1p = cache["padder"].pad(t0, t1)
    with torch.amp.autocast("cuda", enabled=True, dtype=AMP_DTYPE):
        disp = model.forward(t0p, t1p, iters=VALID_ITERS, test_mode=True,
                             optimize_build_volume="pytorch1")
    disp = cache["padder"].unpad(disp.float())
    return disp.data.cpu().numpy().reshape(H, W).clip(0, None)


def main():
    t_start = time.perf_counter()

    # ── Master frame list: from depth2_ffs output ──
    ffs_files = sorted(FFS_OUT_DIR.glob("depth_*.jpg"))
    all_fnums = [int(f.stem.split("_")[1]) for f in ffs_files]
    master_frames = uniform_sample(all_fnums, N_GRID)
    print(f"Master frames ({len(master_frames)}): {master_frames[:5]} ... {master_frames[-3:]}")

    # ── SAM2 overlay lookup (nearest match) ──
    sam2_map = {}
    for f in SAM2_DIR.glob("overlay_*.jpg"):
        sam2_map[int(f.stem.split("_")[1])] = f
    sam2_fnums = sorted(sam2_map.keys())
    print(f"SAM2 overlays: {len(sam2_fnums)} frames [{sam2_fnums[0]}~{sam2_fnums[-1]}]")

    def nearest_sam2(fidx):
        if fidx in sam2_map:
            return fidx, sam2_map[fidx]
        if not sam2_fnums:
            return None, None
        idx = np.searchsorted(sam2_fnums, fidx)
        cands = []
        if idx > 0:
            cands.append(sam2_fnums[idx - 1])
        if idx < len(sam2_fnums):
            cands.append(sam2_fnums[idx])
        best = min(cands, key=lambda x: abs(x - fidx))
        return best, sam2_map[best]

    # ── Open SVO ──
    zed = sl.Camera()
    ip = sl.InitParameters()
    ip.set_from_svo_file(str(SVO_PATH))
    ip.svo_real_time_mode = False
    ip.depth_mode = sl.DEPTH_MODE.NONE
    if zed.open(ip) != sl.ERROR_CODE.SUCCESS:
        print("[ERROR] SVO open failed"); sys.exit(1)

    # ── Load FFS ──
    print("Loading FFS model...")
    torch.autograd.set_grad_enabled(False)
    model = load_ffs()
    cache = {}
    print("Model ready.\n")

    left_mat, right_mat = sl.Mat(), sl.Mat()

    imgs_left, imgs_right, imgs_sam2 = [], [], []
    disp_rois = []           # (fidx, roi_disp_smoothed)

    for i, fidx in enumerate(master_frames):
        zed.set_svo_position(fidx)
        if zed.grab() != sl.ERROR_CODE.SUCCESS:
            ph = np.zeros((ROI[3], ROI[2], 3), dtype=np.uint8)
            imgs_left.append(ph); imgs_right.append(ph)
            imgs_sam2.append(ph)
            disp_rois.append((fidx, np.zeros((ROI[3], ROI[2]), np.float32)))
            print(f"  [{i + 1:2d}/{len(master_frames)}] F{fidx} SKIP")
            continue

        zed.retrieve_image(left_mat, sl.VIEW.LEFT)
        zed.retrieve_image(right_mat, sl.VIEW.RIGHT)
        raw_l_bgr = left_mat.get_data()[:, :, :3].copy()
        raw_r_bgr = right_mat.get_data()[:, :, :3].copy()

        # ── Left / Right ROI (BGR for imwrite) ──
        roi_l = roi_crop(raw_l_bgr, ROI)
        roi_r = roi_crop(raw_r_bgr, ROI)
        cv2.putText(roi_l, f"F{fidx}", (5, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        cv2.putText(roi_r, f"F{fidx}", (5, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        imgs_left.append(roi_l)
        imgs_right.append(roi_r)

        # ── SAM2 overlay (nearest) ──
        sam2_fnum, sam2_path = nearest_sam2(fidx)
        if sam2_path is not None:
            sam2_img = cv2.imread(str(sam2_path))
            if sam2_img is not None:
                if sam2_img.shape[:2] != (ROI[3], ROI[2]):
                    sam2_img = cv2.resize(sam2_img, (ROI[2], ROI[3]))
                lbl = f"F{fidx}" if sam2_fnum == fidx else f"F{fidx}(~{sam2_fnum})"
                cv2.putText(sam2_img, lbl, (5, 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 1)
                imgs_sam2.append(sam2_img)
            else:
                ph = roi_l.copy()
                cv2.putText(ph, "No SAM2", (5, 40),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
                imgs_sam2.append(ph)
        else:
            ph = roi_l.copy()
            cv2.putText(ph, "No SAM2", (5, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
            imgs_sam2.append(ph)

        # ── FFS → bilateral filter ──
        img_l_rgb = np.ascontiguousarray(raw_l_bgr[:, :, ::-1])   # BGR→RGB
        img_r_rgb = np.ascontiguousarray(raw_r_bgr[:, :, ::-1])
        disp = ffs_infer(model, img_l_rgb, img_r_rgb, cache)
        roi_d = roi_crop(disp, ROI)
        roi_d_s = cv2.bilateralFilter(roi_d, BILATERAL_D,
                                      BILATERAL_SIG_C, BILATERAL_SIG_S)
        disp_rois.append((fidx, roi_d_s))
        print(f"  [{i + 1:2d}/{len(master_frames)}] F{fidx}  "
              f"disp={np.median(roi_d[roi_d > 1]):.0f}px  "
              f"sam2={'=' if sam2_fnum == fidx else f'~{sam2_fnum}'}")

    zed.close()
    torch.cuda.empty_cache()

    # ── Global disparity range (percentile) ──
    all_valid = np.concatenate([rd[rd > 1].ravel() for _, rd in disp_rois
                                if np.any(rd > 1)])
    dmin = float(np.percentile(all_valid, 1))
    dmax = float(np.percentile(all_valid, 99))
    print(f"\nGlobal disp range: {dmin:.1f} ~ {dmax:.1f} px")

    # ── Colormap disparity (global norm, TURBO) ──
    imgs_disp = []
    for fidx, rd in disp_rois:
        vis = vis_disparity(rd, min_val=dmin, max_val=dmax,
                            color_map=cv2.COLORMAP_TURBO)
        vis = vis[:, :, ::-1].copy()   # RGB→BGR for imwrite
        cv2.putText(vis, f"F{fidx}", (5, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        imgs_disp.append(vis)

    # ── Make grids ──
    grids = {
        "grid_left_10x3":          imgs_left,
        "grid_right_10x3":         imgs_right,
        "grid_sam2_overlay_10x3":  imgs_sam2,
        "grid_ffs_disparity_10x3": imgs_disp,
    }
    for name, imgs in grids.items():
        grid = make_grid(imgs, COLS, ROWS)
        out = OUT_DIR / f"{name}.jpg"
        cv2.imwrite(str(out), grid, [cv2.IMWRITE_JPEG_QUALITY, 92])
        print(f"  {name}: {grid.shape[1]}x{grid.shape[0]} -> {out.name}")

    print(f"\nDone in {time.perf_counter() - t_start:.1f}s")


if __name__ == "__main__":
    main()
