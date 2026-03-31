"""
run_sam2_t3.py — SAM2 bead width on newest SVO (20260326_213929)
  → matches Ye_RSI_t3_fixed.src robot session
  → output: vis images + bead_width.csv + mm_per_px report

Run with:  run_sam2.bat  build\DataCollection\cam_adjust\run_sam2_t3.py
"""

import sys, csv, cv2, time, numpy as np
import pyzed.sl as sl
from pathlib import Path
from datetime import datetime
import torch

from sam2.build_sam import build_sam2
from sam2.automatic_mask_generator import SAM2AutomaticMaskGenerator

# ─── Paths ───────────────────────────────────────────────────────────────────
_HERE     = Path(__file__).resolve().parent
CKPT_PATH = _HERE / "t2" / "sam2_checkpoints" / "sam2.1_hiera_small.pt"
MODEL_CFG = "configs/sam2.1/sam2.1_hiera_s.yaml"

SVO_PATH  = Path(r"C:\Users\888y9\Desktop\rsi_printing\recorded_data\20260324_174844\recording_20260324_174844.svo2")
RSI_CSV   = Path(r"C:\Users\888y9\Desktop\rsi_printing\rsi_data\rsi_data_20260324_174844.csv")

OUT_BASE  = _HERE.parent / "labeling" / "post_labeling" / "t3"

# ─── Frame selection ─────────────────────────────────────────────────────────
# Sample evenly: 20 frames from first 25% of recording (active print section)
# + 5 diagnostic frames near start/end
N_SAMPLE   = 20
N_DIAG     = 5
SEARCH_WIN = 0.25      # search in first 25% (the actual print moves)

# ─── ROI config (same as v4 production) ─────────────────────────────────────
ROI_SIZE   = 224
ROI_DX     = -30       # px offset from nozzle centre, X
ROI_DY     = 260       # px offset from nozzle bottom, Y (Fix A: was 350, targets current layer not old ones)

SCAR_ZONE_RATIO = 0.40
CONE_RATIO      = 0.55
N_SCANLINES     = 30
WIDTH_MIN       = 2
WIDTH_MAX       = 40
BEAD_MIN_ASPECT = 2.5
BEAD_MIN_AREA   = 30
BEAD_MAX_AREA   = 8000

# ─── SAM2 AMG params ─────────────────────────────────────────────────────────
AMG_PARAMS = dict(
    points_per_side=32,
    points_per_batch=64,
    pred_iou_thresh=0.7,
    stability_score_thresh=0.85,
    min_mask_region_area=20,
)

# ─── GUM uncertainty constants ───────────────────────────────────────────────
U_Z  = 0.002   # ZED 2i depth noise ~2mm at ~0.45m
U_PX = 0.5     # SAM2 mask boundary ±0.5px
U_FX = 1.0     # factory calibration ±1px


# ─── Helpers ─────────────────────────────────────────────────────────────────

def find_nozzle(bgr):
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, np.array([100, 100, 80]), np.array([130, 255, 255]))
    k = cv2.getStructuringElement(cv2.MORPH_RECT, (7, 7))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k, iterations=3)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  k, iterations=2)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    valid = [(c, cv2.contourArea(c)) for c in contours
             if 1000 < cv2.contourArea(c) < 300000]
    if not valid:
        mask = cv2.inRange(hsv, np.array([5, 150, 180]), np.array([25, 255, 255]))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k, iterations=3)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  k, iterations=2)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        valid = [(c, cv2.contourArea(c)) for c in contours
                 if 1000 < cv2.contourArea(c) < 300000]
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


def _find_runs(arr):
    d = np.diff(arr.astype(np.int16))
    starts = list(np.where(d > 0)[0] + 1)
    ends   = list(np.where(d < 0)[0] + 1)
    if not starts and len(arr) > 0 and arr[0] > 0:
        starts = [0]
    if not ends and len(arr) > 0 and arr[-1] > 0:
        ends = [len(arr)]
    return list(zip(starts, ends))


def filter_bead_masks(masks_data):
    candidates = []
    roi_area = ROI_SIZE * ROI_SIZE
    scar_y   = ROI_SIZE * SCAR_ZONE_RATIO
    for m in masks_data:
        seg  = m['segmentation'].astype(np.uint8)
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
        if aspect < BEAD_MIN_ASPECT or rcy < scar_y:
            continue
        hull     = cv2.convexHull(cnt)
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


def measure_bead_width(mask_uint8):
    ys, xs = np.where(mask_uint8 > 0)
    if len(xs) < 5:
        return [], 0
    x_span, y_span = xs.max() - xs.min(), ys.max() - ys.min()
    raw = []
    if x_span >= y_span:
        for sx in np.linspace(xs.min()+2, xs.max()-2, N_SCANLINES).astype(int):
            for s, e in _find_runs(mask_uint8[:, sx]):
                w = e - s
                if WIDTH_MIN <= w <= WIDTH_MAX:
                    raw.append({'pos': sx, 'start': s, 'end': e,
                                'width': w, 'axis': 'Y'})
    else:
        for sy in np.linspace(ys.min()+2, ys.max()-2, N_SCANLINES).astype(int):
            for s, e in _find_runs(mask_uint8[sy, :]):
                w = e - s
                if WIDTH_MIN <= w <= WIDTH_MAX:
                    raw.append({'pos': sy, 'start': s, 'end': e,
                                'width': w, 'axis': 'X'})
    if not raw:
        return [], 0
    ws      = np.array([r['width'] for r in raw])
    med_w   = np.median(ws)
    kept    = [r for r in raw if r['width'] >= med_w * CONE_RATIO]
    removed = len(raw) - len(kept)
    return kept, removed


def compute_uncertainty(w_px_arr, mm_per_px, z_med, fx):
    n      = len(w_px_arr)
    w_mean = w_px_arr.mean()
    w_std  = w_px_arr.std()
    ci95_px = 1.96 * w_std / np.sqrt(n) if n > 1 else w_std
    if mm_per_px and z_med > 0 and w_mean > 0:
        w_mm    = w_mean * mm_per_px
        ci95_mm = ci95_px * mm_per_px
        u_w_mm  = w_mm * np.sqrt(
            (U_Z / z_med)**2 + (U_PX / w_mean)**2 + (U_FX / fx)**2
        )
    else:
        w_mm = ci95_mm = u_w_mm = None
    return {
        'ci95_px': round(ci95_px, 3),
        'ci95_mm': round(ci95_mm, 4) if ci95_mm is not None else '',
        'u_w_mm':  round(u_w_mm, 4)  if u_w_mm  is not None else '',
    }


def save_vis(bgr_full, bgr_roi, x1, y1, nozzle, bead_cands, width_data,
             mm_per_px, out_dir, fidx):
    # full frame with ROI box
    vis = bgr_full.copy()
    cv2.rectangle(vis, (x1, y1), (x1+ROI_SIZE, y1+ROI_SIZE), (0, 255, 255), 2)
    if nozzle:
        cv2.drawMarker(vis, nozzle, (0, 0, 255), cv2.MARKER_CROSS, 20, 2)
    cv2.putText(vis, f"Frame {fidx}", (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
    cv2.imwrite(str(out_dir / f"f{fidx:05d}_full.jpg"),
                vis, [cv2.IMWRITE_JPEG_QUALITY, 88])

    # ROI overlay with mask, scar-zone line, width scanlines
    overlay = bgr_roi.copy()
    if bead_cands:
        overlay[bead_cands[0]['mask'] > 0] = (0, 255, 0)
    blend = cv2.addWeighted(bgr_roi, 0.5, overlay, 0.5, 0)

    sy = int(ROI_SIZE * SCAR_ZONE_RATIO)
    cv2.line(blend, (0, sy), (ROI_SIZE, sy), (0, 0, 255), 1)
    cv2.putText(blend, "scar", (2, sy-3),
                cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0, 0, 255), 1)

    for wd in width_data:
        if wd['axis'] == 'Y':
            cv2.line(blend, (wd['pos'], wd['start']),
                     (wd['pos'], wd['end']), (0, 255, 255), 1)
        else:
            cv2.line(blend, (wd['start'], wd['pos']),
                     (wd['end'],   wd['pos']), (0, 255, 255), 1)

    if width_data:
        w_mean = np.mean([wd['width'] for wd in width_data])
        label  = f"{w_mean:.1f}px"
        if mm_per_px:
            label += f" = {w_mean*mm_per_px:.2f}mm"
        cv2.putText(blend, label, (5, 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1)

    cv2.imwrite(str(out_dir / f"f{fidx:05d}_roi.jpg"),
                blend, [cv2.IMWRITE_JPEG_QUALITY, 92])
    cv2.imwrite(str(out_dir / f"f{fidx:05d}_roi_3x.jpg"),
                cv2.resize(blend, (ROI_SIZE*3, ROI_SIZE*3),
                           interpolation=cv2.INTER_NEAREST),
                [cv2.IMWRITE_JPEG_QUALITY, 92])

    # binary mask
    if bead_cands:
        mask_img = (bead_cands[0]['mask'] * 255).astype(np.uint8)
        cv2.imwrite(str(out_dir / f"f{fidx:05d}_mask.png"), mask_img)
        np.savez_compressed(str(out_dir / f"f{fidx:05d}_bead_mask.npz"),
                            mask=bead_cands[0]['mask'],
                            roi_origin=np.array([x1, y1]))


# ─── Main ─────────────────────────────────────────────────────────────────────

def main():
    # guard
    if not SVO_PATH.exists():
        print(f"SVO not found: {SVO_PATH}"); sys.exit(1)
    if not CKPT_PATH.exists():
        print(f"Checkpoint not found: {CKPT_PATH}")
        print("  Run: curl -L -o <ckpt_path> "
              "https://dl.fbaipublicfiles.com/segment_anything_2/092824/sam2.1_hiera_small.pt")
        sys.exit(1)

    ts      = datetime.now().strftime("%Y%m%d_%H%M%S")
    out_dir = OUT_BASE / ts
    out_dir.mkdir(parents=True, exist_ok=True)
    device  = "cuda" if torch.cuda.is_available() else "cpu"

    # ── Open SVO to get total frames ──────────────────────────────────────────
    print("Opening SVO to probe total frames...")
    zed = sl.Camera()
    p   = sl.InitParameters()
    p.set_from_svo_file(str(SVO_PATH))
    p.svo_real_time_mode = False
    p.depth_mode         = sl.DEPTH_MODE.ULTRA
    p.coordinate_units   = sl.UNIT.METER
    if zed.open(p) != sl.ERROR_CODE.SUCCESS:
        print("ZED open failed"); sys.exit(1)

    cam_info    = zed.get_camera_information()
    calib       = cam_info.camera_configuration.calibration_parameters
    fx          = calib.left_cam.fx
    fy          = calib.left_cam.fy
    cx_cam      = calib.left_cam.cx
    cy_cam      = calib.left_cam.cy
    W           = cam_info.camera_configuration.resolution.width
    H           = cam_info.camera_configuration.resolution.height
    total_frames = zed.get_svo_number_of_frames()
    cam_model    = cam_info.camera_model
    cam_serial   = cam_info.serial_number

    # ── mm/px at typical depth = 0.447m (validated from prior sessions) ───────
    FIXED_Z      = 0.447
    mm_per_px_nominal = FIXED_Z * 1000.0 / fx

    print(f"\n{'='*60}")
    print("SVO CAMERA REPORT")
    print(f"{'='*60}")
    print(f"  SVO:        {SVO_PATH.name}")
    print(f"  Camera:     {cam_model}  S/N {cam_serial}")
    print(f"  Resolution: {W} x {H}")
    print(f"  Total frames: {total_frames:,}")
    print(f"  Intrinsics (left rectified):")
    print(f"    fx = {fx:.4f} px")
    print(f"    fy = {fy:.4f} px")
    print(f"    cx = {cx_cam:.2f} px")
    print(f"    cy = {cy_cam:.2f} px")
    print(f"  mm/px at FIXED_Z={FIXED_Z}m: {mm_per_px_nominal:.5f} mm/px")
    print(f"  Depth sensitivity: d(mm/px)/d(Z) = 1/fx = {1000.0/fx:.5f} mm/px per mm depth")
    print(f"{'='*60}\n")

    # ── Build frame list ──────────────────────────────────────────────────────
    active_end    = int(total_frames * SEARCH_WIN)
    evenly_spaced = np.linspace(200, active_end - 200, N_SAMPLE).astype(int).tolist()
    diag_frames   = [100, 300, active_end // 2,
                     total_frames - 500, total_frames - 200]
    frame_list    = sorted(set(evenly_spaced + diag_frames))
    frame_list    = [f for f in frame_list if 0 < f < total_frames - 50]
    print(f"Frame list ({len(frame_list)} frames): {frame_list}\n")

    # ── Load SAM2 ─────────────────────────────────────────────────────────────
    print("Loading SAM 2.1 hiera_small...", end="", flush=True)
    sam2     = build_sam2(MODEL_CFG, str(CKPT_PATH),
                          device=device, apply_postprocessing=False)
    mask_gen = SAM2AutomaticMaskGenerator(sam2, **AMG_PARAMS)
    print(f" Done  [{device.upper()}]\n")

    img_mat   = sl.Mat()
    depth_mat = sl.Mat()
    csv_rows  = []
    timing_rows = []
    depth_samples = []   # collect per-frame depth for mm/px analysis

    for fidx in frame_list:
        print(f"{'─'*55}")
        print(f"Frame {fidx:5d} / {total_frames-1}")
        t0 = time.perf_counter()

        zed.set_svo_position(fidx)
        if zed.grab() != sl.ERROR_CODE.SUCCESS:
            print("  grab failed\n"); continue

        zed.retrieve_image(img_mat,   sl.VIEW.LEFT)
        zed.retrieve_measure(depth_mat, sl.MEASURE.DEPTH)

        bgr   = cv2.cvtColor(img_mat.get_data()[:, :, :3], cv2.COLOR_RGB2BGR)
        depth = depth_mat.get_data()

        # nozzle anchor
        nozzle = find_nozzle(bgr)
        if nozzle is None:
            print("  nozzle: fallback to centre")
            nozzle = (W // 2, H // 2)
        else:
            print(f"  nozzle: px=({nozzle[0]}, {nozzle[1]})")

        # ROI crop
        roi_cx = nozzle[0] + ROI_DX
        roi_cy = nozzle[1] + ROI_DY
        bgr_roi,   x1, y1 = crop_roi(bgr,   roi_cx, roi_cy, ROI_SIZE)
        depth_roi          = depth[y1:y1+ROI_SIZE, x1:x1+ROI_SIZE].copy()

        # depth → mm/px (live, per-frame)
        valid_d = depth_roi[np.isfinite(depth_roi) & (depth_roi > 0)]
        if len(valid_d) > 0:
            z_med      = float(np.median(valid_d))
            mm_per_px  = z_med * 1000.0 / fx
            depth_samples.append(z_med)
            print(f"  depth: {z_med:.4f}m → mm/px = {mm_per_px:.5f}"
                  f"  (Δ from nominal: {(mm_per_px - mm_per_px_nominal)*1000:.3f} µm/px)")
        else:
            z_med, mm_per_px = FIXED_Z, mm_per_px_nominal
            print(f"  depth: invalid → using nominal {mm_per_px_nominal:.5f} mm/px")

        # SAM2
        t_sam    = time.perf_counter()
        rgb_roi  = cv2.cvtColor(bgr_roi, cv2.COLOR_BGR2RGB)
        torch.cuda.empty_cache()
        with torch.inference_mode():
            masks = mask_gen.generate(rgb_roi)
        t_sam_ms = (time.perf_counter() - t_sam) * 1000
        print(f"  SAM2: {len(masks)} masks  ({t_sam_ms:.0f}ms)")

        # filter → measure
        bead_cands = filter_bead_masks(masks)
        all_wd     = []
        total_cone = 0

        for i, cand in enumerate(bead_cands[:1]):   # top-1 by aspect ratio
            wd, cone_removed = measure_bead_width(cand['mask'] * 255)
            total_cone += cone_removed
            all_wd.extend(wd)
            ws = [w['width'] for w in wd]

            if ws:
                w_arr  = np.array(ws, dtype=float)
                w_mean = w_arr.mean()
                w_std  = w_arr.std()
                w_med  = float(np.median(w_arr))
                w_mm   = w_mean * mm_per_px if mm_per_px else None
                unc    = compute_uncertainty(w_arr, mm_per_px, z_med, fx)

                print(f"  BEAD: {w_mean:.1f}±{w_std:.1f}px"
                      f"  = {w_mm:.3f}mm" if w_mm else "")
                print(f"        CI95={unc['ci95_mm']}mm  u_w={unc['u_w_mm']}mm"
                      f"  aspect={cand['aspect']:.1f}  solidity={cand['solidity']:.2f}"
                      f"  iou={cand['iou']:.3f}  stability={cand['stability']:.3f}")
                print(f"        n_scanlines_kept={len(ws)}  cone_removed={cone_removed}")

                csv_rows.append({
                    'frame':           fidx,
                    'width_mean_px':   round(w_mean, 3),
                    'width_std_px':    round(w_std, 3),
                    'width_median_px': round(w_med, 3),
                    'width_min_px':    int(w_arr.min()),
                    'width_max_px':    int(w_arr.max()),
                    'width_mean_mm':   round(w_mm, 4) if w_mm else '',
                    'width_std_mm':    round(w_std * mm_per_px, 4) if mm_per_px else '',
                    'ci95_px':         unc['ci95_px'],
                    'ci95_mm':         unc['ci95_mm'],
                    'u_w_mm':          unc['u_w_mm'],
                    'mm_per_px':       round(mm_per_px, 6) if mm_per_px else '',
                    'depth_m':         round(z_med, 5),
                    'aspect':          round(cand['aspect'], 2),
                    'solidity':        round(cand['solidity'], 3),
                    'sam2_iou':        round(cand['iou'], 4),
                    'sam2_stability':  round(cand['stability'], 4),
                    'n_scanlines':     len(ws),
                    'cone_removed':    cone_removed,
                    'bead_long_px':    round(cand['long_px'], 1),
                    'nozzle_px_x':     nozzle[0] if nozzle else '',
                    'nozzle_px_y':     nozzle[1] if nozzle else '',
                    'roi_x1':          x1,
                    'roi_y1':          y1,
                })
            else:
                print("  BEAD: no width pairs found")

        t_total_ms = (time.perf_counter() - t0) * 1000
        timing_rows.append({'frame': fidx,
                             'sam2_ms':  round(t_sam_ms, 1),
                             'total_ms': round(t_total_ms, 1)})

        save_vis(bgr, bgr_roi, x1, y1, nozzle, bead_cands,
                 all_wd, mm_per_px, out_dir, fidx)
        print()

    zed.close()

    # ── Write CSVs ────────────────────────────────────────────────────────────
    if csv_rows:
        csv_path = out_dir / "bead_width_t3.csv"
        with open(csv_path, 'w', newline='') as f:
            w = csv.DictWriter(f, fieldnames=list(csv_rows[0].keys()))
            w.writeheader(); w.writerows(csv_rows)
        print(f"\nCSV: {csv_path}")

    if timing_rows:
        with open(out_dir / "timing_t3.csv", 'w', newline='') as f:
            w = csv.DictWriter(f, fieldnames=list(timing_rows[0].keys()))
            w.writeheader(); w.writerows(timing_rows)

    # ── mm/px summary ─────────────────────────────────────────────────────────
    print(f"\n{'='*60}")
    print("mm/px FACTOR REPORT")
    print(f"{'='*60}")
    print(f"  Camera:     fx={fx:.4f} px  (ZED2i, HD1080 rectified left)")
    print(f"  Formula:    mm_per_px = Z_median_m * 1000 / fx")
    print(f"  Nominal Z:  {FIXED_Z:.3f} m  → nominal mm/px = {mm_per_px_nominal:.5f}")
    if depth_samples:
        arr = np.array(depth_samples)
        print(f"  Live Z:     {arr.mean():.4f} ± {arr.std():.4f} m"
              f"  (range {arr.min():.3f}–{arr.max():.3f})")
        live_mmpx = arr.mean() * 1000.0 / fx
        print(f"  Live mm/px: {live_mmpx:.5f}  "
              f"(Δ from nominal: {abs(live_mmpx - mm_per_px_nominal)*1000:.2f} µm/px)")
        print(f"\n  Sensitivity breakdown:")
        print(f"    ΔZ=1mm  → Δ(mm/px) = {1.0/fx:.5f} mm/px  "
              f"({1.0/fx/mm_per_px_nominal*100:.3f}% of nominal)")
        print(f"    ΔZ=5mm  → Δ(mm/px) = {5.0/fx:.5f} mm/px")
        print(f"    ΔZ=10mm → Δ(mm/px) = {10.0/fx:.5f} mm/px")
    print(f"\n  GUM uncertainty budget at Z={FIXED_Z}m:")
    print(f"    u_Z  = {U_Z}m  → rel = {U_Z/FIXED_Z*100:.2f}%")
    print(f"    u_px = {U_PX}px (mask boundary) → affects each scanline width")
    print(f"    u_fx = {U_FX}px (factory calib)")

    # bead width summary
    if csv_rows:
        mm_vals = [r['width_mean_mm'] for r in csv_rows if r['width_mean_mm'] != '']
        if mm_vals:
            arr = np.array(mm_vals)
            print(f"\n{'='*60}")
            print("BEAD WIDTH SUMMARY")
            print(f"{'='*60}")
            print(f"  Frames measured: {len(arr)} / {len(frame_list)}")
            print(f"  Width: {arr.mean():.3f} ± {arr.std():.3f} mm  "
                  f"(range {arr.min():.3f}–{arr.max():.3f})")
            print(f"  Median: {np.median(arr):.3f} mm")
            u_vals = [r['u_w_mm'] for r in csv_rows if r['u_w_mm'] != '']
            if u_vals:
                u_arr = np.array(u_vals)
                print(f"  GUM u_w: {u_arr.mean():.4f} ± {u_arr.std():.4f} mm")
            iou_vals = [r['sam2_iou'] for r in csv_rows]
            stab_vals = [r['sam2_stability'] for r in csv_rows]
            print(f"  SAM2 pred_iou: {np.mean(iou_vals):.3f} ± {np.std(iou_vals):.3f}")
            print(f"  SAM2 stability: {np.mean(stab_vals):.3f} ± {np.std(stab_vals):.3f}")

    if timing_rows:
        t_all   = np.mean([t['total_ms'] for t in timing_rows])
        t_sam   = np.mean([t['sam2_ms']  for t in timing_rows])
        print(f"\n  Timing: SAM2={t_sam:.0f}ms  total={t_all:.0f}ms/frame")

    print(f"\nOutput: {out_dir}")


if __name__ == "__main__":
    main()
