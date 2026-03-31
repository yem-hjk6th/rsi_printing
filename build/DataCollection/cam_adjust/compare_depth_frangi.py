"""
compare_depth_frangi.py — Option 1 (ZED depth slice) vs Option 2 (Sato/Frangi)
vs SAM2 baseline, on the same frames from 20260324_174844.svo2

Accuracy metric: agreement with SAM2 width ± tolerance, and internal CV.
Note: SAM2 is the reference (not ground truth), caliper = 6.92mm is ground truth.

Run: launch_t3.py (sets DLL paths)
"""

import os, sys, csv, cv2, time
import numpy as np
from pathlib import Path
from datetime import datetime

import pyzed.sl as sl
from skimage.filters import sato, frangi

# ─── Paths ───────────────────────────────────────────────────────────────────
SVO_PATH   = Path(r"C:\Users\888y9\Desktop\rsi_printing\recorded_data\20260324_174844\recording_20260324_174844.svo2")
SAM2_CSV   = Path(r"C:\Users\888y9\Desktop\rsi_printing\build\DataCollection\labeling\post_labeling\t3\20260327_042450\bead_width_t3.csv")
OUT_BASE   = Path(r"C:\Users\888y9\Desktop\rsi_printing\build\DataCollection\labeling\post_labeling\compare")

# ─── ROI config (must match run_sam2_t3.py exactly) ──────────────────────────
ROI_SIZE        = 224
ROI_DX          = -30
ROI_DY          = 260       # Fix A: was 350 → targets current layer not old stack
SCAR_ZONE_RATIO = 0.40

# ─── Depth slice params ───────────────────────────────────────────────────────
# Fix B: anchor depth on ROI 10th-percentile (nearest surface in ROI = current bead)
# instead of the nozzle mount body which is ~57mm closer to the camera.
DEPTH_BEHIND_MM  = 8           # include pixels up to 8mm BEHIND the bead surface
DEPTH_FRONT_MM   = 20          # include pixels up to 20mm IN FRONT (toward camera)
MORPH_OPEN_K     = 3           # morphological opening to remove depth noise

# ─── Frangi / Sato params ────────────────────────────────────────────────────
# Bead width in pixels: ~6.92mm / 0.265mm/px ≈ 26px  → sigmas 4–16
VESSEL_SIGMAS    = range(3, 20, 2)   # odd steps covers 3–19 px scale
VESSEL_THRESH_PC = 85                # percentile threshold on vesselness map
# minimum scanline width to count (px)
WIDTH_MIN, WIDTH_MAX = 2, 60
N_SCANLINES = 40

CALIPER_MM = 6.92   # ground truth from physical measurement


# ─── Helpers ─────────────────────────────────────────────────────────────────

def find_nozzle(bgr):
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    for lo, hi in [([100,100,80],[130,255,255]), ([5,150,180],[25,255,255])]:
        mask = cv2.inRange(hsv, np.array(lo), np.array(hi))
        k = cv2.getStructuringElement(cv2.MORPH_RECT,(7,7))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k, iterations=3)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  k, iterations=2)
        cnts,_ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        valid  = [(c,cv2.contourArea(c)) for c in cnts if 1000<cv2.contourArea(c)<300000]
        if valid:
            valid.sort(key=lambda t:t[1], reverse=True)
            c=valid[0][0]; x,y,w,h=cv2.boundingRect(c)
            return (x+w//2, y+h)
    return None


def crop_roi(img, cx, cy, size=224):
    H,W = img.shape[:2]; half=size//2
    x1=max(0,min(cx-half,W-size)); y1=max(0,min(cy-half,H-size))
    return img[y1:y1+size, x1:x1+size].copy(), x1, y1


def _find_runs(arr):
    d=np.diff(arr.astype(np.int16))
    starts=list(np.where(d>0)[0]+1); ends=list(np.where(d<0)[0]+1)
    if not starts and len(arr)>0 and arr[0]>0: starts=[0]
    if not ends   and len(arr)>0 and arr[-1]>0: ends=[len(arr)]
    return list(zip(starts,ends))


def mask_to_widths(mask_uint8):
    """Measure scanline widths from a binary mask (same logic as production)."""
    ys,xs = np.where(mask_uint8>0)
    if len(xs)<5: return []
    dx,dy = xs.max()-xs.min(), ys.max()-ys.min()
    raw=[]
    scar_y = int(ROI_SIZE * SCAR_ZONE_RATIO)
    if dx>=dy:
        for sx in np.linspace(xs.min()+2,xs.max()-2,N_SCANLINES).astype(int):
            col = mask_uint8[scar_y:, sx]
            for s,e in _find_runs(col):
                w=e-s
                if WIDTH_MIN<=w<=WIDTH_MAX: raw.append(w)
    else:
        for sy in np.linspace(max(ys.min()+2,scar_y),ys.max()-2,N_SCANLINES).astype(int):
            row = mask_uint8[sy,:]
            for s,e in _find_runs(row):
                w=e-s
                if WIDTH_MIN<=w<=WIDTH_MAX: raw.append(w)
    if not raw: return []
    ws=np.array(raw); med=np.median(ws)
    return [w for w in raw if w >= med*0.55]  # cone filter


# ─── Option 1: Depth Slice ────────────────────────────────────────────────────

def depth_slice_mask(depth_full, bgr_full, nozzle_px, roi_x1, roi_y1):
    """
    Fix B: anchor depth on ROI 10th-percentile (nearest surface = current bead top).
    Keep pixels within [anchor - DEPTH_BEHIND_MM, anchor + DEPTH_FRONT_MM].
    Returns binary mask (ROI coords) + anchor_z_m.
    """
    depth_roi = depth_full[roi_y1:roi_y1+ROI_SIZE, roi_x1:roi_x1+ROI_SIZE].copy()
    valid_roi = depth_roi[np.isfinite(depth_roi) & (depth_roi > 0.1) & (depth_roi < 2.0)]
    if len(valid_roi) < 30:
        return None, None
    # 10th percentile = nearest surface in ROI (the top of the current bead)
    anchor_z = float(np.percentile(valid_roi, 10))

    lo = anchor_z - DEPTH_BEHIND_MM / 1000.0
    hi = anchor_z + DEPTH_FRONT_MM  / 1000.0

    mask = (np.isfinite(depth_roi) & (depth_roi > lo) & (depth_roi < hi)).astype(np.uint8) * 255

    # morphological cleanup: remove noise, keep elongated blobs
    k_open  = cv2.getStructuringElement(cv2.MORPH_RECT,(MORPH_OPEN_K,MORPH_OPEN_K))
    k_close = cv2.getStructuringElement(cv2.MORPH_RECT,(5,5))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  k_open,  iterations=1)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k_close, iterations=2)

    # keep only largest connected component below scar zone
    scar_y = int(ROI_SIZE * SCAR_ZONE_RATIO)
    mask[:scar_y, :] = 0
    cnts,_ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not cnts: return None, nozzle_z
    best = max(cnts, key=cv2.contourArea)
    if cv2.contourArea(best) < 20: return None, nozzle_z
    clean = np.zeros_like(mask)
    cv2.drawContours(clean,[best],-1,255,-1)
    return clean, nozzle_z


# ─── Option 2: Sato + Frangi ─────────────────────────────────────────────────

def vesselness_mask(bgr_roi, depth_roi):
    """
    Combine Sato (bright ridges = bead) + Frangi (tubular) on grayscale ROI.
    Also weight by depth proximity if available.
    Returns binary mask (ROI coords).
    """
    gray = cv2.cvtColor(bgr_roi, cv2.COLOR_BGR2GRAY).astype(np.float64)/255.0

    # CLAHE normalise for lighting variance
    clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8,8))
    gray_eq = clahe.apply((gray*255).astype(np.uint8)).astype(np.float64)/255.0

    # Sato: detects ridges (bead = bright elongated ridge)
    s_bright = sato(gray_eq, sigmas=VESSEL_SIGMAS, black_ridges=False)
    s_dark   = sato(gray_eq, sigmas=VESSEL_SIGMAS, black_ridges=True)

    # Frangi: tubular / vessel-like
    f_map    = frangi(gray_eq, sigmas=VESSEL_SIGMAS,
                      alpha=0.5, beta=0.5, gamma=15, black_ridges=False)

    # combine: max of normalised maps
    def norm(x):
        mn,mx=x.min(),x.max()
        return (x-mn)/(mx-mn+1e-12)

    combined = np.maximum(norm(s_bright), norm(f_map))

    # optional depth weighting: down-weight pixels too far from median ROI depth
    valid_d = depth_roi[np.isfinite(depth_roi) & (depth_roi>0) & (depth_roi<2.0)]
    if len(valid_d) > 50:
        z_ref = np.median(valid_d)
        z_weight = np.exp(-((depth_roi - z_ref)**2)/(0.030**2))  # sigma=30mm
        z_weight[~np.isfinite(depth_roi) | (depth_roi<=0)] = 0.5
        combined = combined * z_weight

    # threshold
    thresh = np.percentile(combined[combined>0], VESSEL_THRESH_PC) if combined.max()>0 else 0
    mask = (combined >= thresh).astype(np.uint8) * 255

    # exclude scar zone, morph cleanup
    scar_y = int(ROI_SIZE * SCAR_ZONE_RATIO)
    mask[:scar_y,:] = 0
    k = cv2.getStructuringElement(cv2.MORPH_RECT,(3,3))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  k, iterations=1)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k, iterations=2)

    # keep largest component
    cnts,_ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not cnts: return None, combined
    # prefer most elongated component (not necessarily largest)
    def elongation(c):
        if len(c)<5: return 0
        _,(rw,rh),_ = cv2.minAreaRect(c)
        return max(rw,rh)/(min(rw,rh)+1e-6)
    best = max(cnts, key=elongation)
    if cv2.contourArea(best)<15: return None, combined
    clean=np.zeros_like(mask); cv2.drawContours(clean,[best],-1,255,-1)
    return clean, combined


# ─── Visualization ────────────────────────────────────────────────────────────

def save_comparison(bgr_full, bgr_roi, depth_roi, x1, y1, nozzle,
                    mask_depth, mask_vessel, sam2_row,
                    w_depth_px, w_vessel_px, mm_per_px,
                    out_dir, fidx):

    # depth colormap of ROI
    d_vis = depth_roi.copy()
    d_vis[~np.isfinite(d_vis)] = 0
    d_norm = cv2.normalize(d_vis, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
    d_color = cv2.applyColorMap(d_norm, cv2.COLORMAP_JET)

    def overlay_mask(base, mask, color):
        out = base.copy()
        if mask is not None:
            out[mask>0] = (np.array(base[mask>0])*0.4 + np.array(color)*0.6).astype(np.uint8)
        return out

    # row 1: raw ROI | depth colour | depth-mask | vessel-mask
    r_raw   = cv2.resize(bgr_roi, (224,224))
    r_dep   = cv2.resize(d_color, (224,224))
    r_dmask = overlay_mask(r_raw, mask_depth,  [0,200,0])
    r_vmask = overlay_mask(r_raw, mask_vessel, [0,200,255])

    def label(img, txt, color=(255,255,255)):
        out = img.copy()
        cv2.putText(out, txt, (4,16), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0,0,0), 3)
        cv2.putText(out, txt, (4,16), cv2.FONT_HERSHEY_SIMPLEX, 0.42, color, 1)
        return out

    def fmt(w_px, tag):
        if w_px is None: return f"{tag}: no det"
        w_mm = w_px * mm_per_px if mm_per_px else 0
        return f"{tag}: {w_px:.1f}px={w_mm:.2f}mm"

    sam2_str = ""
    if sam2_row:
        sam2_str = f"SAM2: {sam2_row['width_mean_px']}px={sam2_row['width_mean_mm']}mm"

    r_raw   = label(r_raw,   f"f{fidx} raw")
    r_dep   = label(r_dep,   "depth (JET)")
    r_dmask = label(r_dmask, fmt(w_depth_px,  "DEPTH"), (0,255,0))
    r_vmask = label(r_vmask, fmt(w_vessel_px, "VESSEL"), (0,255,255))

    row1 = np.hstack([r_raw, r_dep, r_dmask, r_vmask])

    # annotation row
    ann = np.zeros((30, row1.shape[1], 3), dtype=np.uint8)
    txt = f"Frame {fidx}  |  {sam2_str}  |  caliper={CALIPER_MM}mm"
    cv2.putText(ann, txt, (4,20), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200,200,200), 1)

    panel = np.vstack([ann, row1])
    cv2.imwrite(str(out_dir/f"f{fidx:05d}_compare.jpg"), panel,
                [cv2.IMWRITE_JPEG_QUALITY, 90])


# ─── Main ─────────────────────────────────────────────────────────────────────

def main():
    ts      = datetime.now().strftime("%Y%m%d_%H%M%S")
    out_dir = OUT_BASE / ts
    out_dir.mkdir(parents=True, exist_ok=True)

    # ── Load SAM2 reference ───────────────────────────────────────────────────
    sam2_ref = {}
    if SAM2_CSV.exists():
        with open(SAM2_CSV) as f:
            for row in csv.DictReader(f):
                fidx = int(row['frame'])
                sam2_ref[fidx] = {
                    'width_mean_px': float(row['width_mean_px']),
                    'width_mean_mm': float(row['width_mean_mm']) if row['width_mean_mm'] else None,
                    'depth_m':       float(row['depth_m']),
                    'mm_per_px':     float(row['mm_per_px']) if row['mm_per_px'] else None,
                }
    print(f"SAM2 reference loaded: {len(sam2_ref)} frames\n")

    # ── Open SVO ──────────────────────────────────────────────────────────────
    zed = sl.Camera()
    p   = sl.InitParameters()
    p.set_from_svo_file(str(SVO_PATH))
    p.svo_real_time_mode = False
    p.depth_mode         = sl.DEPTH_MODE.NEURAL
    p.coordinate_units   = sl.UNIT.METER
    if zed.open(p) != sl.ERROR_CODE.SUCCESS:
        print("ZED open failed"); sys.exit(1)

    calib  = zed.get_camera_information().camera_configuration.calibration_parameters
    fx     = calib.left_cam.fx
    total  = zed.get_svo_number_of_frames()
    W      = zed.get_camera_information().camera_configuration.resolution.width
    H      = zed.get_camera_information().camera_configuration.resolution.height
    print(f"SVO: {SVO_PATH.name}  frames={total}  fx={fx:.2f}")
    print(f"Resolution: {W}x{H}\n")

    # run on ALL frames that SAM2 detected + non-detected frames for completeness
    # use same full list from the SAM2 run
    frame_list = sorted(set(
        list(sam2_ref.keys()) +
        [100, 200, 277, 296, 300, 315, 469, 2566]  # SAM2 missed these
    ))

    img_mat   = sl.Mat()
    depth_mat = sl.Mat()

    rows_depth  = []
    rows_vessel = []
    timing      = []

    for fidx in frame_list:
        if fidx >= total: continue
        print(f"{'─'*55}")
        print(f"Frame {fidx:5d}")
        t0 = time.perf_counter()

        zed.set_svo_position(fidx)
        if zed.grab() != sl.ERROR_CODE.SUCCESS:
            print("  grab failed"); continue

        zed.retrieve_image(img_mat,     sl.VIEW.LEFT)
        zed.retrieve_measure(depth_mat, sl.MEASURE.DEPTH)

        bgr   = cv2.cvtColor(img_mat.get_data()[:,:,:3], cv2.COLOR_RGB2BGR)
        depth = depth_mat.get_data()

        nozzle = find_nozzle(bgr) or (W//2, H//2)
        roi_cx = nozzle[0] + ROI_DX
        roi_cy = nozzle[1] + ROI_DY
        bgr_roi,   x1, y1 = crop_roi(bgr,   roi_cx, roi_cy, ROI_SIZE)
        depth_roi          = depth[y1:y1+ROI_SIZE, x1:x1+ROI_SIZE].copy()

        # live mm/px from ROI depth
        valid_d = depth_roi[np.isfinite(depth_roi) & (depth_roi>0.1) & (depth_roi<1.5)]
        z_med      = float(np.median(valid_d)) if len(valid_d)>50 else 0.447
        mm_per_px  = z_med * 1000.0 / fx

        sam2_row = sam2_ref.get(fidx)

        # ── Option 1: Depth slice ──────────────────────────────────────────────
        t1 = time.perf_counter()
        mask_depth, anchor_z = depth_slice_mask(depth, bgr, nozzle, x1, y1)
        t_depth_ms = (time.perf_counter()-t1)*1000

        w_depth_px = None
        if mask_depth is not None:
            wlist = mask_to_widths(mask_depth)
            if wlist:
                w_depth_px = float(np.mean(wlist))

        # ── Option 2: Sato + Frangi ────────────────────────────────────────────
        t2 = time.perf_counter()
        mask_vessel, vessel_map = vesselness_mask(bgr_roi, depth_roi)
        t_vessel_ms = (time.perf_counter()-t2)*1000

        w_vessel_px = None
        if mask_vessel is not None:
            wlist2 = mask_to_widths(mask_vessel)
            if wlist2:
                w_vessel_px = float(np.mean(wlist2))

        t_total_ms = (time.perf_counter()-t0)*1000

        # ── Print frame result ─────────────────────────────────────────────────
        def fmt_mm(w_px):
            if w_px is None: return "no det"
            return f"{w_px:.1f}px = {w_px*mm_per_px:.3f}mm"

        anchor_str = f"{anchor_z:.3f}m" if anchor_z else "n/a"
        print(f"  anchor_z(10p)={anchor_str}  ROI_z={z_med:.3f}m  mm/px={mm_per_px:.5f}")
        print(f"  DEPTH:  {fmt_mm(w_depth_px)}  ({t_depth_ms:.1f}ms)")
        print(f"  VESSEL: {fmt_mm(w_vessel_px)}  ({t_vessel_ms:.1f}ms)")
        if sam2_row:
            print(f"  SAM2:   {sam2_row['width_mean_px']:.1f}px = "
                  f"{sam2_row['width_mean_mm'] or '?'}mm  (ref)")

        # ── Store rows ─────────────────────────────────────────────────────────
        base = dict(frame=fidx, mm_per_px=round(mm_per_px,6), depth_roi_m=round(z_med,4),
                    anchor_z_m=round(anchor_z,4) if anchor_z else '',
                    sam2_width_px=sam2_row['width_mean_px'] if sam2_row else '',
                    sam2_width_mm=sam2_row['width_mean_mm'] if sam2_row else '',
                    caliper_mm=CALIPER_MM)

        rows_depth.append({**base,
            'method':'depth_slice',
            'detected': 1 if w_depth_px else 0,
            'width_px': round(w_depth_px,3) if w_depth_px else '',
            'width_mm': round(w_depth_px*mm_per_px,4) if w_depth_px else '',
            'time_ms':  round(t_depth_ms,1),
        })
        rows_vessel.append({**base,
            'method':'sato_frangi',
            'detected': 1 if w_vessel_px else 0,
            'width_px': round(w_vessel_px,3) if w_vessel_px else '',
            'width_mm': round(w_vessel_px*mm_per_px,4) if w_vessel_px else '',
            'time_ms':  round(t_vessel_ms,1),
        })
        timing.append(dict(frame=fidx, depth_ms=round(t_depth_ms,1),
                           vessel_ms=round(t_vessel_ms,1), total_ms=round(t_total_ms,1)))

        save_comparison(bgr, bgr_roi, depth_roi, x1, y1, nozzle,
                        mask_depth, mask_vessel, sam2_row,
                        w_depth_px, w_vessel_px, mm_per_px, out_dir, fidx)
        print()

    zed.close()

    # ── Write CSVs ────────────────────────────────────────────────────────────
    all_rows = rows_depth + rows_vessel
    if all_rows:
        csv_path = out_dir / "comparison.csv"
        with open(csv_path,'w',newline='') as f:
            w = csv.DictWriter(f, fieldnames=list(all_rows[0].keys()))
            w.writeheader(); w.writerows(all_rows)

    if timing:
        with open(out_dir/"timing.csv",'w',newline='') as f:
            w = csv.DictWriter(f, fieldnames=list(timing[0].keys()))
            w.writeheader(); w.writerows(timing)

    # ── Accuracy report ───────────────────────────────────────────────────────
    print(f"\n{'='*60}")
    print("ACCURACY REPORT")
    print(f"{'='*60}")
    print(f"  Caliper ground truth: {CALIPER_MM} mm")
    print(f"  SAM2 reference: {len(sam2_ref)} frames\n")

    for tag, rows in [("DEPTH SLICE", rows_depth), ("SATO+FRANGI", rows_vessel)]:
        det  = [r for r in rows if r['detected']==1]
        miss = [r for r in rows if r['detected']==0]
        n_total = len(rows)

        print(f"  [{tag}]")
        print(f"    Detection rate: {len(det)}/{n_total} ({len(det)/n_total*100:.0f}%)")

        if det:
            ww = np.array([float(r['width_mm']) for r in det])
            print(f"    Width: {ww.mean():.3f} ± {ww.std():.3f} mm  "
                  f"(median={np.median(ww):.3f}  range=[{ww.min():.2f},{ww.max():.2f}])")
            print(f"    CV:    {ww.std()/ww.mean()*100:.1f}%")

            # bias vs caliper
            bias = ww.mean() - CALIPER_MM
            print(f"    Bias vs caliper ({CALIPER_MM}mm): {bias:+.3f}mm")

            # agreement with SAM2 (frames where both have values)
            pairs = [(float(r['width_mm']), float(r['sam2_width_mm']))
                     for r in det
                     if r['sam2_width_mm'] != '' and r['sam2_width_mm'] is not None]
            if pairs:
                m_arr  = np.array([p[0] for p in pairs])
                s_arr  = np.array([p[1] for p in pairs])
                diff   = np.abs(m_arr - s_arr)
                print(f"    vs SAM2 ({len(pairs)} matched frames):")
                print(f"      |diff| mean={diff.mean():.3f}mm  median={np.median(diff):.3f}mm")
                print(f"      within ±0.5mm: {(diff<=0.5).sum()}/{len(diff)} "
                      f"({(diff<=0.5).mean()*100:.0f}%)")
                print(f"      within ±1.0mm: {(diff<=1.0).sum()}/{len(diff)} "
                      f"({(diff<=1.0).mean()*100:.0f}%)")
                print(f"      within ±2.0mm: {(diff<=2.0).sum()}/{len(diff)} "
                      f"({(diff<=2.0).mean()*100:.0f}%)")
                corr = np.corrcoef(m_arr, s_arr)[0,1] if len(pairs)>2 else float('nan')
                print(f"      Pearson r vs SAM2: {corr:.3f}")

        # timing
        t_arr = np.array([r['time_ms'] for r in rows])
        print(f"    Speed: {t_arr.mean():.1f} ± {t_arr.std():.1f} ms/frame")
        print()

    # SAM2 stats for comparison
    s_mm = [v['width_mean_mm'] for v in sam2_ref.values() if v['width_mean_mm']]
    if s_mm:
        s = np.array(s_mm)
        print(f"  [SAM2 baseline (reference)]")
        print(f"    Detection rate: {len(s_mm)}/{len(sam2_ref)} "
              f"({len(s_mm)/len(sam2_ref)*100:.0f}%)")
        print(f"    Width: {s.mean():.3f} ± {s.std():.3f} mm  "
              f"(median={np.median(s):.3f})")
        print(f"    CV:    {s.std()/s.mean()*100:.1f}%")
        print(f"    Bias vs caliper: {s.mean()-CALIPER_MM:+.3f}mm")
        print(f"    Speed: ~1867ms/frame")

    print(f"\n  Output: {out_dir}")


if __name__ == "__main__":
    main()
