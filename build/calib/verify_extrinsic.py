#!/usr/bin/env python3
"""
Verification of extrinsic calibration results produced by extrinsic_extraction.py.

Reads calibration_result.npz + the original sync CSV, runs full verification,
prints report to console AND writes verification_report.md into the result folder.

Usage:
    python verify_extrinsic.py                         # auto-detect latest res/<ts>/
    python verify_extrinsic.py res/20260331_143000      # specify result dir
    python verify_extrinsic.py res/20260331_143000 --csv path/to/data.csv
"""

import sys, csv, io
import numpy as np
import cv2
from pathlib import Path
from collections import defaultdict


SCRIPT_DIR = Path(__file__).resolve().parent
RES_DIR = SCRIPT_DIR / "res"

# KUKA Tool3 offset (flange → TCP) in mm — update if tool changes
TOOL_X_MM = 225.55
TOOL_Y_MM = -0.6
TOOL_Z_MM = 72.83


# ─── geometry (must match extrinsic_extraction.py) ───────────────────────────

def euler_to_R(a, b, c):
    a, b, c = np.radians([a, b, c])
    Rz = np.array([[np.cos(a),-np.sin(a),0],[np.sin(a),np.cos(a),0],[0,0,1]])
    Ry = np.array([[np.cos(b),0,np.sin(b)],[0,1,0],[-np.sin(b),0,np.cos(b)]])
    Rx = np.array([[1,0,0],[0,np.cos(c),-np.sin(c)],[0,np.sin(c),np.cos(c)]])
    return Rz @ Ry @ Rx

def hmat(R, t):
    T = np.eye(4); T[:3,:3] = R; T[:3,3] = np.ravel(t)
    return T

def orthogonalize_rotation(R):
    U, _, Vt = np.linalg.svd(R)
    R_orth = U @ Vt
    if np.linalg.det(R_orth) < 0:
        U[:, -1] *= -1
        R_orth = U @ Vt
    return R_orth


# ─── CSV loading (supports flat + multi-marker, same as extrinsic_extraction) ─

def _detect_csv_format(header):
    if "rvec_x" in header:
        return "flat", None
    for col in header:
        if col.startswith("m") and col.endswith("_rvec_x"):
            mid = int(col[1:].split("_", 1)[0])
            return "multi", mid
    return None, None

def _available_marker_ids(header):
    ids = []
    for col in header:
        if col.startswith("m") and col.endswith("_rvec_x"):
            ids.append(int(col[1:].split("_", 1)[0]))
    return sorted(set(ids))

def load(csv_path, marker_id=None):
    with open(csv_path, encoding="utf-8") as f:
        rows = list(csv.DictReader(f))
    if not rows:
        return []

    header = list(rows[0].keys())
    fmt, auto_mid = _detect_csv_format(header)

    if fmt == "multi":
        all_mids = _available_marker_ids(header)
        if marker_id is not None:
            mid = marker_id
        else:
            counts = {m: sum(1 for r in rows if r.get(f"m{m}_tvec_z_m","").strip())
                      for m in all_mids}
            mid = max(counts, key=counts.get)
        rv_keys = [f"m{mid}_rvec_x", f"m{mid}_rvec_y", f"m{mid}_rvec_z"]
        tv_keys = [f"m{mid}_tvec_x_m", f"m{mid}_tvec_y_m", f"m{mid}_tvec_z_m"]
        valid_key = f"m{mid}_tvec_z_m"
    elif fmt == "flat":
        rv_keys = ["rvec_x", "rvec_y", "rvec_z"]
        tv_keys = ["tvec_x_m", "tvec_y_m", "tvec_z_m"]
        valid_key = "tvec_z_m"
    else:
        sys.exit("Cannot detect CSV format")

    samples = []
    for r in rows:
        if not r.get(valid_key, "").strip():
            continue
        R_g = euler_to_R(float(r["robot_a_deg"]), float(r["robot_b_deg"]), float(r["robot_c_deg"]))
        t_g = np.array([float(r["robot_x_mm"]), float(r["robot_y_mm"]), float(r["robot_z_mm"])]) / 1000.0
        rv = np.array([float(r[rv_keys[0]]), float(r[rv_keys[1]]), float(r[rv_keys[2]])])
        tv = np.array([float(r[tv_keys[0]]), float(r[tv_keys[1]]), float(r[tv_keys[2]])])
        R_t, _ = cv2.Rodrigues(rv)
        if R_t[2, 2] > 0:
            continue
        samples.append((R_g, t_g, R_t, tv))

    # dedup
    groups = defaultdict(list)
    for s in samples:
        k = tuple(np.round(s[1]*1000).astype(int)) + tuple(np.round(cv2.Rodrigues(s[0])[0].ravel(), 3))
        groups[k].append(s)
    unique = []
    for k, grp in groups.items():
        if len(grp) == 1:
            unique.append(grp[0])
        else:
            Rs = [g[2] for g in grp]
            best_i, best_score = 0, 1e9
            for i in range(len(Rs)):
                score = sum(np.arccos(np.clip((np.trace(Rs[i].T @ Rs[j])-1)/2, -1, 1))
                            for j in range(len(Rs)) if j != i)
                if score < best_score:
                    best_i, best_score = i, score
            unique.append(grp[best_i])
    return unique


def pairwise_filter(samples, angle_tol=8.0):
    n = len(samples)
    if n < 5:
        return samples
    scores = np.zeros(n)
    for i in range(n):
        T_gi = hmat(samples[i][0], samples[i][1])
        T_ci = hmat(samples[i][2], samples[i][3])
        for j in range(i+1, n):
            T_gj = hmat(samples[j][0], samples[j][1])
            T_cj = hmat(samples[j][2], samples[j][3])
            A = np.linalg.inv(T_gj) @ T_gi
            B = T_cj @ np.linalg.inv(T_ci)
            ang_a = np.degrees(np.arccos(np.clip((np.trace(A[:3,:3])-1)/2, -1, 1)))
            ang_b = np.degrees(np.arccos(np.clip((np.trace(B[:3,:3])-1)/2, -1, 1)))
            if abs(ang_a - ang_b) < angle_tol:
                scores[i] += 1; scores[j] += 1
    threshold = np.median(scores)
    keep = [i for i in range(n) if scores[i] >= threshold]
    return [samples[i] for i in keep]


def back_err(samples, T_c2g):
    pts = []
    for Rg, tg, Rt, tt in samples:
        T = hmat(Rg, tg) @ T_c2g @ hmat(Rt, tt)
        pts.append(T[:3, 3])
    pts = np.array(pts)
    errs = np.linalg.norm(pts - pts.mean(0), axis=1) * 1000
    return float(errs.mean()), float(errs.max()), errs, pts


METHODS = {
    "Tsai":      cv2.CALIB_HAND_EYE_TSAI,
    "Park":      cv2.CALIB_HAND_EYE_PARK,
    "Horaud":    cv2.CALIB_HAND_EYE_HORAUD,
    "Andreff":   cv2.CALIB_HAND_EYE_ANDREFF,
    "Daniilidis": cv2.CALIB_HAND_EYE_DANIILIDIS,
}


# ─── resolve inputs ─────────────────────────────────────────────────────────

def find_latest_result_dir():
    """Find the most recent timestamped subfolder in res/ that has calibration_result.npz."""
    if not RES_DIR.exists():
        return None
    dirs = sorted(
        [d for d in RES_DIR.iterdir() if d.is_dir() and (d / "calibration_result.npz").exists()],
        reverse=True,
    )
    return dirs[0] if dirs else None


def parse_csv_from_result_txt(result_dir):
    """Read calibration_result.txt to find the original CSV path."""
    txt = result_dir / "calibration_result.txt"
    if not txt.exists():
        return None
    for line in txt.read_text(encoding="utf-8").splitlines():
        if line.startswith("csv = "):
            return Path(line.split("= ", 1)[1].strip())
    return None


def parse_marker_id_from_result_txt(result_dir):
    """Read calibration_result.txt to find marker_id if present."""
    txt = result_dir / "calibration_result.txt"
    if not txt.exists():
        return None
    for line in txt.read_text(encoding="utf-8").splitlines():
        if line.startswith("marker_id = "):
            return int(line.split("= ", 1)[1].strip())
    return None


# ─── tee output: print + capture for md ──────────────────────────────────────

class TeeWriter:
    """Captures all printed output for writing to markdown file."""
    def __init__(self):
        self.buf = io.StringIO()

    def print(self, *args, **kwargs):
        line = io.StringIO()
        print(*args, file=line, **kwargs)
        text = line.getvalue()
        sys.stdout.write(text)
        self.buf.write(text)

    def get_text(self):
        return self.buf.getvalue()


# ─── main ────────────────────────────────────────────────────────────────────

def main():
    import argparse
    parser = argparse.ArgumentParser(description="Verify extrinsic calibration result")
    parser.add_argument("result_dir", nargs="?", default=None,
                        help="Path to res/<timestamp>/ (default: latest)")
    parser.add_argument("--csv", type=str, default=None,
                        help="Override CSV path (default: read from calibration_result.txt)")
    parser.add_argument("--marker-id", type=int, default=None,
                        help="Marker ID for multi-marker CSV")
    args = parser.parse_args()

    # resolve result dir
    if args.result_dir:
        result_dir = Path(args.result_dir)
        if not result_dir.is_absolute():
            result_dir = SCRIPT_DIR / result_dir
    else:
        result_dir = find_latest_result_dir()
        if result_dir is None:
            sys.exit("No result directories found in res/. Run extrinsic_extraction.py first.")

    result_dir = result_dir.resolve()
    npz_path = result_dir / "calibration_result.npz"
    if not npz_path.exists():
        sys.exit(f"calibration_result.npz not found in {result_dir}")

    # load calibration result
    res = np.load(str(npz_path), allow_pickle=True)
    T_cal = res["T_cam2gripper"]
    cal_method = str(res["method"][0]) if "method" in res else "unknown"

    # resolve CSV
    if args.csv:
        csv_path = Path(args.csv).resolve()
    else:
        csv_path = parse_csv_from_result_txt(result_dir)
        # fallback: if stored path doesn't exist, try sync_robot_aruco.csv in result dir
        if csv_path is None or not csv_path.exists():
            local_csv = result_dir / "sync_robot_aruco.csv"
            if local_csv.exists():
                csv_path = local_csv
            elif csv_path is None:
                sys.exit("Cannot determine CSV path. Use --csv to specify.")
    if not csv_path.exists():
        sys.exit(f"CSV not found: {csv_path}")

    marker_id = args.marker_id or parse_marker_id_from_result_txt(result_dir)

    # load & filter data
    samples_raw = load(str(csv_path), marker_id=marker_id)
    samples = pairwise_filter(samples_raw)

    if len(samples) < 3:
        sys.exit(f"Not enough valid poses ({len(samples)})")

    # ── output ────────────────────────────────────────────────────────────
    out = TeeWriter()
    P = out.print

    P("=" * 60)
    P("  验算报告  /  VERIFICATION REPORT")
    P("=" * 60)
    P(f"  Result dir : {result_dir}")
    P(f"  CSV        : {csv_path}")
    P(f"  Cal method : {cal_method}")

    # ── 1. 数据概况 ──
    P(f"\n{'─'*60}")
    P("1. 数据概况 (Data Summary)")
    P(f"{'─'*60}")
    P(f"  去重后: {len(samples_raw)} 位姿")
    P(f"  Pairwise filter 后: {len(samples)} 位姿")
    tvz = [s[3][2]*1000 for s in samples]
    P(f"  tvec_z 范围: [{min(tvz):.0f}, {max(tvz):.0f}] mm")

    # ── 2. 旋转矩阵验证 ──
    P(f"\n{'─'*60}")
    P("2. 旋转矩阵验证 (Rotation Matrix Check)")
    P(f"{'─'*60}")
    R = T_cal[:3, :3]
    det_val = np.linalg.det(R)
    orth_val = np.linalg.norm(R @ R.T - np.eye(3))
    P(f"  det(R) = {det_val:.8f}  (应=1.0)")
    P(f"  ||R·R^T - I|| = {orth_val:.2e}  (应≈0)")
    orth_ok = abs(det_val - 1) < 0.001 and orth_val < 1e-6
    P(f"  → {'✓ 正交' if orth_ok else '✗ 异常'}")

    # ── 3. 5种方法对比 ──
    P(f"\n{'─'*60}")
    P(f"3. 五种方法对比 ({len(samples)} poses, pairwise filtered)")
    P(f"{'─'*60}")
    Rg = [s[0] for s in samples]; tg = [s[1].reshape(3, 1) for s in samples]
    Rt = [s[2] for s in samples]; tt = [s[3].reshape(3, 1) for s in samples]

    P(f"  {'Method':<12} {'Mean':>6} {'Max':>6}  {'Tx':>8} {'Ty':>8} {'Tz':>8}  |t|")
    P(f"  {'─'*12} {'─'*6} {'─'*6}  {'─'*8} {'─'*8} {'─'*8}  {'─'*6}")

    all_results = {}
    best_method_name = None
    best_me = 1e9
    for name, flag in METHODS.items():
        try:
            R_, t_ = cv2.calibrateHandEye(Rg, tg, Rt, tt, method=flag)
            R_ = orthogonalize_rotation(R_)
            T_ = hmat(R_, t_.ravel())
            me_i, mx_i, _, _ = back_err(samples, T_)
            tx, ty, tz = T_[:3, 3] * 1000
            tn = np.linalg.norm(T_[:3, 3]) * 1000
            mark = " ◄" if name == cal_method else ""
            P(f"  {name:<12} {me_i:6.1f} {mx_i:6.1f}  {tx:8.1f} {ty:8.1f} {tz:8.1f}  {tn:6.1f}{mark}")
            all_results[name] = T_
            if me_i < best_me:
                best_me = me_i
                best_method_name = name
        except Exception:
            P(f"  {name:<12}  FAILED")

    # consensus check
    robust = ["Park", "Horaud", "Daniilidis"]
    robust_ts = [all_results[n][:3, 3]*1000 for n in robust if n in all_results]
    if len(robust_ts) >= 2:
        robust_ts = np.array(robust_ts)
        r_spread = np.max(robust_ts, axis=0) - np.min(robust_ts, axis=0)
        P(f"\n  Park/Horaud/Daniilidis 一致性:")
        P(f"    ΔX={r_spread[0]:.1f}mm  ΔY={r_spread[1]:.1f}mm  ΔZ={r_spread[2]:.1f}mm  total={np.linalg.norm(r_spread):.1f}mm")

    # ── 4. 逐点回代误差 ──
    P(f"\n{'─'*60}")
    P("4. 逐点回代误差 (Back-substitution Error)")
    P(f"{'─'*60}")
    me, mx, errs, pts = back_err(samples, T_cal)
    P(f"  {'#':>3}  {'Err':>7}  {'World X':>8} {'World Y':>8} {'World Z':>8}  Robot XYZ")
    for i in range(len(samples)):
        p = pts[i] * 1000
        rp = samples[i][1] * 1000
        P(f"  {i+1:3d}  {errs[i]:6.1f}mm  ({p[0]:7.1f}, {p[1]:7.1f}, {p[2]:7.1f})  ({rp[0]:.0f},{rp[1]:.0f},{rp[2]:.0f})")

    mean_pt = pts.mean(0) * 1000
    std_pt = pts.std(0) * 1000
    P(f"\n  Marker 世界坐标均值: ({mean_pt[0]:.1f}, {mean_pt[1]:.1f}, {mean_pt[2]:.1f}) mm")
    P(f"  Marker 世界坐标 std:  ({std_pt[0]:.1f}, {std_pt[1]:.1f}, {std_pt[2]:.1f}) mm")
    P(f"  均值误差: {me:.1f} mm,  最大: {mx:.1f} mm")

    # ── 5. Leave-One-Out 交叉验证 ──
    P(f"\n{'─'*60}")
    P("5. Leave-One-Out 交叉验证")
    P(f"{'─'*60}")
    # use the method from calibration result for LOO
    loo_method = cal_method if cal_method in METHODS else (best_method_name or "Daniilidis")
    loo_flag = METHODS[loo_method]
    P(f"  LOO method: {loo_method}")

    loo_errs = []
    loo_ts = []
    for i in range(len(samples)):
        train = [s for j, s in enumerate(samples) if j != i]
        Rg_t = [s[0] for s in train]; tg_t = [s[1].reshape(3, 1) for s in train]
        Rt_t = [s[2] for s in train]; tt_t = [s[3].reshape(3, 1) for s in train]
        try:
            R_, t_ = cv2.calibrateHandEye(Rg_t, tg_t, Rt_t, tt_t, method=loo_flag)
            T_loo = hmat(R_, t_.ravel())
            loo_ts.append(T_loo[:3, 3] * 1000)
            s = samples[i]
            pt_test = (hmat(s[0], s[1]) @ T_loo @ hmat(s[2], s[3]))[:3, 3]
            train_pts = [(hmat(s2[0], s2[1]) @ T_loo @ hmat(s2[2], s2[3]))[:3, 3] for s2 in train]
            mean_train = np.mean(train_pts, axis=0)
            err = np.linalg.norm(pt_test - mean_train) * 1000
            loo_errs.append(err)
        except Exception:
            loo_errs.append(np.nan)

    loo_errs = np.array(loo_errs)
    valid = loo_errs[~np.isnan(loo_errs)]
    loo_mean = float(np.mean(valid)) if len(valid) > 0 else 999.0
    P(f"  LOO 误差: mean={loo_mean:.1f}mm, max={np.max(valid):.1f}mm, std={np.std(valid):.1f}mm")

    loo_ts = np.array(loo_ts)
    t_std = loo_ts.std(axis=0) if len(loo_ts) > 1 else np.zeros(3)
    t_std_norm = float(np.linalg.norm(t_std))
    P(f"  T_cam2gripper 稳定性 (LOO std): ({t_std[0]:.1f}, {t_std[1]:.1f}, {t_std[2]:.1f}) mm")

    sorted_idx = np.argsort(loo_errs)[::-1]
    P(f"\n  最差 LOO 位姿:")
    for idx in sorted_idx[:5]:
        if np.isnan(loo_errs[idx]):
            continue
        rp = samples[idx][1] * 1000
        P(f"    Pose {idx+1:2d}: {loo_errs[idx]:6.1f}mm  robot=({rp[0]:.0f},{rp[1]:.0f},{rp[2]:.0f})")

    # ── 6. 物理合理性检查 ──
    P(f"\n{'─'*60}")
    P("6. 物理合理性检查 (Physical Plausibility)")
    P(f"{'─'*60}")
    t_mm = T_cal[:3, 3] * 1000
    t_norm = np.linalg.norm(t_mm)

    R_cal = T_cal[:3, :3]
    B_rad = np.arcsin(np.clip(-R_cal[2, 0], -1, 1))
    A_rad = np.arctan2(R_cal[1, 0], R_cal[0, 0])
    C_rad = np.arctan2(R_cal[2, 1], R_cal[2, 2])
    A_deg, B_deg, C_deg = np.degrees([A_rad, B_rad, C_rad])

    P(f"  T_cam2gripper (相机→TCP):")
    P(f"    平移: ({t_mm[0]:.1f}, {t_mm[1]:.1f}, {t_mm[2]:.1f}) mm")
    P(f"    |t| = {t_norm:.1f} mm")
    P(f"    旋转 A,B,C = ({A_deg:.1f}°, {B_deg:.1f}°, {C_deg:.1f}°)")

    tool_norm = np.sqrt(TOOL_X_MM**2 + TOOL_Y_MM**2 + TOOL_Z_MM**2)
    P(f"\n  Tool3 (TCP offset from flange):")
    P(f"    ({TOOL_X_MM}, {TOOL_Y_MM}, {TOOL_Z_MM}) mm, |t|={tool_norm:.1f} mm")

    T_f2t = np.eye(4)
    T_f2t[:3, 3] = [TOOL_X_MM / 1000, TOOL_Y_MM / 1000, TOOL_Z_MM / 1000]
    T_t2f = np.linalg.inv(T_f2t)
    T_cam2flange = T_t2f @ T_cal
    t_flange = T_cam2flange[:3, 3] * 1000

    P(f"\n  T_cam2flange (相机→法兰):")
    P(f"    平移: ({t_flange[0]:.1f}, {t_flange[1]:.1f}, {t_flange[2]:.1f}) mm")
    P(f"    |t| = {np.linalg.norm(t_flange):.1f} mm")
    P(f"    (相机与法兰之间的物理距离)")

    # ── 7. 总评 ──
    P(f"\n{'='*60}")
    P("  总评 (FINAL ASSESSMENT)")
    P(f"{'='*60}")

    score = 0; notes = []

    if me < 5:
        score += 3; notes.append(f"误差极低 (<5mm)")
    elif me < 10:
        score += 2; notes.append(f"误差良好 ({me:.1f}mm)")
    elif me < 20:
        score += 1; notes.append(f"误差偏高 ({me:.1f}mm)")
    else:
        notes.append(f"误差过高 ({me:.1f}mm)")

    if loo_mean < 10:
        score += 2; notes.append(f"LOO验证好 ({loo_mean:.1f}mm)")
    elif loo_mean < 20:
        score += 1; notes.append(f"LOO验证中等 ({loo_mean:.1f}mm)")
    else:
        notes.append(f"LOO验证差 ({loo_mean:.1f}mm)")

    if t_std_norm < 10:
        score += 2; notes.append(f"T变换稳定 (std={t_std_norm:.1f}mm)")
    elif t_std_norm < 30:
        score += 1; notes.append(f"T变换基本稳定 (std={t_std_norm:.1f}mm)")
    else:
        notes.append(f"T变换不稳定 (std={t_std_norm:.1f}mm)")

    max_ang = 0
    for i in range(len(samples)):
        for j in range(i+1, len(samples)):
            v = np.clip((np.trace(samples[i][0].T @ samples[j][0])-1)/2, -1, 1)
            a = np.degrees(np.arccos(v))
            if a > max_ang:
                max_ang = a
    if max_ang > 60:
        score += 2; notes.append(f"旋转充分 ({max_ang:.0f}°)")
    elif max_ang > 30:
        score += 1; notes.append(f"旋转勉强够 ({max_ang:.0f}°)")
    else:
        notes.append(f"旋转不足 ({max_ang:.0f}°)")

    if len(samples) >= 15:
        score += 1; notes.append(f"位姿数足够 ({len(samples)})")
    elif len(samples) >= 10:
        score += 0.5; notes.append(f"位姿数偏少 ({len(samples)})")
    else:
        notes.append(f"位姿数不足 ({len(samples)})")

    quality_map = {10: "★★★★★ 优秀", 9: "★★★★☆ 很好", 8: "★★★★ 好", 7: "★★★☆ 中上",
                   6: "★★★ 可用", 5: "★★☆ 中等", 4: "★★ 偏低", 3: "★☆ 差"}
    quality = quality_map.get(int(score), "★ 需改进" if score < 3 else "★★★★ 好")

    P(f"\n  质量评分: {score:.0f}/10 → {quality}")
    for n in notes:
        P(f"    · {n}")

    P(f"\n  改进建议:")
    if max_ang < 60:
        P(f"    ▸ 增加更大旋转角度的位姿 (当前最大 {max_ang:.0f}°, 建议 > 60°)")
    if len(samples) < 20:
        P(f"    ▸ 增加更多位姿 (当前 {len(samples)}, 建议 > 20)")
    if loo_mean > 10:
        P(f"    ▸ 检查并移除异常位姿")
    if t_std_norm > 20:
        P(f"    ▸ 数据质量需提升 (使用更大 ArUco 标记或减少相机-标记距离)")
    P()

    # ── Write verification_report.md ──
    md_path = result_dir / "verification_report.md"
    with open(md_path, "w", encoding="utf-8") as f:
        f.write("```\n")
        f.write(out.get_text())
        f.write("```\n")
    print(f"[SAVE] {md_path}")


if __name__ == "__main__":
    main()
