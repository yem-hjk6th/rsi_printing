#!/usr/bin/env python3
"""
Full verification of the calibration result (with pairwise filter, matching calibrate.py).
"""
import sys, csv
import numpy as np
import cv2
from pathlib import Path


def euler_to_R(a, b, c):
    a, b, c = np.radians([a, b, c])
    Rz = np.array([[np.cos(a),-np.sin(a),0],[np.sin(a),np.cos(a),0],[0,0,1]])
    Ry = np.array([[np.cos(b),0,np.sin(b)],[0,1,0],[-np.sin(b),0,np.cos(b)]])
    Rx = np.array([[1,0,0],[0,np.cos(c),-np.sin(c)],[0,np.sin(c),np.cos(c)]])
    return Rz @ Ry @ Rx

def hmat(R, t):
    T = np.eye(4); T[:3,:3] = R; T[:3,3] = np.ravel(t)
    return T

def load(csv_path):
    from collections import defaultdict
    with open(csv_path, encoding="utf-8") as f:
        rows = list(csv.DictReader(f))
    samples = []
    for r in rows:
        if not r.get("tvec_z_m","").strip(): continue
        R_g = euler_to_R(float(r["robot_a_deg"]), float(r["robot_b_deg"]), float(r["robot_c_deg"]))
        t_g = np.array([float(r["robot_x_mm"]), float(r["robot_y_mm"]), float(r["robot_z_mm"])]) / 1000.0
        rv = np.array([float(r["rvec_x"]), float(r["rvec_y"]), float(r["rvec_z"])])
        tv = np.array([float(r["tvec_x_m"]), float(r["tvec_y_m"]), float(r["tvec_z_m"])])
        R_t, _ = cv2.Rodrigues(rv)
        if R_t[2,2] > 0: continue
        samples.append((R_g, t_g, R_t, tv))
    # dedup
    groups = defaultdict(list)
    for s in samples:
        k = tuple(np.round(s[1]*1000).astype(int)) + tuple(np.round(cv2.Rodrigues(s[0])[0].ravel(),3))
        groups[k].append(s)
    unique = []
    for k, grp in groups.items():
        if len(grp) == 1:
            unique.append(grp[0])
        else:
            Rs = [g[2] for g in grp]
            best_i, best_score = 0, 1e9
            for i in range(len(Rs)):
                score = sum(np.arccos(np.clip((np.trace(Rs[i].T @ Rs[j])-1)/2,-1,1))
                            for j in range(len(Rs)) if j != i)
                if score < best_score: best_i, best_score = i, score
            unique.append(grp[best_i])
    return unique

def pairwise_filter(samples, angle_tol=8.0):
    n = len(samples)
    if n < 5: return samples
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
        pts.append(T[:3,3])
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

def main():
    csv_path = Path(sys.argv[1]) if len(sys.argv) > 1 else Path("sync_robot_aruco_2.csv")
    
    # Load calibration result
    res = np.load("calibration_result.npz", allow_pickle=True)
    T_cal = res['T_cam2gripper']
    
    # Load & filter data (same pipeline as calibrate.py)
    samples_raw = load(str(csv_path))
    samples = pairwise_filter(samples_raw)
    
    print("=" * 60)
    print("  验算报告  /  VERIFICATION REPORT")
    print("=" * 60)

    # ── 1. 数据概况 ──
    print(f"\n{'─'*60}")
    print("1. 数据概况 (Data Summary)")
    print(f"{'─'*60}")
    print(f"  CSV 行数: 25")
    print(f"  去重后: {len(samples_raw)} 位姿")
    print(f"  Pairwise filter 后: {len(samples)} 位姿")
    tvz = [s[3][2]*1000 for s in samples]
    print(f"  tvec_z 范围: [{min(tvz):.0f}, {max(tvz):.0f}] mm")
    print(f"  (ZED SDK 修复后，之前是 ~2000mm)")

    # ── 2. 旋转矩阵验证 ──
    print(f"\n{'─'*60}")
    print("2. 旋转矩阵验证 (Rotation Matrix Check)")
    print(f"{'─'*60}")
    R = T_cal[:3,:3]
    det = np.linalg.det(R)
    orth = np.linalg.norm(R @ R.T - np.eye(3))
    print(f"  det(R) = {det:.8f}  (应=1.0)")
    print(f"  ||R·R^T - I|| = {orth:.2e}  (应≈0)")
    print(f"  → {'✓ 正交' if abs(det-1)<0.001 and orth<1e-6 else '✗ 异常'}")

    # ── 3. 5种方法对比 (pairwise filtered) ──
    print(f"\n{'─'*60}")
    print(f"3. 五种方法对比 ({len(samples)} poses, pairwise filtered)")
    print(f"{'─'*60}")
    Rg = [s[0] for s in samples]; tg = [s[1].reshape(3,1) for s in samples]
    Rt = [s[2] for s in samples]; tt = [s[3].reshape(3,1) for s in samples]
    
    print(f"  {'Method':<12} {'Mean':>6} {'Max':>6}  {'Tx':>8} {'Ty':>8} {'Tz':>8}  |t|")
    print(f"  {'':─<12} {'mm':─>6} {'mm':─>6}  {'mm':─>8} {'mm':─>8} {'mm':─>8}  {'mm':─>4}")
    
    all_results = {}
    for name, flag in METHODS.items():
        try:
            R_, t_ = cv2.calibrateHandEye(Rg, tg, Rt, tt, method=flag)
            T_ = hmat(R_, t_.ravel())
            me, mx, _, _ = back_err(samples, T_)
            tx, ty, tz = T_[:3,3]*1000
            tn = np.linalg.norm(T_[:3,3])*1000
            mark = " ◄" if name == "Horaud" else ""
            print(f"  {name:<12} {me:6.1f} {mx:6.1f}  {tx:8.1f} {ty:8.1f} {tz:8.1f}  {tn:6.1f}{mark}")
            all_results[name] = T_
        except:
            print(f"  {name:<12}  FAILED")
    
    # Check Park/Horaud/Daniilidis consensus (robust methods)
    robust = ["Park", "Horaud", "Daniilidis"]
    robust_ts = [all_results[n][:3,3]*1000 for n in robust if n in all_results]
    if len(robust_ts) >= 2:
        robust_ts = np.array(robust_ts)
        r_spread = np.max(robust_ts, axis=0) - np.min(robust_ts, axis=0)
        print(f"\n  Park/Horaud/Daniilidis 一致性:")
        print(f"    ΔX={r_spread[0]:.1f}mm  ΔY={r_spread[1]:.1f}mm  ΔZ={r_spread[2]:.1f}mm  total={np.linalg.norm(r_spread):.1f}mm")

    # ── 4. 逐点回代误差 ──
    print(f"\n{'─'*60}")
    print("4. 逐点回代误差 (Back-substitution Error)")
    print(f"{'─'*60}")
    me, mx, errs, pts = back_err(samples, T_cal)
    print(f"  {'#':>3}  {'Err':>7}  {'World X':>8} {'World Y':>8} {'World Z':>8}  Robot XYZ")
    for i in range(len(samples)):
        p = pts[i]*1000
        rp = samples[i][1]*1000
        print(f"  {i+1:3d}  {errs[i]:6.1f}mm  ({p[0]:7.1f}, {p[1]:7.1f}, {p[2]:7.1f})  ({rp[0]:.0f},{rp[1]:.0f},{rp[2]:.0f})")
    
    mean_pt = pts.mean(0)*1000
    std_pt = pts.std(0)*1000
    print(f"\n  Marker 世界坐标均值: ({mean_pt[0]:.1f}, {mean_pt[1]:.1f}, {mean_pt[2]:.1f}) mm")
    print(f"  Marker 世界坐标 std:  ({std_pt[0]:.1f}, {std_pt[1]:.1f}, {std_pt[2]:.1f}) mm")
    print(f"  均值误差: {me:.1f} mm,  最大: {mx:.1f} mm")

    # ── 5. Leave-One-Out 交叉验证 ──
    print(f"\n{'─'*60}")
    print("5. Leave-One-Out 交叉验证")
    print(f"{'─'*60}")
    flag = METHODS["Horaud"]
    loo_errs = []
    loo_ts = []
    for i in range(len(samples)):
        train = [s for j, s in enumerate(samples) if j != i]
        Rg_t = [s[0] for s in train]; tg_t = [s[1].reshape(3,1) for s in train]
        Rt_t = [s[2] for s in train]; tt_t = [s[3].reshape(3,1) for s in train]
        try:
            R_, t_ = cv2.calibrateHandEye(Rg_t, tg_t, Rt_t, tt_t, method=flag)
            T_loo = hmat(R_, t_.ravel())
            loo_ts.append(T_loo[:3,3]*1000)
            # Test held-out sample against training mean
            s = samples[i]
            pt_test = (hmat(s[0], s[1]) @ T_loo @ hmat(s[2], s[3]))[:3,3]
            train_pts = [(hmat(s2[0], s2[1]) @ T_loo @ hmat(s2[2], s2[3]))[:3,3] for s2 in train]
            mean_train = np.mean(train_pts, axis=0)
            err = np.linalg.norm(pt_test - mean_train) * 1000
            loo_errs.append(err)
        except:
            loo_errs.append(np.nan)
    
    loo_errs = np.array(loo_errs)
    valid = loo_errs[~np.isnan(loo_errs)]
    print(f"  LOO 误差: mean={np.mean(valid):.1f}mm, max={np.max(valid):.1f}mm, std={np.std(valid):.1f}mm")
    
    # T stability across LOO folds
    loo_ts = np.array(loo_ts)
    t_std = loo_ts.std(axis=0)
    print(f"  T_cam2gripper 稳定性 (LOO std): ({t_std[0]:.1f}, {t_std[1]:.1f}, {t_std[2]:.1f}) mm")
    
    # Show worst LOO poses
    sorted_idx = np.argsort(loo_errs)[::-1]
    print(f"\n  最差 LOO 位姿:")
    for idx in sorted_idx[:5]:
        if np.isnan(loo_errs[idx]): continue
        rp = samples[idx][1]*1000
        print(f"    Pose {idx+1:2d}: {loo_errs[idx]:6.1f}mm  robot=({rp[0]:.0f},{rp[1]:.0f},{rp[2]:.0f})")

    # ── 6. 物理合理性检查 ──
    print(f"\n{'─'*60}")
    print("6. 物理合理性检查 (Physical Plausibility)")
    print(f"{'─'*60}")
    t_mm = T_cal[:3,3]*1000
    t_norm = np.linalg.norm(t_mm)
    
    R_cal = T_cal[:3,:3]
    B_rad = np.arcsin(np.clip(-R_cal[2,0], -1, 1))
    A_rad = np.arctan2(R_cal[1,0], R_cal[0,0])
    C_rad = np.arctan2(R_cal[2,1], R_cal[2,2])
    A_deg, B_deg, C_deg = np.degrees([A_rad, B_rad, C_rad])
    
    print(f"  T_cam2gripper (即相机→TCP的变换):")
    print(f"    平移: ({t_mm[0]:.1f}, {t_mm[1]:.1f}, {t_mm[2]:.1f}) mm")
    print(f"    |t| = {t_norm:.1f} mm")
    print(f"    旋转 A,B,C = ({A_deg:.1f}°, {B_deg:.1f}°, {C_deg:.1f}°)")

    # Tool3 offset
    tool_x, tool_y, tool_z = 225.55, -0.6, 72.83
    tool_norm = np.sqrt(tool_x**2 + tool_y**2 + tool_z**2)
    print(f"\n  Tool3 (TCP offset from flange):")
    print(f"    ({tool_x}, {tool_y}, {tool_z}) mm, |t|={tool_norm:.1f} mm")
    
    # T_cam2flange = T_tcp2flange * T_cam2tcp
    # T_tcp2flange = inv(T_flange2tcp) where T_flange2tcp is just translation
    T_f2t = np.eye(4)
    T_f2t[:3,3] = [tool_x/1000, tool_y/1000, tool_z/1000]
    T_t2f = np.linalg.inv(T_f2t)
    T_cam2flange = T_t2f @ T_cal
    t_flange = T_cam2flange[:3,3]*1000
    
    print(f"\n  T_cam2flange (相机→法兰):")
    print(f"    平移: ({t_flange[0]:.1f}, {t_flange[1]:.1f}, {t_flange[2]:.1f}) mm")
    print(f"    |t| = {np.linalg.norm(t_flange):.1f} mm")
    print(f"    (相机与法兰之间的物理距离)")

    # ── 7. 总评 ──
    print(f"\n{'='*60}")
    print("  总评 (FINAL ASSESSMENT)")
    print(f"{'='*60}")
    
    score = 0; notes = []
    
    # Error quality
    if me < 5: score += 3; notes.append("误差极低 (<5mm)")
    elif me < 10: score += 2; notes.append(f"误差良好 ({me:.1f}mm)")
    elif me < 20: score += 1; notes.append(f"误差偏高 ({me:.1f}mm)")
    else: notes.append(f"误差过高 ({me:.1f}mm)")
    
    # LOO
    loo_mean = np.mean(valid)
    if loo_mean < 10: score += 2; notes.append(f"LOO验证好 ({loo_mean:.1f}mm)")
    elif loo_mean < 20: score += 1; notes.append(f"LOO验证中等 ({loo_mean:.1f}mm)")
    else: notes.append(f"LOO验证差 ({loo_mean:.1f}mm)")
    
    # T stability
    t_std_norm = np.linalg.norm(t_std)
    if t_std_norm < 10: score += 2; notes.append(f"T变换稳定 (std={t_std_norm:.1f}mm)")
    elif t_std_norm < 30: score += 1; notes.append(f"T变换基本稳定 (std={t_std_norm:.1f}mm)")
    else: notes.append(f"T变换不稳定 (std={t_std_norm:.1f}mm)")
    
    # Rotation spread
    max_ang = 0
    for i in range(len(samples)):
        for j in range(i+1, len(samples)):
            v = np.clip((np.trace(samples[i][0].T @ samples[j][0])-1)/2, -1, 1)
            a = np.degrees(np.arccos(v))
            if a > max_ang: max_ang = a
    if max_ang > 60: score += 2; notes.append(f"旋转充分 ({max_ang:.0f}°)")
    elif max_ang > 30: score += 1; notes.append(f"旋转勉强够 ({max_ang:.0f}°)")
    else: notes.append(f"旋转不足 ({max_ang:.0f}°)")
    
    # N poses
    if len(samples) >= 15: score += 1; notes.append(f"位姿数足够 ({len(samples)})")
    elif len(samples) >= 10: score += 0.5; notes.append(f"位姿数偏少 ({len(samples)})")
    else: notes.append(f"位姿数不足 ({len(samples)})")
    
    quality_map = {10: "★★★★★ 优秀", 9: "★★★★☆ 很好", 8: "★★★★ 好", 7: "★★★☆ 中上",
                   6: "★★★ 可用", 5: "★★☆ 中等", 4: "★★ 偏低", 3: "★☆ 差"}
    quality = quality_map.get(int(score), "★ 需改进" if score < 3 else "★★★★ 好")
    
    print(f"\n  质量评分: {score:.0f}/10 → {quality}")
    for n in notes:
        print(f"    · {n}")
    
    print(f"\n  改进建议:")
    if max_ang < 60:
        print(f"    ▸ 增加更大旋转角度的位姿 (当前最大 {max_ang:.0f}°, 建议 > 60°)")
    if len(samples) < 20:
        print(f"    ▸ 增加更多位姿 (当前 {len(samples)}, 建议 > 20)")
    if loo_mean > 10:
        print(f"    ▸ 检查并移除异常位姿")
    if t_std_norm > 20:
        print(f"    ▸ 数据质量需提升 (使用更大 ArUco 标记或减少相机-标记距离)")
    
    print()


if __name__ == "__main__":
    main()
