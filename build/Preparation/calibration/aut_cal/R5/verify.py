#!/usr/bin/env python3
"""
Verification script for hand-eye calibration result.

Checks:
1. Data quality: tvec range, rvec consistency, duplicate detection
2. Rotation matrix validity (orthogonality, det=+1)
3. Back-substitution error per pose (marker should map to same world point)
4. Physical plausibility of T_cam2gripper
5. All 5 methods comparison (no outlier removal)
6. Re-run best method without iterative removal for comparison
7. Cross-validation: leave-one-out
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


def load_raw(csv_path):
    """Load all rows without any filtering."""
    with open(csv_path, encoding="utf-8") as f:
        rows = list(csv.DictReader(f))
    samples = []
    for r in rows:
        if not r.get("tvec_z_m","").strip():
            continue
        R_g = euler_to_R(float(r["robot_a_deg"]), float(r["robot_b_deg"]), float(r["robot_c_deg"]))
        t_g = np.array([float(r["robot_x_mm"]), float(r["robot_y_mm"]), float(r["robot_z_mm"])]) / 1000.0
        rv = np.array([float(r["rvec_x"]), float(r["rvec_y"]), float(r["rvec_z"])])
        tv = np.array([float(r["tvec_x_m"]), float(r["tvec_y_m"]), float(r["tvec_z_m"])])
        R_t, _ = cv2.Rodrigues(rv)
        robot_abc = (float(r["robot_a_deg"]), float(r["robot_b_deg"]), float(r["robot_c_deg"]))
        robot_xyz = (float(r["robot_x_mm"]), float(r["robot_y_mm"]), float(r["robot_z_mm"]))
        samples.append({
            'R_g': R_g, 't_g': t_g, 'R_t': R_t, 't_t': tv, 'rv': rv,
            'robot_xyz': robot_xyz, 'robot_abc': robot_abc,
            'flipped': R_t[2,2] > 0
        })
    return samples


def load_filtered(csv_path):
    """Load with all filters (same as calibrate.py)."""
    samples_raw = load_raw(csv_path)
    samples = [(s['R_g'], s['t_g'], s['R_t'], s['t_t']) for s in samples_raw if not s['flipped']]
    
    # dedup
    from collections import defaultdict
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
                if score < best_score:
                    best_i, best_score = i, score
            unique.append(grp[best_i])
    return unique


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
    print("=" * 60)
    print("  HAND-EYE CALIBRATION VERIFICATION")
    print("=" * 60)
    print(f"\nCSV: {csv_path}")

    # ── 1. Data Quality ──────────────────────────────────────
    print("\n" + "─" * 60)
    print("1. DATA QUALITY")
    print("─" * 60)
    raw = load_raw(str(csv_path))
    print(f"  Total rows: {len(raw)}")
    
    tvec_z = [s['t_t'][2] for s in raw]
    tvec_x = [s['t_t'][0] for s in raw]
    tvec_y = [s['t_t'][1] for s in raw]
    print(f"  tvec_x range: [{min(tvec_x)*1000:.1f}, {max(tvec_x)*1000:.1f}] mm")
    print(f"  tvec_y range: [{min(tvec_y)*1000:.1f}, {max(tvec_y)*1000:.1f}] mm")
    print(f"  tvec_z range: [{min(tvec_z)*1000:.1f}, {max(tvec_z)*1000:.1f}] mm")
    
    n_flip = sum(1 for s in raw if s['flipped'])
    print(f"  Flipped (R[2,2]>0): {n_flip}/{len(raw)}")
    
    # Check for in-transit captures (consecutive rows with same rvec but slightly different robot pose)
    n_transit = 0
    for i in range(1, len(raw)):
        rv_same = np.allclose(raw[i]['rv'], raw[i-1]['rv'], atol=1e-6)
        robot_diff = np.linalg.norm(np.array(raw[i]['robot_xyz']) - np.array(raw[i-1]['robot_xyz']))
        if rv_same and robot_diff > 1.0:  # same camera, robot moved > 1mm
            n_transit += 1
    print(f"  Possible in-transit: {n_transit}")

    # Unique robot poses
    poses = set()
    for s in raw:
        k = tuple(np.round(np.array(s['robot_xyz'])).astype(int))
        poses.add(k)
    print(f"  Unique robot positions: {len(poses)}")

    # ── 2. Filtered Data ──────────────────────────────────────
    print("\n" + "─" * 60)
    print("2. FILTERED DATA")
    print("─" * 60)
    samples = load_filtered(str(csv_path))
    print(f"  After flip filter + dedup: {len(samples)} poses")

    # Rotation diversity
    max_ang = 0
    for i in range(len(samples)):
        for j in range(i+1, len(samples)):
            v = np.clip((np.trace(samples[i][0].T @ samples[j][0])-1)/2, -1, 1)
            a = np.degrees(np.arccos(v))
            if a > max_ang: max_ang = a
    print(f"  Max rotation spread: {max_ang:.1f} deg")

    # Translation diversity
    ts = np.array([s[1] for s in samples]) * 1000
    xr = ts[:,0].max() - ts[:,0].min()
    yr = ts[:,1].max() - ts[:,1].min()
    zr = ts[:,2].max() - ts[:,2].min()
    print(f"  Translation spread: X={xr:.0f}mm, Y={yr:.0f}mm, Z={zr:.0f}mm")

    # ── 3. All Methods (no outlier removal) ────────────────────
    print("\n" + "─" * 60)
    print("3. ALL METHODS (no outlier removal)")
    print("─" * 60)
    Rg = [s[0] for s in samples]
    tg = [s[1].reshape(3,1) for s in samples]
    Rt = [s[2] for s in samples]
    tt = [s[3].reshape(3,1) for s in samples]

    results = {}
    print(f"  {'Method':<12} {'Mean mm':>8} {'Max mm':>8}  T_xyz (mm)")
    print("  " + "-" * 60)
    for name, flag in METHODS.items():
        try:
            R, t = cv2.calibrateHandEye(Rg, tg, Rt, tt, method=flag)
            T = hmat(R, t.ravel())
            me, mx, errs, pts = back_err(samples, T)
            tx, ty, tz = T[:3,3] * 1000
            print(f"  {name:<12} {me:8.1f} {mx:8.1f}  ({tx:.1f}, {ty:.1f}, {tz:.1f})")
            results[name] = {'T': T, 'mean': me, 'max': mx, 'errs': errs, 'pts': pts}
        except Exception as e:
            print(f"  {name:<12}   FAILED: {e}")

    # ── 4. Rotation Matrix Validity ────────────────────────────
    print("\n" + "─" * 60)
    print("4. ROTATION MATRIX VALIDITY")
    print("─" * 60)
    for name, res in results.items():
        R = res['T'][:3,:3]
        det = np.linalg.det(R)
        orth = np.linalg.norm(R @ R.T - np.eye(3))
        print(f"  {name:<12} det={det:.6f}  ||RR^T-I||={orth:.2e}  {'OK' if abs(det-1)<0.01 and orth<0.01 else 'BAD'}")

    # ── 5. Best Method Detail ──────────────────────────────────
    best_name = min(results, key=lambda n: results[n]['mean'])
    best = results[best_name]
    print("\n" + "─" * 60)
    print(f"5. BEST METHOD: {best_name}")
    print("─" * 60)
    T = best['T']
    print(f"  T_cam2gripper:")
    for row in T:
        print(f"    [{row[0]:10.6f} {row[1]:10.6f} {row[2]:10.6f} {row[3]:10.6f}]")
    
    t_mm = T[:3,3] * 1000
    print(f"\n  Translation: ({t_mm[0]:.1f}, {t_mm[1]:.1f}, {t_mm[2]:.1f}) mm")
    print(f"  |t| = {np.linalg.norm(t_mm):.1f} mm")
    
    # Extract Euler angles from R
    R = T[:3,:3]
    # ZYX order: A=atan2(R10,R00), B=asin(-R20), C=atan2(R21,R22)
    B = np.degrees(np.arcsin(np.clip(-R[2,0], -1, 1)))
    A = np.degrees(np.arctan2(R[1,0], R[0,0]))
    C = np.degrees(np.arctan2(R[2,1], R[2,2]))
    print(f"  Euler (KUKA A,B,C): ({A:.1f}, {B:.1f}, {C:.1f}) deg")

    print(f"\n  Back-substitution error per pose:")
    print(f"  {'Pose':>4}  {'Err mm':>8}  {'World XYZ (mm)':>30}")
    pts = best['pts']
    errs = best['errs']
    for i in range(len(samples)):
        p = pts[i] * 1000
        print(f"  {i+1:4d}  {errs[i]:8.1f}  ({p[0]:8.1f}, {p[1]:8.1f}, {p[2]:8.1f})")
    
    mean_pt = pts.mean(0) * 1000
    print(f"  Mean marker world position: ({mean_pt[0]:.1f}, {mean_pt[1]:.1f}, {mean_pt[2]:.1f}) mm")

    # ── 6. Method Consensus ────────────────────────────────────
    print("\n" + "─" * 60)
    print("6. METHOD CONSENSUS (translation agreement)")
    print("─" * 60)
    ts_all = []
    for name, res in results.items():
        ts_all.append(res['T'][:3,3] * 1000)
    ts_all = np.array(ts_all)
    spread = ts_all.max(0) - ts_all.min(0)
    print(f"  X spread: {spread[0]:.1f} mm")
    print(f"  Y spread: {spread[1]:.1f} mm")
    print(f"  Z spread: {spread[2]:.1f} mm")
    print(f"  Total spread: {np.linalg.norm(spread):.1f} mm")
    if np.linalg.norm(spread) < 50:
        print("  → Methods agree well (< 50mm)")
    else:
        print("  → Methods DISAGREE — data may have issues")

    # ── 7. Leave-One-Out Cross-Validation ────────────────────────
    print("\n" + "─" * 60)
    print("7. LEAVE-ONE-OUT CROSS-VALIDATION")
    print("─" * 60)
    loo_errs = []
    flag = METHODS[best_name]
    for i in range(len(samples)):
        train = [s for j, s in enumerate(samples) if j != i]
        Rg_t = [s[0] for s in train]
        tg_t = [s[1].reshape(3,1) for s in train]
        Rt_t = [s[2] for s in train]
        tt_t = [s[3].reshape(3,1) for s in train]
        try:
            R, t = cv2.calibrateHandEye(Rg_t, tg_t, Rt_t, tt_t, method=flag)
            T_loo = hmat(R, t.ravel())
            # Test on held-out sample
            s = samples[i]
            pt_i = (hmat(s[0], s[1]) @ T_loo @ hmat(s[2], s[3]))[:3,3]
            # Also compute training mean point
            train_pts = []
            for s2 in train:
                pt2 = (hmat(s2[0], s2[1]) @ T_loo @ hmat(s2[2], s2[3]))[:3,3]
                train_pts.append(pt2)
            mean_train = np.mean(train_pts, axis=0)
            err = np.linalg.norm(pt_i - mean_train) * 1000
            loo_errs.append(err)
        except:
            loo_errs.append(np.nan)
    
    loo_errs = np.array(loo_errs)
    valid = loo_errs[~np.isnan(loo_errs)]
    print(f"  LOO errors: mean={np.mean(valid):.1f}mm, max={np.max(valid):.1f}mm, std={np.std(valid):.1f}mm")
    print(f"  Per-pose LOO errors:")
    for i, e in enumerate(loo_errs):
        marker = " ← outlier" if e > np.mean(valid) + 2*np.std(valid) else ""
        print(f"    Pose {i+1:2d}: {e:8.1f} mm{marker}")

    # ── 8. Pairwise Relative Error ────────────────────────────────
    print("\n" + "─" * 60)
    print("8. PAIRWISE RELATIVE MOTION CONSISTENCY")
    print("─" * 60)
    T_c2g = best['T']
    rel_errs = []
    for i in range(len(samples)):
        for j in range(i+1, len(samples)):
            T_gi = hmat(samples[i][0], samples[i][1])
            T_gj = hmat(samples[j][0], samples[j][1])
            T_ci = hmat(samples[i][2], samples[i][3])
            T_cj = hmat(samples[j][2], samples[j][3])
            
            # Robot relative motion
            A = np.linalg.inv(T_gj) @ T_gi
            # Camera relative motion mapped through T_c2g
            B = T_c2g @ T_cj @ np.linalg.inv(T_ci) @ np.linalg.inv(T_c2g)
            
            # Rotation angle difference
            ang_a = np.degrees(np.arccos(np.clip((np.trace(A[:3,:3])-1)/2, -1, 1)))
            ang_b = np.degrees(np.arccos(np.clip((np.trace(B[:3,:3])-1)/2, -1, 1)))
            
            # Translation magnitude difference  
            dt = np.linalg.norm(A[:3,3] - B[:3,3]) * 1000
            
            rel_errs.append((i, j, abs(ang_a-ang_b), dt))
    
    ang_diffs = [r[2] for r in rel_errs]
    t_diffs = [r[3] for r in rel_errs]
    print(f"  Rotation angle diffs: mean={np.mean(ang_diffs):.2f}°, max={np.max(ang_diffs):.2f}°")
    print(f"  Translation diffs:    mean={np.mean(t_diffs):.1f}mm, max={np.max(t_diffs):.1f}mm")

    # ── 9. Summary ──────────────────────────────────────────────
    print("\n" + "=" * 60)
    print("  SUMMARY")
    print("=" * 60)
    print(f"  Data: {len(raw)} raw → {len(samples)} filtered poses")
    print(f"  Best method: {best_name}")
    print(f"  Back-sub error: {best['mean']:.1f}mm mean, {best['max']:.1f}mm max")
    print(f"  LOO error: {np.mean(valid):.1f}mm mean")
    print(f"  T_cam2gripper translation: ({t_mm[0]:.1f}, {t_mm[1]:.1f}, {t_mm[2]:.1f}) mm")
    print(f"  Method consensus spread: {np.linalg.norm(spread):.1f}mm")
    
    score = 0
    issues = []
    if best['mean'] < 10: score += 3
    elif best['mean'] < 20: score += 2
    elif best['mean'] < 50: score += 1
    else: issues.append("high back-sub error")
    
    if np.mean(valid) < 15: score += 2
    elif np.mean(valid) < 30: score += 1
    else: issues.append("high LOO error")
    
    if np.linalg.norm(spread) < 50: score += 2
    elif np.linalg.norm(spread) < 100: score += 1
    else: issues.append("methods disagree")
    
    if max_ang > 40: score += 2
    elif max_ang > 25: score += 1
    else: issues.append("low rotation diversity")
    
    if len(samples) >= 15: score += 1
    elif len(samples) < 8: issues.append("too few poses")
    
    quality = {10: "EXCELLENT", 9: "VERY GOOD", 8: "GOOD", 7: "GOOD", 
               6: "OK", 5: "OK", 4: "FAIR", 3: "FAIR"}.get(score, "POOR" if score < 3 else "GOOD")
    print(f"\n  Overall quality score: {score}/10 → {quality}")
    if issues:
        print(f"  Issues: {', '.join(issues)}")
    print()


if __name__ == "__main__":
    main()
