#!/usr/bin/env python3
"""
Minimal hand-eye calibration.

Usage:  python calibrate.py                        # uses ./sync_robot_aruco.csv
        python calibrate.py path/to/data.csv       # specify CSV
"""

import sys, csv
import numpy as np
import cv2
from pathlib import Path


def euler_to_R(a, b, c):
    """KUKA ZYX (A,B,C) degrees → rotation matrix."""
    a, b, c = np.radians([a, b, c])
    Rz = np.array([[np.cos(a),-np.sin(a),0],[np.sin(a),np.cos(a),0],[0,0,1]])
    Ry = np.array([[np.cos(b),0,np.sin(b)],[0,1,0],[-np.sin(b),0,np.cos(b)]])
    Rx = np.array([[1,0,0],[0,np.cos(c),-np.sin(c)],[0,np.sin(c),np.cos(c)]])
    return Rz @ Ry @ Rx


def hmat(R, t):
    T = np.eye(4); T[:3,:3] = R; T[:3,3] = np.ravel(t)
    return T


def load(csv_path):
    with open(csv_path, encoding="utf-8") as f:
        rows = list(csv.DictReader(f))

    samples = []
    n_flip = 0
    for r in rows:
        if not r.get("tvec_z_m","").strip():
            continue

        R_g = euler_to_R(float(r["robot_a_deg"]), float(r["robot_b_deg"]), float(r["robot_c_deg"]))
        t_g = np.array([float(r["robot_x_mm"]), float(r["robot_y_mm"]), float(r["robot_z_mm"])]) / 1000.0

        rv = np.array([float(r["rvec_x"]), float(r["rvec_y"]), float(r["rvec_z"])])
        tv = np.array([float(r["tvec_x_m"]), float(r["tvec_y_m"]), float(r["tvec_z_m"])])
        R_t, _ = cv2.Rodrigues(rv)

        if R_t[2,2] > 0:   # flipped ArUco pose
            n_flip += 1
            continue

        samples.append((R_g, t_g, R_t, tv))

    if n_flip:
        print(f"  filtered {n_flip} flipped detections")

    # deduplicate by robot pose — keep only when rvec is consistent
    from collections import defaultdict
    groups = defaultdict(list)
    for s in samples:
        k = tuple(np.round(s[1]*1000).astype(int)) + tuple(np.round(cv2.Rodrigues(s[0])[0].ravel(),3))
        groups[k].append(s)

    unique = []
    n_inconsistent = 0
    for k, grp in groups.items():
        if len(grp) == 1:
            unique.append(grp[0])
        else:
            # pick the pair with smallest mutual angle — consistent detection
            Rs = [g[2] for g in grp]
            best_i, best_score = 0, 1e9
            for i in range(len(Rs)):
                score = sum(np.arccos(np.clip((np.trace(Rs[i].T @ Rs[j])-1)/2,-1,1))
                            for j in range(len(Rs)) if j != i)
                if score < best_score:
                    best_i, best_score = i, score
            # reject if best still has high spread
            worst = max(np.degrees(np.arccos(np.clip((np.trace(Rs[best_i].T @ Rs[j])-1)/2,-1,1)))
                        for j in range(len(Rs)) if j != best_i)
            if worst > 20:
                n_inconsistent += 1
            unique.append(grp[best_i])

    if n_inconsistent:
        print(f"  {n_inconsistent} poses had inconsistent ArUco detections (>20 deg spread)")
    if len(unique) < len(samples):
        print(f"  dedup {len(samples)} → {len(unique)}")
    return unique


def back_err(samples, T_c2g):
    pts = []
    for Rg, tg, Rt, tt in samples:
        T = hmat(Rg, tg) @ T_c2g @ hmat(Rt, tt)
        pts.append(T[:3,3])
    pts = np.array(pts)
    errs = np.linalg.norm(pts - pts.mean(0), axis=1) * 1000
    return float(errs.mean()), float(errs.max()), errs


def pairwise_filter(samples, angle_tol=8.0):
    """Remove samples with inconsistent relative motions.
    For valid pairs: rotation angle of A_ij must match rotation angle of B_ij.
    Keep samples that are consistent with the majority."""
    n = len(samples)
    if n < 5:
        return samples

    # build consistency score per sample
    scores = np.zeros(n)
    for i in range(n):
        T_gi = hmat(samples[i][0], samples[i][1])
        T_ci = hmat(samples[i][2], samples[i][3])
        for j in range(i+1, n):
            T_gj = hmat(samples[j][0], samples[j][1])
            T_cj = hmat(samples[j][2], samples[j][3])

            # relative robot motion
            A = np.linalg.inv(T_gj) @ T_gi
            ang_a = np.degrees(np.arccos(np.clip((np.trace(A[:3,:3])-1)/2, -1, 1)))

            # relative camera motion
            B = T_cj @ np.linalg.inv(T_ci)
            ang_b = np.degrees(np.arccos(np.clip((np.trace(B[:3,:3])-1)/2, -1, 1)))

            diff = abs(ang_a - ang_b)
            if diff < angle_tol:
                scores[i] += 1
                scores[j] += 1

    # keep samples with above-median consistency
    threshold = np.median(scores)
    keep = [i for i in range(n) if scores[i] >= threshold]
    removed = n - len(keep)
    if removed:
        print(f"  pairwise filter: {n} → {len(keep)} (removed {removed} inconsistent)")
    return [samples[i] for i in keep]


METHODS = {
    "Tsai":      cv2.CALIB_HAND_EYE_TSAI,
    "Park":      cv2.CALIB_HAND_EYE_PARK,
    "Horaud":    cv2.CALIB_HAND_EYE_HORAUD,
    "Andreff":   cv2.CALIB_HAND_EYE_ANDREFF,
    "Daniilidis": cv2.CALIB_HAND_EYE_DANIILIDIS,
}


def main():
    csv_path = Path(sys.argv[1]) if len(sys.argv) > 1 else Path("sync_robot_aruco.csv")
    if not csv_path.exists():
        sys.exit(f"Not found: {csv_path}")

    print(f"=== Hand-Eye Calibration ===\nCSV: {csv_path}\n")
    samples = load(str(csv_path))
    n_raw = len(samples)
    if n_raw < 3:
        sys.exit(f"Only {n_raw} valid poses, need >= 3")

    # pairwise consistency filter
    samples = pairwise_filter(samples)
    n = len(samples)
    if n < 3:
        sys.exit(f"Only {n} poses after filtering, need >= 3")

    # diversity check
    max_ang = 0
    for i in range(n):
        for j in range(i+1, n):
            v = np.clip((np.trace(samples[i][0].T @ samples[j][0])-1)/2, -1, 1)
            a = np.degrees(np.arccos(v))
            if a > max_ang: max_ang = a
    print(f"  {n} poses, rotation spread = {max_ang:.1f} deg")
    if max_ang < 30:
        print(f"  WARNING: need > 30 deg spread for good calibration")

    # solve
    Rg = [s[0] for s in samples]
    tg = [s[1].reshape(3,1) for s in samples]
    Rt = [s[2] for s in samples]
    tt = [s[3].reshape(3,1) for s in samples]

    print(f"\n{'Method':<12} {'Mean mm':>8} {'Max mm':>8}")
    print("-"*30)
    best = None
    for name, flag in METHODS.items():
        try:
            R, t = cv2.calibrateHandEye(Rg, tg, Rt, tt, method=flag)
            T = hmat(R, t.ravel())
            me, mx, _ = back_err(samples, T)
            print(f"{name:<12} {me:8.1f} {mx:8.1f}")
            if best is None or me < best[0]:
                best = (me, mx, name, T)
        except Exception:
            print(f"{name:<12}   FAILED")

    if best is None:
        sys.exit("All methods failed")

    me, mx, name, T = best

    # iterative outlier removal (stop if error increases)
    _, _, errs = back_err(samples, T)
    best_me, best_mx, best_T, best_samples = me, mx, T.copy(), list(samples)
    for iteration in range(3):
        thr = np.mean(errs) + 1.5 * np.std(errs)
        keep = [i for i in range(len(samples)) if errs[i] < thr]
        if len(keep) < 5 or len(keep) == len(samples):
            break
        removed = len(samples) - len(keep)
        samples = [samples[i] for i in keep]
        Rg = [s[0] for s in samples]
        tg = [s[1].reshape(3,1) for s in samples]
        Rt = [s[2] for s in samples]
        tt = [s[3].reshape(3,1) for s in samples]
        R, t = cv2.calibrateHandEye(Rg, tg, Rt, tt, method=METHODS[name])
        T = hmat(R, t.ravel())
        me, mx, errs = back_err(samples, T)
        print(f"  iter{iteration+1}: removed {removed} outliers → {len(samples)} poses, err={me:.1f}mm")
        if me < best_me:
            best_me, best_mx, best_T, best_samples = me, mx, T.copy(), list(samples)
        else:
            print(f"  error increased, reverting to previous ({best_me:.1f}mm)")
            break
    me, mx, T, samples = best_me, best_mx, best_T, best_samples

    print(f"\n{'='*40}")
    print(f"Best: {name}  ({len(samples)} poses)")
    print(f"Error: {me:.1f} mm mean, {mx:.1f} mm max")
    print(f"\nT_cam2gripper:\n{np.array2string(T, precision=6, suppress_small=True)}")

    if me < 10:   q = "GOOD"
    elif me < 30: q = "OK - add more poses"
    else:         q = "POOR - need more diverse poses"
    print(f"\nQuality: {q}")

    out = csv_path.parent
    np.savez(out/"calibration_result.npz", T_cam2gripper=T,
             method=np.array([name]), mean_err_mm=np.array([me]), n_poses=np.array([n]))
    with open(out/"calibration_result.txt","w") as f:
        f.write(f"method={name}\nposes={n}\nmean_err_mm={me:.3f}\nmax_err_mm={mx:.3f}\n\n")
        f.write(f"T_cam2gripper:\n{np.array2string(T, precision=8)}\n")
    print(f"Saved: {out}/calibration_result.*")


if __name__ == "__main__":
    main()
