#!/usr/bin/env python3
"""
Hand-eye calibration (eye-in-hand).

Usage:
    python calibrate.py                          # uses ./sync_robot_aruco.csv
    python calibrate.py path/to/data.csv         # specify CSV

Workflow:
    1. Run robot SRC + aut_cam_cal.py  → sync_robot_aruco.csv
    2. python calibrate.py             → see error
    3. Add more poses, re-run step 1-2 → error should decrease
"""

import sys
import csv
import numpy as np
import cv2
from pathlib import Path


# ── Helpers ──────────────────────────────────────────────────────────────────

def euler_to_R(a_deg, b_deg, c_deg):
    """KUKA ZYX intrinsic euler (A,B,C) → 3×3 rotation matrix."""
    a, b, c = np.radians([a_deg, b_deg, c_deg])
    ca, sa = np.cos(a), np.sin(a)
    cb, sb = np.cos(b), np.sin(b)
    cc, sc = np.cos(c), np.sin(c)
    Rz = np.array([[ca, -sa, 0], [sa, ca, 0], [0, 0, 1]])
    Ry = np.array([[cb, 0, sb], [0, 1, 0], [-sb, 0, cb]])
    Rx = np.array([[1, 0, 0], [0, cc, -sc], [0, sc, cc]])
    return Rz @ Ry @ Rx


def hmat(R, t):
    """Build 4×4 homogeneous matrix."""
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = R
    T[:3, 3] = np.asarray(t, dtype=np.float64).ravel()
    return T


def rot_angle(R1, R2):
    """Rotation angle (deg) between two rotation matrices."""
    v = np.clip((np.trace(R1.T @ R2) - 1.0) / 2.0, -1.0, 1.0)
    return np.degrees(np.arccos(v))


# ── Load & Filter ────────────────────────────────────────────────────────────

def load_samples(csv_path):
    """Read CSV, auto pick best marker, filter flipped poses, deduplicate."""
    with open(csv_path, "r", encoding="utf-8") as f:
        rows = list(csv.DictReader(f))

    if len(rows) < 3:
        sys.exit(f"Only {len(rows)} rows in CSV, need >= 3")

    # find marker with most detections
    header = list(rows[0].keys())
    mids = [int(c[1:].split("_")[0]) for c in header
            if c.startswith("m") and c.endswith("_rvec_x")]
    counts = {m: sum(1 for r in rows if r.get(f"m{m}_tvec_z_m", "").strip())
              for m in mids}
    mid = max(counts, key=counts.get)
    print(f"[data] marker {mid}: {counts[mid]}/{len(rows)} detections")

    # extract valid samples
    samples = []
    n_flip = 0
    for r in rows:
        if not r.get(f"m{mid}_tvec_z_m", "").strip():
            continue

        # robot pose → T_gripper2base
        x = float(r["robot_x_mm"])
        y = float(r["robot_y_mm"])
        z = float(r["robot_z_mm"])
        a = float(r["robot_a_deg"])
        b = float(r["robot_b_deg"])
        c = float(r["robot_c_deg"])
        R_g2b = euler_to_R(a, b, c)
        t_g2b = np.array([x, y, z]) / 1000.0

        # aruco → T_target2cam
        rv = np.array([float(r[f"m{mid}_rvec_{ax}"]) for ax in "xyz"])
        tv = np.array([float(r[f"m{mid}_tvec_{ax}_m"]) for ax in "xyz"])
        R_t2c, _ = cv2.Rodrigues(rv)

        # filter: marker normal must face camera (R[2,2] < 0)
        if R_t2c[2, 2] > 0:
            n_flip += 1
            continue

        samples.append((R_g2b, t_g2b, R_t2c, tv))

    if n_flip:
        print(f"[data] filtered {n_flip} flipped ArUco poses")

    # deduplicate: keep one sample per unique robot pose (round to 1mm + 0.1°)
    seen = {}
    for s in samples:
        rkey = tuple(np.round(s[1] * 1000).astype(int))            # XYZ mm
        akey = tuple(np.round(cv2.Rodrigues(s[0])[0].ravel(), 3))  # orientation
        key = rkey + akey
        seen[key] = s  # last observation wins (robot most stable)

    unique = list(seen.values())
    if len(unique) < len(samples):
        print(f"[data] deduplicated {len(samples)} → {len(unique)} unique poses")

    return unique


# ── Calibration ──────────────────────────────────────────────────────────────

METHODS = {
    "Tsai":      cv2.CALIB_HAND_EYE_TSAI,
    "Park":      cv2.CALIB_HAND_EYE_PARK,
    "Horaud":    cv2.CALIB_HAND_EYE_HORAUD,
    "Andreff":   cv2.CALIB_HAND_EYE_ANDREFF,
    "Daniilidis": cv2.CALIB_HAND_EYE_DANIILIDIS,
}


def back_sub_error(samples, T_c2g):
    """Back-substitution: T_g2b @ T_c2g @ T_t2c should be constant."""
    positions = []
    for R_g2b, t_g2b, R_t2c, t_t2c in samples:
        T = hmat(R_g2b, t_g2b) @ T_c2g @ hmat(R_t2c, t_t2c)
        positions.append(T[:3, 3])

    pts = np.array(positions)
    center = pts.mean(axis=0)
    errs = np.linalg.norm(pts - center, axis=1) * 1000.0  # m→mm
    return float(errs.mean()), float(errs.max())


def run_calibration(samples):
    """Try all 5 methods, return (mean_err_mm, method_name, T_cam2gripper)."""
    R_list = [s[0] for s in samples]
    tR_list = [s[1].reshape(3, 1) for s in samples]
    Rc_list = [s[2] for s in samples]
    tc_list = [s[3].reshape(3, 1) for s in samples]

    print(f"\n{'Method':<12} {'Mean(mm)':>9} {'Max(mm)':>9}")
    print("-" * 33)

    best = None
    for name, flag in METHODS.items():
        try:
            R, t = cv2.calibrateHandEye(
                R_list, tR_list, Rc_list, tc_list, method=flag)
            T = hmat(R, t.ravel())
            mean_e, max_e = back_sub_error(samples, T)
            print(f"{name:<12} {mean_e:9.1f} {max_e:9.1f}")
            if best is None or mean_e < best[0]:
                best = (mean_e, max_e, name, T)
        except Exception:
            print(f"{name:<12}     FAILED")

    return best


# ── Diversity check ──────────────────────────────────────────────────────────

def check_diversity(samples):
    """Check rotation diversity and sample count."""
    n = len(samples)
    max_ang = 0.0
    for i in range(n):
        for j in range(i + 1, min(n, i + 50)):  # cap comparisons
            ang = rot_angle(samples[i][0], samples[j][0])
            if ang > max_ang:
                max_ang = ang

    print(f"[info] {n} poses, max rotation spread = {max_ang:.1f} deg")

    warnings = []
    if n < 8:
        warnings.append(f"  -> need >= 8 poses (have {n})")
    if max_ang < 30:
        warnings.append(f"  -> need > 30 deg rotation spread (have {max_ang:.1f})")
    for w in warnings:
        print(w)

    return len(warnings) == 0


# ── Main ─────────────────────────────────────────────────────────────────────

def main():
    csv_path = Path(sys.argv[1]) if len(sys.argv) > 1 else Path("sync_robot_aruco.csv")
    if not csv_path.exists():
        sys.exit(f"Not found: {csv_path}")

    print(f"=== Hand-Eye Calibration ===")
    print(f"CSV: {csv_path}\n")

    samples = load_samples(str(csv_path))
    if len(samples) < 3:
        sys.exit("Not enough valid samples (need >= 3)")

    ok = check_diversity(samples)
    result = run_calibration(samples)

    if result is None:
        sys.exit("All methods failed!")

    mean_err, max_err, method, T_c2g = result

    # ── Result ───────────────────────────────────────────────────
    print(f"\n{'='*45}")
    print(f"Best method : {method}")
    print(f"Poses used  : {len(samples)}")
    print(f"Mean error  : {mean_err:.1f} mm")
    print(f"Max  error  : {max_err:.1f} mm")
    print(f"\nT_cam2gripper:")
    print(np.array2string(T_c2g, precision=6, suppress_small=True))

    if mean_err < 10:
        verdict = "GOOD"
    elif mean_err < 30:
        verdict = "OK - add more poses to improve"
    else:
        verdict = "POOR - need more diverse poses"
    print(f"\nQuality: {verdict}")
    if not ok:
        print("Tip: vary robot orientation more (change A, B, C angles)")

    # ── Save ─────────────────────────────────────────────────────
    out_dir = csv_path.parent
    np.savez(
        out_dir / "calibration_result.npz",
        T_cam2gripper=T_c2g,
        method=np.array([method]),
        mean_err_mm=np.array([mean_err]),
        n_poses=np.array([len(samples)]),
    )
    txt = out_dir / "calibration_result.txt"
    with open(txt, "w") as f:
        f.write(f"method={method}\n")
        f.write(f"poses={len(samples)}\n")
        f.write(f"mean_err_mm={mean_err:.3f}\n")
        f.write(f"max_err_mm={max_err:.3f}\n\n")
        f.write(f"T_cam2gripper:\n{np.array2string(T_c2g, precision=8)}\n")
    print(f"\nSaved: {out_dir}/calibration_result.*")


if __name__ == "__main__":
    main()
