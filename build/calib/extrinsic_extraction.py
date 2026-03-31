#!/usr/bin/env python3
"""
Hand-eye extrinsic calibration — extract T_cam2gripper from sync CSV.

Based on R6/calibrate.py with timestamped output management.

Usage:
    python extrinsic_extraction.py                          # uses ./sync_robot_aruco.csv
    python extrinsic_extraction.py path/to/data.csv         # specify CSV
    python extrinsic_extraction.py data.csv --marker-id 1   # specify marker (multi-marker CSV)
"""

import sys, csv, datetime
import numpy as np
import cv2
from pathlib import Path


SCRIPT_DIR = Path(__file__).resolve().parent
RES_DIR = SCRIPT_DIR / "res"
LOG_FILE = RES_DIR / "calibration_log.md"


# ─── geometry ────────────────────────────────────────────────────────────────

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


def orthogonalize_rotation(R):
    """Project R onto SO(3) via SVD."""
    U, _, Vt = np.linalg.svd(R)
    R_orth = U @ Vt
    if np.linalg.det(R_orth) < 0:
        U[:, -1] *= -1
        R_orth = U @ Vt
    return R_orth


# ─── CSV loading (supports both R6-flat and exp04-multi-marker formats) ──────

def _detect_csv_format(header):
    """Return ('flat', None) for R6 format, ('multi', marker_id) for multi-marker."""
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
    """Load CSV, auto-detect format, return list of (R_g, t_g, R_t, tv) tuples."""
    with open(csv_path, encoding="utf-8") as f:
        rows = list(csv.DictReader(f))
    if not rows:
        return [], "flat", None

    header = list(rows[0].keys())
    fmt, auto_mid = _detect_csv_format(header)

    if fmt == "multi":
        all_mids = _available_marker_ids(header)
        if marker_id is not None:
            if marker_id not in all_mids:
                sys.exit(f"Marker {marker_id} not in CSV columns: {all_mids}")
            mid = marker_id
        elif auto_mid is not None:
            # pick marker with most valid rows
            counts = {m: sum(1 for r in rows if r.get(f"m{m}_tvec_z_m","").strip())
                      for m in all_mids}
            mid = max(counts, key=counts.get)
        else:
            sys.exit("Cannot detect marker columns")
        rv_keys = [f"m{mid}_rvec_x", f"m{mid}_rvec_y", f"m{mid}_rvec_z"]
        tv_keys = [f"m{mid}_tvec_x_m", f"m{mid}_tvec_y_m", f"m{mid}_tvec_z_m"]
        valid_key = f"m{mid}_tvec_z_m"
        print(f"  multi-marker CSV, using marker ID={mid}")
    elif fmt == "flat":
        rv_keys = ["rvec_x", "rvec_y", "rvec_z"]
        tv_keys = ["tvec_x_m", "tvec_y_m", "tvec_z_m"]
        valid_key = "tvec_z_m"
        mid = marker_id
    else:
        sys.exit("Cannot detect CSV format (need rvec_x or m{id}_rvec_x columns)")

    samples = []
    n_flip = 0
    for r in rows:
        if not r.get(valid_key, "").strip():
            continue

        R_g = euler_to_R(float(r["robot_a_deg"]), float(r["robot_b_deg"]), float(r["robot_c_deg"]))
        t_g = np.array([float(r["robot_x_mm"]), float(r["robot_y_mm"]), float(r["robot_z_mm"])]) / 1000.0

        rv = np.array([float(r[rv_keys[0]]), float(r[rv_keys[1]]), float(r[rv_keys[2]])])
        tv = np.array([float(r[tv_keys[0]]), float(r[tv_keys[1]]), float(r[tv_keys[2]])])
        R_t, _ = cv2.Rodrigues(rv)

        if R_t[2, 2] > 0:
            n_flip += 1
            continue

        samples.append((R_g, t_g, R_t, tv))

    if n_flip:
        print(f"  filtered {n_flip} flipped detections")

    # deduplicate by robot pose
    from collections import defaultdict
    groups = defaultdict(list)
    for s in samples:
        k = tuple(np.round(s[1]*1000).astype(int)) + tuple(np.round(cv2.Rodrigues(s[0])[0].ravel(), 3))
        groups[k].append(s)

    unique = []
    n_inconsistent = 0
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
            worst = max(np.degrees(np.arccos(np.clip((np.trace(Rs[best_i].T @ Rs[j])-1)/2, -1, 1)))
                        for j in range(len(Rs)) if j != best_i)
            if worst > 20:
                n_inconsistent += 1
            unique.append(grp[best_i])

    if n_inconsistent:
        print(f"  {n_inconsistent} poses had inconsistent ArUco detections (>20 deg spread)")
    if len(unique) < len(samples):
        print(f"  dedup {len(samples)} → {len(unique)}")
    return unique, fmt, mid


# ─── evaluation ──────────────────────────────────────────────────────────────

def rot_angle_deg(R):
    """Rotation angle of R in degrees."""
    v = float(np.clip((np.trace(R) - 1.0) * 0.5, -1.0, 1.0))
    return float(np.rad2deg(np.arccos(v)))


def back_err(samples, T_c2g):
    pts = []
    for Rg, tg, Rt, tt in samples:
        T = hmat(Rg, tg) @ T_c2g @ hmat(Rt, tt)
        pts.append(T[:3, 3])
    pts = np.array(pts)
    errs = np.linalg.norm(pts - pts.mean(0), axis=1) * 1000
    return float(errs.mean()), float(errs.max()), errs, pts


def pairwise_filter(samples, angle_tol=8.0):
    """Remove samples with inconsistent relative motions."""
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
            ang_a = np.degrees(np.arccos(np.clip((np.trace(A[:3,:3])-1)/2, -1, 1)))
            B = T_cj @ np.linalg.inv(T_ci)
            ang_b = np.degrees(np.arccos(np.clip((np.trace(B[:3,:3])-1)/2, -1, 1)))
            diff = abs(ang_a - ang_b)
            if diff < angle_tol:
                scores[i] += 1
                scores[j] += 1

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


# ─── output ──────────────────────────────────────────────────────────────────

def _format_T(T):
    """4x4 matrix → fixed-width string."""
    lines = []
    for row in T:
        lines.append("  ".join(f"{v: .6f}" for v in row))
    return "\n".join(lines)


def save_results(out_dir, csv_path, name, me, mx, T, n_poses, n_raw, fmt, mid,
                 all_candidates):
    """Save .npz, .txt, and T_cam2gripper.txt into out_dir."""
    out_dir.mkdir(parents=True, exist_ok=True)

    # ── calibration_result.npz
    np.savez(
        out_dir / "calibration_result.npz",
        T_cam2gripper=T,
        method=np.array([name]),
        mean_err_mm=np.array([me]),
        max_err_mm=np.array([mx]),
        n_poses=np.array([n_poses]),
        n_raw=np.array([n_raw]),
    )

    # ── calibration_result.txt  (human-readable)
    with open(out_dir / "calibration_result.txt", "w", encoding="utf-8") as f:
        f.write(f"csv = {csv_path}\n")
        if mid is not None:
            f.write(f"marker_id = {mid}\n")
        f.write(f"csv_format = {fmt}\n")
        f.write(f"method = {name}\n")
        f.write(f"poses_used = {n_poses}\n")
        f.write(f"poses_raw = {n_raw}\n")
        f.write(f"mean_err_mm = {me:.3f}\n")
        f.write(f"max_err_mm = {mx:.3f}\n\n")
        f.write(f"T_cam2gripper:\n{_format_T(T)}\n")
        if all_candidates:
            f.write(f"\nAll methods (sorted by mean error):\n")
            for _me, _mx, _name, _ in all_candidates:
                f.write(f"  {_name:<12} mean={_me:.3f} mm  max={_mx:.3f} mm\n")

    # ── T_cam2gripper.txt  (direct load format for reconstruct_svo.py)
    with open(out_dir / "T_cam2gripper.txt", "w", encoding="utf-8") as f:
        f.write("# Camera-to-Gripper Extrinsics (T_cam2gripper)\n")
        f.write("# Format: 4x4 row-major, unit: meter\n")
        for row in T:
            f.write(" ".join(f"{v:.6f}" for v in row) + "\n")


def update_log(timestamp, csv_path, name, me, mx, n_poses, T):
    """Append a row to res/calibration_log.md.  Creates the file if needed."""
    RES_DIR.mkdir(parents=True, exist_ok=True)

    if not LOG_FILE.exists():
        LOG_FILE.write_text(
            "# Calibration Log\n\n"
            "| timestamp | csv | method | poses | mean_mm | max_mm | memo |\n"
            "|-----------|-----|--------|-------|---------|--------|------|\n",
            encoding="utf-8",
        )

    t_str = f"`{_format_T(T)}`"  # inline code block for compact viewing
    row = (
        f"| {timestamp} "
        f"| `{csv_path.name}` "
        f"| {name} "
        f"| {n_poses} "
        f"| {me:.3f} "
        f"| {mx:.3f} "
        f"| _(memo)_ |\n"
    )

    with open(LOG_FILE, "a", encoding="utf-8") as f:
        f.write(row)

    # also write the T matrix as a collapsible block
    with open(LOG_FILE, "a", encoding="utf-8") as f:
        f.write(f"\n<details><summary>{timestamp} T_cam2gripper</summary>\n\n```\n")
        f.write(_format_T(T))
        f.write(f"\n```\n</details>\n\n")


# ─── main ────────────────────────────────────────────────────────────────────

def main():
    import argparse
    parser = argparse.ArgumentParser(description="Hand-eye extrinsic extraction")
    parser.add_argument("csv", nargs="?", default=None,
                        help="Path to sync_robot_aruco.csv (default: ./sync_robot_aruco.csv)")
    parser.add_argument("--marker-id", type=int, default=None,
                        help="Marker ID for multi-marker CSV (auto-detect if omitted)")
    args = parser.parse_args()

    if args.csv:
        csv_path = Path(args.csv)
    else:
        # Try to find the latest sync_robot_aruco.csv in res/ subdirectories
        csv_path = None
        if RES_DIR.exists():
            subdirs = sorted(
                [d for d in RES_DIR.iterdir() if d.is_dir()],
                key=lambda d: d.name,
                reverse=True,
            )
            for d in subdirs:
                candidate = d / "sync_robot_aruco.csv"
                if candidate.exists():
                    csv_path = candidate
                    break
        if csv_path is None:
            # Fallback to script directory
            csv_path = SCRIPT_DIR / "sync_robot_aruco.csv"
    if not csv_path.exists():
        sys.exit(f"Not found: {csv_path}")
    csv_path = csv_path.resolve()

    timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    # If CSV is inside a res/<ts>/ subfolder, output results there
    if csv_path.parent.parent == RES_DIR.resolve():
        out_dir = csv_path.parent
    else:
        out_dir = RES_DIR / timestamp

    print(f"{'='*50}")
    print(f"  Hand-Eye Extrinsic Extraction")
    print(f"  CSV   : {csv_path}")
    print(f"  Output: {out_dir}")
    print(f"{'='*50}\n")

    # load
    samples, fmt, mid = load(str(csv_path), marker_id=args.marker_id)
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
            if a > max_ang:
                max_ang = a
    print(f"  {n} poses, rotation spread = {max_ang:.1f} deg")
    if max_ang < 30:
        print(f"  WARNING: need > 30 deg spread for good calibration")

    # solve all methods
    Rg = [s[0] for s in samples]
    tg = [s[1].reshape(3, 1) for s in samples]
    Rt = [s[2] for s in samples]
    tt = [s[3].reshape(3, 1) for s in samples]

    print(f"\n{'Method':<12} {'Mean mm':>8} {'Max mm':>8}")
    print("-" * 30)

    best = None
    all_candidates = []
    for method_name, flag in METHODS.items():
        try:
            R, t = cv2.calibrateHandEye(Rg, tg, Rt, tt, method=flag)
            R = orthogonalize_rotation(R)
            T = hmat(R, t.ravel())
            me, mx, _, _ = back_err(samples, T)
            print(f"{method_name:<12} {me:8.1f} {mx:8.1f}")
            all_candidates.append((me, mx, method_name, T))
            if best is None or me < best[0]:
                best = (me, mx, method_name, T)
        except Exception:
            print(f"{method_name:<12}   FAILED")

    if best is None:
        sys.exit("All methods failed")

    me, mx, name, T = best
    all_candidates.sort(key=lambda x: x[0])

    # iterative outlier removal
    _, _, errs, _ = back_err(samples, T)
    best_me, best_mx, best_T, best_samples = me, mx, T.copy(), list(samples)
    for iteration in range(3):
        thr = np.mean(errs) + 1.5 * np.std(errs)
        keep = [i for i in range(len(samples)) if errs[i] < thr]
        if len(keep) < 5 or len(keep) == len(samples):
            break
        removed = len(samples) - len(keep)
        samples = [samples[i] for i in keep]
        Rg = [s[0] for s in samples]
        tg = [s[1].reshape(3, 1) for s in samples]
        Rt = [s[2] for s in samples]
        tt = [s[3].reshape(3, 1) for s in samples]
        R, t = cv2.calibrateHandEye(Rg, tg, Rt, tt, method=METHODS[name])
        R = orthogonalize_rotation(R)
        T = hmat(R, t.ravel())
        me, mx, errs, _ = back_err(samples, T)
        print(f"  iter{iteration+1}: removed {removed} outliers → {len(samples)} poses, err={me:.1f}mm")
        if me < best_me:
            best_me, best_mx, best_T, best_samples = me, mx, T.copy(), list(samples)
        else:
            print(f"  error increased, reverting to previous ({best_me:.1f}mm)")
            break

    me, mx, T, samples = best_me, best_mx, best_T, best_samples
    n_final = len(samples)

    # quality
    if me < 10:
        q = "GOOD"
    elif me < 30:
        q = "OK — consider adding more poses"
    else:
        q = "POOR — need more diverse poses"

    print(f"\n{'='*50}")
    print(f"  Best method : {name}  ({n_final} poses)")
    print(f"  Error       : {me:.3f} mm mean, {mx:.3f} mm max")
    print(f"  Quality     : {q}")
    print(f"\n  T_cam2gripper:")
    print(f"{_format_T(T)}")
    print(f"{'='*50}")

    # save
    save_results(out_dir, csv_path, name, me, mx, T, n_final, n_raw, fmt, mid,
                 all_candidates)
    update_log(timestamp, csv_path, name, me, mx, n_final, T)

    print(f"\n  Saved to : {out_dir}")
    print(f"  Log      : {LOG_FILE}")


if __name__ == "__main__":
    main()
