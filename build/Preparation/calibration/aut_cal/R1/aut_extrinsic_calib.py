#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import csv
import os
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np


def euler_zyx_deg_to_rot(a_deg: float, b_deg: float, c_deg: float) -> np.ndarray:
    a = np.deg2rad(a_deg)
    b = np.deg2rad(b_deg)
    c = np.deg2rad(c_deg)

    ca, sa = np.cos(a), np.sin(a)
    cb, sb = np.cos(b), np.sin(b)
    cc, sc = np.cos(c), np.sin(c)

    r_z = np.array([[ca, -sa, 0.0], [sa, ca, 0.0], [0.0, 0.0, 1.0]], dtype=np.float64)
    r_y = np.array([[cb, 0.0, sb], [0.0, 1.0, 0.0], [-sb, 0.0, cb]], dtype=np.float64)
    r_x = np.array([[1.0, 0.0, 0.0], [0.0, cc, -sc], [0.0, sc, cc]], dtype=np.float64)
    return r_z @ r_y @ r_x


def to_h(R: np.ndarray, t: np.ndarray) -> np.ndarray:
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = R
    T[:3, 3] = t.reshape(3)
    return T


def rot_angle_deg(R: np.ndarray) -> float:
    v = (np.trace(R) - 1.0) * 0.5
    v = float(np.clip(v, -1.0, 1.0))
    return float(np.rad2deg(np.arccos(v)))


def parse_sync_rows(csv_path: Path) -> List[Dict[str, str]]:
    with csv_path.open("r", encoding="utf-8", newline="") as f:
        reader = csv.DictReader(f)
        return list(reader)


def available_marker_ids(header: List[str]) -> List[int]:
    marker_ids: List[int] = []
    for col in header:
        if col.startswith("m") and col.endswith("_rvec_x"):
            mid = int(col[1:].split("_", 1)[0])
            marker_ids.append(mid)
    return sorted(set(marker_ids))


def count_valid_marker_rows(rows: List[Dict[str, str]], marker_id: int) -> int:
    k = f"m{marker_id}_tvec_z_m"
    return sum(1 for r in rows if r.get(k, "").strip() != "")


def build_handeye_inputs(
    rows: List[Dict[str, str]], marker_id: int
) -> Tuple[List[np.ndarray], List[np.ndarray], List[np.ndarray], List[np.ndarray], List[int]]:
    R_g2b: List[np.ndarray] = []
    t_g2b: List[np.ndarray] = []
    R_t2c: List[np.ndarray] = []
    t_t2c: List[np.ndarray] = []
    row_indices: List[int] = []

    for i, r in enumerate(rows):
        t_key = f"m{marker_id}_tvec_z_m"
        if r.get(t_key, "").strip() == "":
            continue

        x_mm = float(r["robot_x_mm"])
        y_mm = float(r["robot_y_mm"])
        z_mm = float(r["robot_z_mm"])
        a_deg = float(r["robot_a_deg"])
        b_deg = float(r["robot_b_deg"])
        c_deg = float(r["robot_c_deg"])

        R_bg = euler_zyx_deg_to_rot(a_deg, b_deg, c_deg)
        t_bg = np.array([x_mm, y_mm, z_mm], dtype=np.float64).reshape(3, 1) / 1000.0

        rvec = np.array(
            [
                float(r[f"m{marker_id}_rvec_x"]),
                float(r[f"m{marker_id}_rvec_y"]),
                float(r[f"m{marker_id}_rvec_z"]),
            ],
            dtype=np.float64,
        ).reshape(3, 1)
        tvec = np.array(
            [
                float(r[f"m{marker_id}_tvec_x_m"]),
                float(r[f"m{marker_id}_tvec_y_m"]),
                float(r[f"m{marker_id}_tvec_z_m"]),
            ],
            dtype=np.float64,
        ).reshape(3, 1)
        R_ct, _ = cv2.Rodrigues(rvec)

        R_g2b.append(R_bg)
        t_g2b.append(t_bg)
        R_t2c.append(R_ct)
        t_t2c.append(tvec)
        row_indices.append(i)

    return R_g2b, t_g2b, R_t2c, t_t2c, row_indices


def back_substitution_check(
    R_g2b: List[np.ndarray],
    t_g2b: List[np.ndarray],
    R_t2c: List[np.ndarray],
    t_t2c: List[np.ndarray],
    R_c2g: np.ndarray,
    t_c2g: np.ndarray,
) -> Dict[str, float]:
    T_g_c = to_h(R_c2g, t_c2g)

    T_b_t_list: List[np.ndarray] = []
    for R_bg, t_bg, R_ct, t_ct in zip(R_g2b, t_g2b, R_t2c, t_t2c):
        T_b_g = to_h(R_bg, t_bg)
        T_c_t = to_h(R_ct, t_ct)
        T_b_t = T_b_g @ T_g_c @ T_c_t
        T_b_t_list.append(T_b_t)

    t_all = np.array([T[:3, 3] for T in T_b_t_list], dtype=np.float64)
    t_mean = np.mean(t_all, axis=0)
    t_err_mm = np.linalg.norm((t_all - t_mean), axis=1) * 1000.0

    R_ref = T_b_t_list[0][:3, :3]
    ang_err_deg = np.array([rot_angle_deg(R_ref.T @ T[:3, :3]) for T in T_b_t_list], dtype=np.float64)

    return {
        "t_err_mm_mean": float(np.mean(t_err_mm)),
        "t_err_mm_max": float(np.max(t_err_mm)),
        "ang_err_deg_mean": float(np.mean(ang_err_deg)),
        "ang_err_deg_max": float(np.max(ang_err_deg)),
    }


def main() -> None:
    script_dir = Path(__file__).resolve().parent

    parser = argparse.ArgumentParser(description="Extrinsic calibration from sync_robot_aruco.csv with auto back-check")
    parser.add_argument(
        "--sync-csv",
        type=str,
        default=str(script_dir / "sync_robot_aruco.csv"),
        help="Path to synchronized robot+aruco CSV",
    )
    parser.add_argument(
        "--marker-id",
        type=str,
        default="auto",
        help="Marker id to use (e.g. 4), or 'auto' for best coverage",
    )
    parser.add_argument(
        "--method",
        type=str,
        default="auto",
        help="auto | Tsai | Park | Horaud | Andreff | Daniilidis",
    )
    parser.add_argument("--accept-trans-mm", type=float, default=20.0, help="pass threshold for mean translation back-check")
    parser.add_argument("--accept-ang-deg", type=float, default=3.0, help="pass threshold for mean angle back-check")
    parser.add_argument(
        "--out",
        type=str,
        default=str(script_dir / "extrinsic_result"),
        help="Output prefix without extension",
    )
    args = parser.parse_args()

    csv_path = Path(args.sync_csv)
    if not csv_path.exists():
        raise FileNotFoundError(f"CSV not found: {csv_path}")

    rows = parse_sync_rows(csv_path)
    if len(rows) < 3:
        raise RuntimeError("Not enough rows in CSV.")

    header = list(rows[0].keys())
    mids = available_marker_ids(header)
    if not mids:
        raise RuntimeError("No marker columns found in CSV.")

    if args.marker_id.lower() == "auto":
        best_mid = max(mids, key=lambda mid: count_valid_marker_rows(rows, mid))
        marker_id = int(best_mid)
    else:
        marker_id = int(args.marker_id)
        if marker_id not in mids:
            raise RuntimeError(f"Marker id {marker_id} not found in CSV columns: {mids}")

    R_g2b, t_g2b, R_t2c, t_t2c, used_idx = build_handeye_inputs(rows, marker_id)
    n = len(R_g2b)
    if n < 3:
        raise RuntimeError(f"Not enough valid pairs for marker {marker_id}. Need >=3, got {n}")

    method_map = {
        "Tsai": cv2.CALIB_HAND_EYE_TSAI,
        "Park": cv2.CALIB_HAND_EYE_PARK,
        "Horaud": cv2.CALIB_HAND_EYE_HORAUD,
        "Andreff": cv2.CALIB_HAND_EYE_ANDREFF,
        "Daniilidis": cv2.CALIB_HAND_EYE_DANIILIDIS,
    }
    if args.method == "auto":
        selected_methods = list(method_map.keys())
    else:
        if args.method not in method_map:
            raise RuntimeError(f"Unknown method: {args.method}")
        selected_methods = [args.method]

    candidates = []
    for method_name in selected_methods:
        R_c2g, t_c2g = cv2.calibrateHandEye(
            R_g2b,
            t_g2b,
            R_t2c,
            t_t2c,
            method=method_map[method_name],
        )
        t_c2g = t_c2g.reshape(3, 1)
        metrics = back_substitution_check(R_g2b, t_g2b, R_t2c, t_t2c, R_c2g, t_c2g)
        score = metrics["t_err_mm_mean"] + 10.0 * metrics["ang_err_deg_mean"]
        candidates.append((score, method_name, R_c2g, t_c2g, metrics))

    candidates.sort(key=lambda x: x[0])
    _, chosen_method, R_c2g, t_c2g, metrics = candidates[0]
    T_c2g = to_h(R_c2g, t_c2g)

    pass_check = (
        metrics["t_err_mm_mean"] <= args.accept_trans_mm
        and metrics["ang_err_deg_mean"] <= args.accept_ang_deg
    )

    out_prefix = Path(args.out)
    out_prefix.parent.mkdir(parents=True, exist_ok=True)
    txt_path = out_prefix.with_suffix(".txt")
    npz_path = out_prefix.with_suffix(".npz")

    with txt_path.open("w", encoding="utf-8") as f:
        f.write(f"csv={csv_path}\n")
        f.write(f"marker_id={marker_id}\n")
        f.write(f"method={chosen_method}\n")
        f.write(f"pairs_used={n}\n")
        f.write(f"rows_used(0-based)={used_idx}\n\n")

        f.write("R_cam2gripper:\n")
        f.write(np.array2string(R_c2g, precision=8, suppress_small=True))
        f.write("\n\nt_cam2gripper_m:\n")
        f.write(np.array2string(t_c2g.reshape(-1), precision=8, suppress_small=True))
        f.write("\n\nT_cam2gripper:\n")
        f.write(np.array2string(T_c2g, precision=8, suppress_small=True))
        f.write("\n\nBackSubstitution:\n")
        for k, v in metrics.items():
            f.write(f"{k}={v}\n")
        f.write(f"pass_check={pass_check}\n")
        f.write(f"threshold_trans_mm={args.accept_trans_mm}\n")
        f.write(f"threshold_ang_deg={args.accept_ang_deg}\n")
        if len(candidates) > 1:
            f.write("\nMethodCandidates(sorted):\n")
            for score, method_name, _, _, m in candidates:
                f.write(
                    f"{method_name}: score={score:.3f}, t_mean={m['t_err_mm_mean']:.3f}mm, "
                    f"ang_mean={m['ang_err_deg_mean']:.3f}deg\n"
                )

    np.savez(
        npz_path,
        R_cam2gripper=R_c2g,
        t_cam2gripper=t_c2g,
        T_cam2gripper=T_c2g,
        method=np.array([chosen_method]),
        marker_id=np.array([marker_id], dtype=np.int32),
        pairs_used=np.array([n], dtype=np.int32),
        pass_check=np.array([int(pass_check)], dtype=np.int32),
        t_err_mm_mean=np.array([metrics["t_err_mm_mean"]], dtype=np.float64),
        t_err_mm_max=np.array([metrics["t_err_mm_max"]], dtype=np.float64),
        ang_err_deg_mean=np.array([metrics["ang_err_deg_mean"]], dtype=np.float64),
        ang_err_deg_max=np.array([metrics["ang_err_deg_max"]], dtype=np.float64),
    )

    print(f"[OK] csv={csv_path}")
    print(f"[OK] marker_id={marker_id} pairs={n} method={chosen_method}")
    print("[RESULT] T_cam2gripper:")
    print(np.array2string(T_c2g, precision=6, suppress_small=True))
    print(
        "[CHECK] "
        f"t_err_mean={metrics['t_err_mm_mean']:.3f}mm "
        f"t_err_max={metrics['t_err_mm_max']:.3f}mm "
        f"ang_err_mean={metrics['ang_err_deg_mean']:.3f}deg "
        f"ang_err_max={metrics['ang_err_deg_max']:.3f}deg"
    )
    print(
        f"[QUALITY] pass={pass_check} "
        f"(thresholds: trans<={args.accept_trans_mm:.1f}mm, ang<={args.accept_ang_deg:.1f}deg)"
    )
    if len(candidates) > 1:
        print("[CANDIDATES]")
        for score, method_name, _, _, m in candidates:
            print(
                f"- {method_name}: score={score:.3f}, "
                f"t_mean={m['t_err_mm_mean']:.3f}mm, ang_mean={m['ang_err_deg_mean']:.3f}deg"
            )
    print(f"[SAVE] {txt_path}")
    print(f"[SAVE] {npz_path}")


if __name__ == "__main__":
    main()
