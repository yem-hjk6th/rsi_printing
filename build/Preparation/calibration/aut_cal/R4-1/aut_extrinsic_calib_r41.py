#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import csv
from pathlib import Path
from typing import Dict, List, Tuple

import cv2
import numpy as np

SCRIPT_DIR = Path(__file__).resolve().parent
R4_DIR = SCRIPT_DIR.parent / "R4"


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


def inv_h(T: np.ndarray) -> np.ndarray:
    R = T[:3, :3]
    t = T[:3, 3]
    T_inv = np.eye(4, dtype=np.float64)
    T_inv[:3, :3] = R.T
    T_inv[:3, 3] = -R.T @ t
    return T_inv


def rot_angle_deg(R: np.ndarray) -> float:
    v = float(np.clip((np.trace(R) - 1.0) * 0.5, -1.0, 1.0))
    return float(np.rad2deg(np.arccos(v)))


def parse_sync_rows(csv_path: Path) -> List[Dict[str, str]]:
    with csv_path.open("r", encoding="utf-8", newline="") as f:
        return list(csv.DictReader(f))


def available_marker_ids(header: List[str]) -> List[int]:
    mids: List[int] = []
    for col in header:
        if col.startswith("m") and col.endswith("_rvec_x"):
            mids.append(int(col[1:].split("_", 1)[0]))
    return sorted(set(mids))


def count_valid_marker_rows(rows: List[Dict[str, str]], marker_id: int) -> int:
    key = f"m{marker_id}_tvec_z_m"
    return sum(1 for r in rows if r.get(key, "").strip() != "")


def tool_transform(tool_xyz_mm: Tuple[float, float, float], tool_abc_deg: Tuple[float, float, float]) -> np.ndarray:
    tx, ty, tz = tool_xyz_mm
    a, b, c = tool_abc_deg
    R_ft = euler_zyx_deg_to_rot(a, b, c)
    t_ft = np.array([tx, ty, tz], dtype=np.float64) / 1000.0
    return to_h(R_ft, t_ft)


def build_inputs(
    rows: List[Dict[str, str]],
    marker_id: int,
    assume_robot_pose_is_tcp: bool,
    T_flange_tcp: np.ndarray,
) -> Tuple[List[np.ndarray], List[np.ndarray], List[np.ndarray], List[np.ndarray], List[int], List[np.ndarray], List[np.ndarray]]:
    R_g2b: List[np.ndarray] = []
    t_g2b: List[np.ndarray] = []
    R_t2c: List[np.ndarray] = []
    t_t2c: List[np.ndarray] = []
    row_indices: List[int] = []

    T_b_g_list: List[np.ndarray] = []
    T_t_c_list: List[np.ndarray] = []

    for i, r in enumerate(rows):
        if r.get(f"m{marker_id}_tvec_z_m", "").strip() == "":
            continue

        x_mm = float(r["robot_x_mm"])
        y_mm = float(r["robot_y_mm"])
        z_mm = float(r["robot_z_mm"])
        a_deg = float(r["robot_a_deg"])
        b_deg = float(r["robot_b_deg"])
        c_deg = float(r["robot_c_deg"])

        R_b_pose = euler_zyx_deg_to_rot(a_deg, b_deg, c_deg)
        t_b_pose = np.array([x_mm, y_mm, z_mm], dtype=np.float64) / 1000.0
        T_b_pose = to_h(R_b_pose, t_b_pose)

        if assume_robot_pose_is_tcp:
            T_b_g = T_b_pose @ inv_h(T_flange_tcp)
        else:
            T_b_g = T_b_pose

        T_g_b = inv_h(T_b_g)

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

        R_tc, _ = cv2.Rodrigues(rvec)

        R_g2b.append(T_g_b[:3, :3])
        t_g2b.append(T_g_b[:3, 3].reshape(3, 1))
        R_t2c.append(R_tc)
        t_t2c.append(tvec)
        row_indices.append(i)

        T_b_g_list.append(T_b_g)
        T_t_c_list.append(to_h(R_tc, tvec))

    return R_g2b, t_g2b, R_t2c, t_t2c, row_indices, T_b_g_list, T_t_c_list


def back_substitution_check(
    T_b_g_list: List[np.ndarray],
    T_t_c_list: List[np.ndarray],
    T_c_g: np.ndarray,
) -> Dict[str, float]:
    T_g_c = inv_h(T_c_g)

    T_b_t_list: List[np.ndarray] = []
    for T_b_g, T_t_c in zip(T_b_g_list, T_t_c_list):
        T_c_t = inv_h(T_t_c)
        T_b_t = T_b_g @ T_g_c @ T_c_t
        T_b_t_list.append(T_b_t)

    t_all = np.array([T[:3, 3] for T in T_b_t_list], dtype=np.float64)
    t_mean = np.mean(t_all, axis=0)
    t_err_mm = np.linalg.norm(t_all - t_mean, axis=1) * 1000.0

    R_ref = T_b_t_list[0][:3, :3]
    ang_err_deg = np.array([rot_angle_deg(R_ref.T @ T[:3, :3]) for T in T_b_t_list], dtype=np.float64)

    return {
        "t_err_mm_mean": float(np.mean(t_err_mm)),
        "t_err_mm_max": float(np.max(t_err_mm)),
        "ang_err_deg_mean": float(np.mean(ang_err_deg)),
        "ang_err_deg_max": float(np.max(ang_err_deg)),
    }


def evaluate_existing_result(npz_path: Path, T_b_g_list: List[np.ndarray], T_t_c_list: List[np.ndarray]) -> Dict[str, float]:
    data = np.load(npz_path)
    R_c_g = np.array(data["R_cam2gripper"], dtype=np.float64)
    t_c_g = np.array(data["t_cam2gripper"], dtype=np.float64).reshape(3, 1)
    T_c_g = to_h(R_c_g, t_c_g)
    return back_substitution_check(T_b_g_list, T_t_c_list, T_c_g)


def solve_one_assumption(
    rows: List[Dict[str, str]],
    marker_id: int,
    methods: List[str],
    method_map: Dict[str, int],
    assume_robot_pose_is_tcp: bool,
    T_flange_tcp: np.ndarray,
) -> Dict[str, object]:
    R_g2b, t_g2b, R_t2c, t_t2c, used_idx, T_b_g_list, T_t_c_list = build_inputs(
        rows,
        marker_id,
        assume_robot_pose_is_tcp=assume_robot_pose_is_tcp,
        T_flange_tcp=T_flange_tcp,
    )

    n = len(R_g2b)
    if n < 3:
        raise RuntimeError(f"Not enough valid pairs under assumption tcp={assume_robot_pose_is_tcp}: {n}")

    candidates = []
    for method_name in methods:
        R_c_g, t_c_g = cv2.calibrateHandEye(
            R_g2b,
            t_g2b,
            R_t2c,
            t_t2c,
            method=method_map[method_name],
        )
        t_c_g = t_c_g.reshape(3, 1)
        T_c_g = to_h(R_c_g, t_c_g)
        metrics = back_substitution_check(T_b_g_list, T_t_c_list, T_c_g)
        score = metrics["t_err_mm_mean"] + 10.0 * metrics["ang_err_deg_mean"]
        candidates.append((score, method_name, T_c_g, metrics))

    candidates.sort(key=lambda x: x[0])
    score, chosen_method, T_c_g_best, m_best = candidates[0]

    return {
        "assumption": "tcp" if assume_robot_pose_is_tcp else "flange",
        "score": score,
        "method": chosen_method,
        "T_cam2gripper": T_c_g_best,
        "metrics": m_best,
        "candidates": candidates,
        "used_idx": used_idx,
        "pairs_used": n,
        "T_b_g_list": T_b_g_list,
        "T_t_c_list": T_t_c_list,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description="R4-1 corrected extrinsic solve + compare with R4 result")
    parser.add_argument("--sync-csv", type=str, default=str(R4_DIR / "sync_robot_aruco.csv"))
    parser.add_argument("--r4-result", type=str, default=str(R4_DIR / "extrinsic_result_r4.npz"))
    parser.add_argument("--marker-id", type=str, default="auto")
    parser.add_argument("--method", type=str, default="auto", help="auto|Tsai|Park|Horaud|Andreff|Daniilidis")

    parser.add_argument(
        "--pose-assumption",
        type=str,
        default="auto",
        choices=["auto", "tcp", "flange"],
        help="Interpret robot RIst pose as tcp, flange, or try both (auto)",
    )
    parser.add_argument("--tool-x-mm", type=float, default=225.550)
    parser.add_argument("--tool-y-mm", type=float, default=-0.600)
    parser.add_argument("--tool-z-mm", type=float, default=72.830)
    parser.add_argument("--tool-a-deg", type=float, default=0.0)
    parser.add_argument("--tool-b-deg", type=float, default=0.0)
    parser.add_argument("--tool-c-deg", type=float, default=0.0)

    parser.add_argument("--out", type=str, default=str(SCRIPT_DIR / "extrinsic_result_r41"))
    args = parser.parse_args()

    csv_path = Path(args.sync_csv)
    if not csv_path.exists():
        raise FileNotFoundError(f"CSV not found: {csv_path}")

    rows = parse_sync_rows(csv_path)
    if len(rows) < 3:
        raise RuntimeError("Not enough rows in CSV")

    mids = available_marker_ids(list(rows[0].keys()))
    if not mids:
        raise RuntimeError("No marker columns found")

    if args.marker_id.lower() == "auto":
        marker_id = max(mids, key=lambda m: count_valid_marker_rows(rows, m))
    else:
        marker_id = int(args.marker_id)
        if marker_id not in mids:
            raise RuntimeError(f"Marker id {marker_id} not in {mids}")

    T_flange_tcp = tool_transform(
        (args.tool_x_mm, args.tool_y_mm, args.tool_z_mm),
        (args.tool_a_deg, args.tool_b_deg, args.tool_c_deg),
    )

    method_map = {
        "Tsai": cv2.CALIB_HAND_EYE_TSAI,
        "Park": cv2.CALIB_HAND_EYE_PARK,
        "Horaud": cv2.CALIB_HAND_EYE_HORAUD,
        "Andreff": cv2.CALIB_HAND_EYE_ANDREFF,
        "Daniilidis": cv2.CALIB_HAND_EYE_DANIILIDIS,
    }

    if args.method == "auto":
        methods = list(method_map.keys())
    else:
        if args.method not in method_map:
            raise RuntimeError(f"Unknown method: {args.method}")
        methods = [args.method]

    assumption_flags = []
    if args.pose_assumption == "auto":
        assumption_flags = [True, False]
    elif args.pose_assumption == "tcp":
        assumption_flags = [True]
    else:
        assumption_flags = [False]

    assumption_results: List[Dict[str, object]] = []
    for assume_tcp in assumption_flags:
        result = solve_one_assumption(
            rows=rows,
            marker_id=marker_id,
            methods=methods,
            method_map=method_map,
            assume_robot_pose_is_tcp=assume_tcp,
            T_flange_tcp=T_flange_tcp,
        )
        assumption_results.append(result)

    assumption_results.sort(key=lambda r: float(r["score"]))
    best = assumption_results[0]
    chosen_assumption = str(best["assumption"])
    chosen_method = str(best["method"])
    T_c_g_best = np.array(best["T_cam2gripper"], dtype=np.float64)
    m_best = dict(best["metrics"])
    candidates = list(best["candidates"])
    used_idx = list(best["used_idx"])
    n = int(best["pairs_used"])
    T_b_g_list = best["T_b_g_list"]
    T_t_c_list = best["T_t_c_list"]

    T_g_c_best = inv_h(T_c_g_best)

    r4_compare = None
    r4_result_path = Path(args.r4_result)
    if r4_result_path.exists():
        r4_compare = evaluate_existing_result(r4_result_path, T_b_g_list, T_t_c_list)

    out_prefix = Path(args.out)
    out_prefix.parent.mkdir(parents=True, exist_ok=True)
    txt_path = out_prefix.with_suffix(".txt")
    npz_path = out_prefix.with_suffix(".npz")

    with txt_path.open("w", encoding="utf-8") as f:
        f.write(f"csv={csv_path}\n")
        f.write(f"marker_id={marker_id}\n")
        f.write(f"pose_assumption={chosen_assumption}\n")
        f.write(f"pairs_used={n}\n")
        f.write(f"rows_used(0-based)={used_idx}\n")
        f.write(
            "tool_flange_to_tcp(mm,deg)="
            f"({args.tool_x_mm},{args.tool_y_mm},{args.tool_z_mm},{args.tool_a_deg},{args.tool_b_deg},{args.tool_c_deg})\n\n"
        )

        f.write(f"method={chosen_method}\n")
        f.write("T_cam2gripper:\n")
        f.write(np.array2string(T_c_g_best, precision=8, suppress_small=True))
        f.write("\n\nT_gripper2cam:\n")
        f.write(np.array2string(T_g_c_best, precision=8, suppress_small=True))

        f.write("\n\nBackSubstitution(new):\n")
        for k, v in m_best.items():
            f.write(f"{k}={v}\n")

        if r4_compare is not None:
            f.write("\nBackSubstitution(existing_R4_npz):\n")
            for k, v in r4_compare.items():
                f.write(f"{k}={v}\n")

        if len(candidates) > 1:
            f.write("\nMethodCandidates(sorted):\n")
            for s, method_name, _, mm in candidates:
                f.write(
                    f"{method_name}: score={s:.3f}, "
                    f"t_mean={mm['t_err_mm_mean']:.3f}mm, ang_mean={mm['ang_err_deg_mean']:.3f}deg\n"
                )

        f.write("\nAssumptionCandidates(sorted):\n")
        for rr in assumption_results:
            mm = rr["metrics"]
            f.write(
                f"{rr['assumption']}: score={float(rr['score']):.3f}, method={rr['method']}, "
                f"pairs={rr['pairs_used']}, t_mean={mm['t_err_mm_mean']:.3f}mm, ang_mean={mm['ang_err_deg_mean']:.3f}deg\n"
            )

    np.savez(
        npz_path,
        T_cam2gripper=T_c_g_best,
        T_gripper2cam=T_g_c_best,
        R_cam2gripper=T_c_g_best[:3, :3],
        t_cam2gripper=T_c_g_best[:3, 3],
        marker_id=np.array([marker_id], dtype=np.int32),
        pairs_used=np.array([n], dtype=np.int32),
        pose_assumption=np.array([chosen_assumption]),
        method=np.array([chosen_method]),
        t_err_mm_mean=np.array([m_best["t_err_mm_mean"]], dtype=np.float64),
        t_err_mm_max=np.array([m_best["t_err_mm_max"]], dtype=np.float64),
        ang_err_deg_mean=np.array([m_best["ang_err_deg_mean"]], dtype=np.float64),
        ang_err_deg_max=np.array([m_best["ang_err_deg_max"]], dtype=np.float64),
    )

    print(f"[OK] csv={csv_path}")
    print(f"[OK] marker={marker_id} pairs={n} assumption={chosen_assumption} method={chosen_method}")
    print("[NEW] BackSubstitution:")
    print(
        f"  t_mean={m_best['t_err_mm_mean']:.3f}mm "
        f"t_max={m_best['t_err_mm_max']:.3f}mm "
        f"ang_mean={m_best['ang_err_deg_mean']:.3f}deg "
        f"ang_max={m_best['ang_err_deg_max']:.3f}deg"
    )

    if r4_compare is not None:
        print("[R4] Existing npz BackSubstitution:")
        print(
            f"  t_mean={r4_compare['t_err_mm_mean']:.3f}mm "
            f"t_max={r4_compare['t_err_mm_max']:.3f}mm "
            f"ang_mean={r4_compare['ang_err_deg_mean']:.3f}deg "
            f"ang_max={r4_compare['ang_err_deg_max']:.3f}deg"
        )

    print("[ASSUMPTIONS]")
    for rr in assumption_results:
        mm = rr["metrics"]
        print(
            f"  - {rr['assumption']}: score={float(rr['score']):.3f}, method={rr['method']}, "
            f"pairs={rr['pairs_used']}, t_mean={mm['t_err_mm_mean']:.3f}mm, ang_mean={mm['ang_err_deg_mean']:.3f}deg"
        )

    print(f"[SAVE] {txt_path}")
    print(f"[SAVE] {npz_path}")


if __name__ == "__main__":
    main()
