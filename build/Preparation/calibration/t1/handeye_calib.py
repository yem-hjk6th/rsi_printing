import argparse
import csv
import os
from typing import List, Tuple

import cv2
import numpy as np


def _euler_to_rot(a_deg: float, b_deg: float, c_deg: float, order: str = "ZYX") -> np.ndarray:
    a = np.deg2rad(a_deg)
    b = np.deg2rad(b_deg)
    c = np.deg2rad(c_deg)

    ca, sa = np.cos(a), np.sin(a)
    cb, sb = np.cos(b), np.sin(b)
    cc, sc = np.cos(c), np.sin(c)

    if order == "ZYX":
        r_z = np.array([[ca, -sa, 0.0], [sa, ca, 0.0], [0.0, 0.0, 1.0]])
        r_y = np.array([[cb, 0.0, sb], [0.0, 1.0, 0.0], [-sb, 0.0, cb]])
        r_x = np.array([[1.0, 0.0, 0.0], [0.0, cc, -sc], [0.0, sc, cc]])
        return r_z @ r_y @ r_x
    if order == "XYZ":
        r_x = np.array([[1.0, 0.0, 0.0], [0.0, ca, -sa], [0.0, sa, ca]])
        r_y = np.array([[cb, 0.0, sb], [0.0, 1.0, 0.0], [-sb, 0.0, cb]])
        r_z = np.array([[cc, -sc, 0.0], [sc, cc, 0.0], [0.0, 0.0, 1.0]])
        return r_x @ r_y @ r_z
    raise ValueError(f"Unsupported euler order: {order}")


def _load_robot_poses(csv_path: str) -> Tuple[List[np.ndarray], List[np.ndarray]]:
    r_gripper2base: List[np.ndarray] = []
    t_gripper2base: List[np.ndarray] = []

    with open(csv_path, "r", encoding="utf-8") as f:
        reader = csv.reader(f)
        header = next(reader)
        if header and header[0].startswith("start_time:"):
            header = next(reader)

        idx = {name: i for i, name in enumerate(header)}
        needed = ["x", "y", "z", "a", "b", "c"]
        if not all(k in idx for k in needed):
            raise RuntimeError(f"Robot CSV missing columns: {needed}")

        for row in reader:
            if not row:
                continue
            x = float(row[idx["x"]])
            y = float(row[idx["y"]])
            z = float(row[idx["z"]])
            a = float(row[idx["a"]])
            b = float(row[idx["b"]])
            c = float(row[idx["c"]])

            r_base_gripper = _euler_to_rot(a, b, c, order="ZYX")
            t_base_gripper = np.array([x, y, z], dtype=np.float64).reshape(3, 1)

            r_gripper = r_base_gripper.T
            t_gripper = -r_base_gripper.T @ t_base_gripper

            r_gripper2base.append(r_gripper)
            t_gripper2base.append(t_gripper)

    return r_gripper2base, t_gripper2base


def _load_aruco_poses(csv_path: str) -> Tuple[List[np.ndarray], List[np.ndarray]]:
    r_target2cam: List[np.ndarray] = []
    t_target2cam: List[np.ndarray] = []

    with open(csv_path, "r", encoding="utf-8") as f:
        reader = csv.reader(f)
        header = next(reader)
        idx = {name: i for i, name in enumerate(header)}
        needed = ["rvec_x", "rvec_y", "rvec_z", "tvec_x", "tvec_y", "tvec_z"]
        if not all(k in idx for k in needed):
            raise RuntimeError(f"ArUco CSV missing columns: {needed}")

        for row in reader:
            if not row:
                continue
            rvec = np.array(
                [float(row[idx["rvec_x"]]), float(row[idx["rvec_y"]]), float(row[idx["rvec_z"]])],
                dtype=np.float64,
            ).reshape(3, 1)
            tvec = np.array(
                [float(row[idx["tvec_x"]]), float(row[idx["tvec_y"]]), float(row[idx["tvec_z"]])],
                dtype=np.float64,
            ).reshape(3, 1)

            rmat, _ = cv2.Rodrigues(rvec)
            r_target2cam.append(rmat)
            t_target2cam.append(tvec)

    return r_target2cam, t_target2cam


def _apply_slice(data: List[np.ndarray], start: int, count: int) -> List[np.ndarray]:
    if count <= 0:
        return data[start:]
    return data[start : start + count]


def _to_homogeneous(r: np.ndarray, t: np.ndarray) -> np.ndarray:
    t = t.reshape(3, 1)
    mat = np.eye(4, dtype=np.float64)
    mat[:3, :3] = r
    mat[:3, 3:4] = t
    return mat


def main() -> None:
    parser = argparse.ArgumentParser(description="Hand-eye calibration (AX=XB)")
    parser.add_argument("--robot-csv", required=True, help="RSI robot pose CSV (x,y,z,a,b,c)")
    parser.add_argument("--aruco-csv", required=True, help="ArUco pose CSV (rvec/tvec)")
    parser.add_argument("--robot-start", type=int, default=0, help="start index for robot poses")
    parser.add_argument("--aruco-start", type=int, default=0, help="start index for aruco poses")
    parser.add_argument("--count", type=int, default=-1, help="number of paired poses to use")
    parser.add_argument(
        "--robot-units-mm",
        action="store_true",
        help="robot x/y/z are in mm (will be converted to meters)",
    )
    parser.add_argument(
        "--cam-units-mm",
        action="store_true",
        help="ArUco tvec are in mm (will be converted to meters)",
    )
    parser.add_argument(
        "--method",
        type=str,
        default="TSai",
        help="Tsai | Park | Horaud | Andreff | Daniilidis",
    )
    parser.add_argument("--out", type=str, default="handeye_result.txt", help="output file path")

    args = parser.parse_args()

    r_g2b, t_g2b = _load_robot_poses(args.robot_csv)
    r_t2c, t_t2c = _load_aruco_poses(args.aruco_csv)

    r_g2b = _apply_slice(r_g2b, args.robot_start, args.count)
    t_g2b = _apply_slice(t_g2b, args.robot_start, args.count)
    r_t2c = _apply_slice(r_t2c, args.aruco_start, args.count)
    t_t2c = _apply_slice(t_t2c, args.aruco_start, args.count)

    n = min(len(r_g2b), len(r_t2c))
    r_g2b = r_g2b[:n]
    t_g2b = t_g2b[:n]
    r_t2c = r_t2c[:n]
    t_t2c = t_t2c[:n]

    if n < 3:
        raise RuntimeError("Not enough pose pairs. Need at least 3.")

    if args.robot_units_mm:
        t_g2b = [t / 1000.0 for t in t_g2b]
    if args.cam_units_mm:
        t_t2c = [t / 1000.0 for t in t_t2c]

    method_map = {
        "Tsai": cv2.CALIB_HAND_EYE_TSAI,
        "Park": cv2.CALIB_HAND_EYE_PARK,
        "Horaud": cv2.CALIB_HAND_EYE_HORAUD,
        "Andreff": cv2.CALIB_HAND_EYE_ANDREFF,
        "Daniilidis": cv2.CALIB_HAND_EYE_DANIILIDIS,
    }
    if args.method not in method_map:
        raise RuntimeError(f"Unknown method: {args.method}")

    r_cam2gripper, t_cam2gripper = cv2.calibrateHandEye(
        r_g2b,
        t_g2b,
        r_t2c,
        t_t2c,
        method=method_map[args.method],
    )

    t_cam2gripper = t_cam2gripper.reshape(3, 1)
    t_gripper2cam = -r_cam2gripper.T @ t_cam2gripper
    r_gripper2cam = r_cam2gripper.T

    t_base_gripper = -np.zeros((3, 1))
    t_base_gripper = t_base_gripper  # placeholder for clarity

    cam_in_gripper = _to_homogeneous(r_cam2gripper, t_cam2gripper)
    gripper_in_cam = _to_homogeneous(r_gripper2cam, t_gripper2cam)

    result = {
        "R_cam2gripper": r_cam2gripper,
        "t_cam2gripper": t_cam2gripper,
        "T_cam2gripper": cam_in_gripper,
        "T_gripper2cam": gripper_in_cam,
    }

    out_path = args.out
    out_dir = os.path.dirname(out_path)
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)

    with open(out_path, "w", encoding="utf-8") as f:
        f.write("R_cam2gripper:\n")
        f.write(np.array2string(r_cam2gripper, precision=8, suppress_small=True))
        f.write("\n\nt_cam2gripper:\n")
        f.write(np.array2string(t_cam2gripper.reshape(-1), precision=8, suppress_small=True))
        f.write("\n\nT_cam2gripper:\n")
        f.write(np.array2string(cam_in_gripper, precision=8, suppress_small=True))
        f.write("\n\nT_gripper2cam:\n")
        f.write(np.array2string(gripper_in_cam, precision=8, suppress_small=True))
        f.write("\n")

    np.savez(
        os.path.splitext(out_path)[0] + ".npz",
        R_cam2gripper=r_cam2gripper,
        t_cam2gripper=t_cam2gripper,
        T_cam2gripper=cam_in_gripper,
        T_gripper2cam=gripper_in_cam,
    )

    print("Saved:", out_path)
    print("Saved:", os.path.splitext(out_path)[0] + ".npz")
    print("T_cam2gripper:\n", cam_in_gripper)


if __name__ == "__main__":
    main()
