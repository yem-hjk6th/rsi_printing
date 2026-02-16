import csv
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Tuple

import cv2
import numpy as np

# ArUco rvec unit: set to "RAD" if already in radians, "DEG" if in degrees
ARUCO_RVEC_UNIT = "RAD"

# If these files exist, use them for validation instead of recomputing
R_CAM2GRIPPER_NPY = "R_cam2gripper.npy"
T_CAM2GRIPPER_NPY = "t_cam2gripper.npy"

# If set, only compute for these marker IDs (e.g., [0]); None means all
TARGET_MARKER_IDS: List[int] | None = None

# Robot pose interpretation
# If the recorded robot pose is TCP (tool) coordinates, set ROBOT_POSE_IS_TCP = True
# Tool offset (flange->tool) for TOOL3: units mm / deg
ROBOT_POSE_IS_TCP = True
TOOL_OFFSET_MM = np.array([225.550, -0.600, 72.830], dtype=np.float64)
TOOL_OFFSET_DEG = np.array([0.0, 0.0, 0.0], dtype=np.float64)

# If True, run both TCP and flange interpretations for comparison
TEST_BOTH_ROBOT_POSE_MODES = True


@dataclass
class Pose:
    rvec: np.ndarray  # (3,)
    tvec: np.ndarray  # (3,)


def read_robot_poses(robot_path: Path) -> List[Pose]:
    poses: List[Pose] = []
    with robot_path.open("r", encoding="utf-8") as f:
        reader = csv.reader(f)
        header = next(reader, None)
        if header is None:
            raise ValueError("robot_pos.txt is empty")
        for row in reader:
            if not row or len(row) < 6:
                continue
            x_mm, y_mm, z_mm, a_deg, b_deg, c_deg = [float(v) for v in row[:6]]
            t = np.array([x_mm, y_mm, z_mm], dtype=np.float64) / 1000.0
            a, b, c = np.deg2rad([a_deg, b_deg, c_deg])
            # KUKA ABC intrinsic Z-Y-X: A around Z, B around Y, C around X
            r = euler_zyx_to_rmat(a, b, c)
            rvec, _ = cv2.Rodrigues(r)
            poses.append(Pose(rvec.reshape(3), t.reshape(3)))
    return poses


def euler_zyx_to_rmat(a: float, b: float, c: float) -> np.ndarray:
    cz, sz = np.cos(a), np.sin(a)
    cy, sy = np.cos(b), np.sin(b)
    cx, sx = np.cos(c), np.sin(c)

    rz = np.array([[cz, -sz, 0.0], [sz, cz, 0.0], [0.0, 0.0, 1.0]], dtype=np.float64)
    ry = np.array([[cy, 0.0, sy], [0.0, 1.0, 0.0], [-sy, 0.0, cy]], dtype=np.float64)
    rx = np.array([[1.0, 0.0, 0.0], [0.0, cx, -sx], [0.0, sx, cx]], dtype=np.float64)
    return rz @ ry @ rx


def invert_pose(rvec: np.ndarray, tvec: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    rmat, _ = cv2.Rodrigues(rvec.reshape(3))
    r_inv = rmat.T
    t_inv = -r_inv @ tvec.reshape(3)
    rvec_inv, _ = cv2.Rodrigues(r_inv)
    return rvec_inv.reshape(3), t_inv.reshape(3)


def rvec_tvec_to_T(rvec: np.ndarray, tvec: np.ndarray) -> np.ndarray:
    rmat, _ = cv2.Rodrigues(rvec.reshape(3))
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = rmat
    T[:3, 3] = tvec.reshape(3)
    return T


def invert_T(T: np.ndarray) -> np.ndarray:
    r = T[:3, :3]
    t = T[:3, 3]
    T_inv = np.eye(4, dtype=np.float64)
    T_inv[:3, :3] = r.T
    T_inv[:3, 3] = -r.T @ t
    return T_inv


def make_tool_T() -> np.ndarray:
    t = TOOL_OFFSET_MM / 1000.0
    a, b, c = np.deg2rad(TOOL_OFFSET_DEG)
    r = euler_zyx_to_rmat(a, b, c)
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = r
    T[:3, 3] = t
    return T


def average_rotation(rotations: List[np.ndarray]) -> np.ndarray:
    r_sum = np.zeros((3, 3), dtype=np.float64)
    for r in rotations:
        r_sum += r
    u, _, vt = np.linalg.svd(r_sum)
    r_avg = u @ vt
    if np.linalg.det(r_avg) < 0:
        r_avg[:, -1] *= -1
    return r_avg


def rotation_error_deg(r_ref: np.ndarray, r_est: np.ndarray) -> float:
    r_err = r_ref.T @ r_est
    trace = np.clip((np.trace(r_err) - 1.0) / 2.0, -1.0, 1.0)
    angle = np.arccos(trace)
    return float(np.rad2deg(angle))


def read_aruco_poses(aruco_path: Path) -> Dict[float, Dict[int, Pose]]:
    poses: Dict[float, Dict[int, Pose]] = {}
    with aruco_path.open("r", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            ts = float(row["timestamp_s"])
            marker_id = int(row["marker_id"])
            rvec = np.array(
                [float(row["rvec_x"]), float(row["rvec_y"]), float(row["rvec_z"])],
                dtype=np.float64,
            )
            if ARUCO_RVEC_UNIT.upper() == "DEG":
                rvec = np.deg2rad(rvec)
            tvec = np.array([float(row["tvec_x"]), float(row["tvec_y"]), float(row["tvec_z"])], dtype=np.float64)
            poses.setdefault(ts, {})[marker_id] = Pose(rvec, tvec)
    return poses


def average_pose(pose_list: List[Pose]) -> Pose:
    if len(pose_list) == 1:
        return pose_list[0]

    # Average translation
    t = np.mean([p.tvec for p in pose_list], axis=0)

    # Average rotation with SVD on summed rotation matrices
    r_sum = np.zeros((3, 3), dtype=np.float64)
    for p in pose_list:
        rmat, _ = cv2.Rodrigues(p.rvec.reshape(3))
        r_sum += rmat
    u, _, vt = np.linalg.svd(r_sum)
    r_avg = u @ vt
    if np.linalg.det(r_avg) < 0:
        r_avg[:, -1] *= -1
    rvec_avg, _ = cv2.Rodrigues(r_avg)
    return Pose(rvec_avg.reshape(3), t.reshape(3))


def main() -> None:
    base_dir = Path(__file__).resolve().parent
    aruco_path = base_dir / "manual_aruco_poses.csv"
    robot_path = base_dir / "robot_pos.txt"
    r_npy_path = base_dir / R_CAM2GRIPPER_NPY
    t_npy_path = base_dir / T_CAM2GRIPPER_NPY

    if not aruco_path.exists():
        raise FileNotFoundError(f"Missing {aruco_path}")
    if not robot_path.exists():
        raise FileNotFoundError(f"Missing {robot_path}")

    aruco_by_ts = read_aruco_poses(aruco_path)
    robot_poses = read_robot_poses(robot_path)

    timestamps = sorted(aruco_by_ts.keys())
    if len(timestamps) != len(robot_poses):
        raise ValueError(
            "Count mismatch: "
            f"aruco timestamps={len(timestamps)} vs robot poses={len(robot_poses)}. "
            "Please ensure one robot pose per timestamp."
        )

    # Determine marker IDs to process
    all_marker_ids = set.intersection(*[set(aruco_by_ts[ts].keys()) for ts in timestamps])
    if TARGET_MARKER_IDS is not None:
        marker_ids = [mid for mid in TARGET_MARKER_IDS if mid in all_marker_ids]
    else:
        marker_ids = sorted(all_marker_ids)

    if not marker_ids:
        raise ValueError("No common marker IDs across all timestamps. Please check data.")

    pose_modes = [ROBOT_POSE_IS_TCP]
    if TEST_BOTH_ROBOT_POSE_MODES:
        pose_modes = [True, False]

    for marker_id in marker_ids:
        r_gripper2base: List[np.ndarray] = []
        t_gripper2base: List[np.ndarray] = []
        r_target2cam: List[np.ndarray] = []
        t_target2cam: List[np.ndarray] = []
        for robot_pose_is_tcp in pose_modes:
            mode_label = "tcp" if robot_pose_is_tcp else "flange"
            print(f"\n=== Marker ID: {marker_id} | pose={mode_label} ===")

            r_gripper2base = []
            t_gripper2base = []
            r_target2cam = []
            t_target2cam = []

            tool_T = make_tool_T()
            for idx, ts in enumerate(timestamps):
                cam_pose = aruco_by_ts[ts][marker_id]

                # OpenCV expects target->camera
                r_target2cam.append(cam_pose.rvec.reshape(3, 1))
                t_target2cam.append(cam_pose.tvec.reshape(3, 1))

                # Robot pose provided as base->(TCP or flange)
                rob_pose = robot_poses[idx]
                base_T_pose = rvec_tvec_to_T(rob_pose.rvec, rob_pose.tvec)

                if robot_pose_is_tcp:
                    # base->tool given; convert to base->flange
                    base_T_flange = base_T_pose @ invert_T(tool_T)
                else:
                    base_T_flange = base_T_pose

                # We treat gripper as flange frame for hand-eye
                rvec_f, _ = cv2.Rodrigues(base_T_flange[:3, :3])
                r_g2b, t_g2b = invert_pose(rvec_f.reshape(3), base_T_flange[:3, 3])
                r_gripper2base.append(r_g2b.reshape(3, 1))
                t_gripper2base.append(t_g2b.reshape(3, 1))

            use_npy = (not TEST_BOTH_ROBOT_POSE_MODES) and r_npy_path.exists() and t_npy_path.exists()
            if use_npy:
                r_cam2gripper = np.load(r_npy_path)
                t_cam2gripper = np.load(t_npy_path).reshape(3, 1)
                print("Loaded R_cam2gripper from .npy")
                print("Loaded t_cam2gripper from .npy")
            else:
                r_cam2gripper, t_cam2gripper = cv2.calibrateHandEye(
                    r_gripper2base,
                    t_gripper2base,
                    r_target2cam,
                    t_target2cam,
                    method=cv2.CALIB_HAND_EYE_TSAI,
                )

                print("R_cam2gripper:\n", r_cam2gripper)
                print("t_cam2gripper (m):\n", t_cam2gripper.reshape(3))

                out_path = base_dir / f"handeye_result_marker_{marker_id}_{mode_label}.npz"
                np.savez(
                    out_path,
                    R_cam2gripper=r_cam2gripper,
                    t_cam2gripper=t_cam2gripper.reshape(3),
                )
                print(f"Saved results to: {out_path}")

            # Back-substitution validation: base_T_target should be consistent across samples
            cam2gripper_T = np.eye(4, dtype=np.float64)
            cam2gripper_T[:3, :3] = r_cam2gripper
            cam2gripper_T[:3, 3] = t_cam2gripper.reshape(3)
            gripper2cam_T = invert_T(cam2gripper_T)

            base_T_targets: List[np.ndarray] = []
            for idx, ts in enumerate(timestamps):
                rob_pose = robot_poses[idx]  # base->(TCP or flange)
                base_T_pose = rvec_tvec_to_T(rob_pose.rvec, rob_pose.tvec)

                if robot_pose_is_tcp:
                    base_T_gripper = base_T_pose @ invert_T(tool_T)
                else:
                    base_T_gripper = base_T_pose

                cam_pose = aruco_by_ts[ts][marker_id]  # target->cam
                target2cam_T = rvec_tvec_to_T(cam_pose.rvec, cam_pose.tvec)
                cam2target_T = invert_T(target2cam_T)

                base_T_target = base_T_gripper @ gripper2cam_T @ cam2target_T
                base_T_targets.append(base_T_target)

            translations = [T[:3, 3] for T in base_T_targets]
            rotations = [T[:3, :3] for T in base_T_targets]
            t_mean = np.mean(translations, axis=0)
            r_mean = average_rotation(rotations)

            t_errors = [np.linalg.norm(t - t_mean) for t in translations]
            r_errors = [rotation_error_deg(r_mean, r) for r in rotations]

            print("Back-substitution check (base_T_target consistency)")
            print(
                f"Translation error (m): mean={np.mean(t_errors):.6f}, "
                f"std={np.std(t_errors):.6f}, max={np.max(t_errors):.6f}"
            )
            print(
                f"Rotation error (deg): mean={np.mean(r_errors):.6f}, "
                f"std={np.std(r_errors):.6f}, max={np.max(r_errors):.6f}"
            )


if __name__ == "__main__":
    main()
