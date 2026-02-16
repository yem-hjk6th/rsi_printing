import os
import numpy as np

IMU_PATH = os.path.join(
    os.path.dirname(__file__),
    "imu",
    "zed_imu_samples.npz",
)
ARUCO_REC_PATH = os.path.join(os.path.dirname(__file__), "ArUco_rec.py")


def _format_np_array(name: str, arr: np.ndarray) -> str:
    flat = arr.tolist()
    return f"{name} = np.array({flat}, dtype=np.float64)\n"


def _update_aruco_rec(imu_ts: int, accel: np.ndarray, gyro: np.ndarray, quat: np.ndarray) -> None:
    with open(ARUCO_REC_PATH, "r", encoding="utf-8") as f:
        text = f.read()

    start = text.find("# IMU_HARDCODED_START")
    end = text.find("# IMU_HARDCODED_END")
    if start == -1 or end == -1:
        raise RuntimeError("IMU hardcoded block not found in ArUco_rec.py")

    block = []
    block.append("# IMU_HARDCODED_START\n")
    block.append(f"IMU_SAMPLE_TIMESTAMP_NS = {imu_ts}\n")
    block.append(_format_np_array("IMU_LINEAR_ACCEL", accel))
    block.append(_format_np_array("IMU_ANGULAR_VELOCITY", gyro))
    block.append(_format_np_array("IMU_ORIENTATION_QUAT", quat))
    block.append("# IMU_HARDCODED_END\n")

    new_text = text[:start] + "".join(block) + text[end + len("# IMU_HARDCODED_END"):]

    with open(ARUCO_REC_PATH, "w", encoding="utf-8") as f:
        f.write(new_text)


def main() -> None:
    data = np.load(IMU_PATH)
    timestamps = data["timestamps_ns"]
    accels = data["linear_accel"]
    gyros = data["angular_velocity"]
    quats = data["orientation_quat"]

    imu_ts = int(timestamps[-1])
    accel = accels[-1]
    gyro = gyros[-1]
    quat = quats[-1]

    _update_aruco_rec(imu_ts, accel, gyro, quat)
    print(f"Applied IMU constants to: {ARUCO_REC_PATH}")


if __name__ == "__main__":
    main()
