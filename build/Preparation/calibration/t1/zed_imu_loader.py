import os
import time
from typing import List, Tuple

import numpy as np
import pyzed.sl as sl

OUTPUT_DIR = os.path.join(
    os.path.dirname(__file__),
    "imu",
)
OUTPUT_PATH = os.path.join(OUTPUT_DIR, "zed_imu_samples.npz")

# Capture settings
SAMPLE_COUNT = 200
WARMUP_FRAMES = 50
SLEEP_SEC = 0.01

# ZED settings (edit here)
ZED_RESOLUTION = sl.RESOLUTION.HD1080
ZED_FPS = 30
ZED_DEPTH_MODE = sl.DEPTH_MODE.QUALITY
ZED_COORDINATE_UNITS = sl.UNIT.MILLIMETER


def _vec3_to_list(v) -> List[float]:
    if hasattr(v, "get"):
        return list(v.get())
    return list(v)


def _quat_to_list(q) -> List[float]:
    if hasattr(q, "get"):
        return list(q.get())
    return list(q)


def _timestamp_ns(imu_data) -> int:
    ts = None
    if hasattr(imu_data, "get_timestamp"):
        try:
            ts = imu_data.get_timestamp(sl.TIME_REFERENCE.CURRENT)
        except Exception:
            ts = None
    if ts is None and hasattr(imu_data, "timestamp"):
        ts = imu_data.timestamp
    if ts is None:
        return -1
    if hasattr(ts, "get_nanoseconds"):
        return int(ts.get_nanoseconds())
    if hasattr(ts, "get_milliseconds"):
        return int(ts.get_milliseconds() * 1e6)
    return -1


def main() -> None:
    os.makedirs(OUTPUT_DIR, exist_ok=True)

    zed = sl.Camera()
    init_params = sl.InitParameters()
    init_params.camera_resolution = ZED_RESOLUTION
    init_params.camera_fps = ZED_FPS
    init_params.depth_mode = ZED_DEPTH_MODE
    init_params.coordinate_units = ZED_COORDINATE_UNITS
    init_params.sensors_required = True

    if zed.open(init_params) != sl.ERROR_CODE.SUCCESS:
        raise RuntimeError("Failed to open ZED camera")

    for _ in range(WARMUP_FRAMES):
        zed.grab()
        time.sleep(SLEEP_SEC)

    sensors_data = sl.SensorsData()
    timestamps = []
    accels = []
    gyros = []
    orientations = []

    while len(timestamps) < SAMPLE_COUNT:
        if zed.get_sensors_data(sensors_data, sl.TIME_REFERENCE.CURRENT) != sl.ERROR_CODE.SUCCESS:
            continue
        imu = sensors_data.get_imu_data()
        ts_ns = _timestamp_ns(imu)
        accels.append(_vec3_to_list(imu.get_linear_acceleration()))
        gyros.append(_vec3_to_list(imu.get_angular_velocity()))
        orientations.append(_quat_to_list(imu.get_pose().get_orientation()))
        timestamps.append(ts_ns)
        time.sleep(SLEEP_SEC)

    zed.close()

    np.savez(
        OUTPUT_PATH,
        timestamps_ns=np.array(timestamps, dtype=np.int64),
        linear_accel=np.array(accels, dtype=np.float64),
        angular_velocity=np.array(gyros, dtype=np.float64),
        orientation_quat=np.array(orientations, dtype=np.float64),
    )
    print(f"Saved IMU samples: {OUTPUT_PATH}")


if __name__ == "__main__":
    main()
