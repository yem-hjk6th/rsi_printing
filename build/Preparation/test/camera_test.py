import os
import time
from datetime import datetime
import cv2
import pyzed.sl as sl

# === ZED Camera Test Recorder ===
# 2s recording test for res test
# 可选分辨率：
#   sl.RESOLUTION.HD2K  (2208x1242)
#   sl.RESOLUTION.HD1080 (1920x1080)
#   sl.RESOLUTION.HD720  (1280x720)
#   sl.RESOLUTION.VGA    (672x376)
# 可选深度模式：
#   sl.DEPTH_MODE.NONE / PERFORMANCE / QUALITY / ULTRA

class CameraTestConfig:
    RESOLUTION = sl.RESOLUTION.HD2K
    FPS = 30
    ENABLE_DEPTH = True
    DEPTH_MODE = sl.DEPTH_MODE.ULTRA
    COORDINATE_UNITS = sl.UNIT.MILLIMETER
    VIDEO_CODEC = sl.SVO_COMPRESSION_MODE.H264
    OUTPUT_DIR = os.path.join("recorded_data", "res_test")


def init_camera(config=CameraTestConfig):
    zed = sl.Camera()
    init_params = sl.InitParameters()
    init_params.camera_resolution = config.RESOLUTION
    init_params.camera_fps = config.FPS
    init_params.depth_mode = config.DEPTH_MODE if config.ENABLE_DEPTH else sl.DEPTH_MODE.NONE
    init_params.coordinate_units = config.COORDINATE_UNITS

    if zed.open(init_params) != sl.ERROR_CODE.SUCCESS:
        print("Failed to open ZED camera")
        return None
    return zed


def start_recording(zed, output_path, config=CameraTestConfig):
    rec_params = sl.RecordingParameters()
    rec_params.video_filename = output_path
    rec_params.compression_mode = config.VIDEO_CODEC

    err = zed.enable_recording(rec_params)
    if err != sl.ERROR_CODE.SUCCESS:
        print(f"Recording failed: {err}")
        return False
    return True


def run_test_recording(config=CameraTestConfig, duration=2.0):
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    output_dir = config.OUTPUT_DIR
    os.makedirs(output_dir, exist_ok=True)

    output_file = os.path.join(output_dir, f"recording_{timestamp}.svo")

    zed = init_camera(config)
    if not zed:
        return None

    if not start_recording(zed, output_file, config):
        zed.close()
        return None

    print(f"Recording to: {output_file}")
    print(f"Recording for {duration:.1f}s...")

    runtime_params = sl.RuntimeParameters()
    start_time = time.time()

    try:
        while True:
            if zed.grab(runtime_params) == sl.ERROR_CODE.SUCCESS:
                if time.time() - start_time >= duration:
                    break
    except KeyboardInterrupt:
        pass
    finally:
        zed.disable_recording()
        zed.close()
        print(f"Saved: {output_file}")

    return output_file


if __name__ == "__main__":
    run_test_recording()
