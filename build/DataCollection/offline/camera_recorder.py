import pyzed.sl as sl
import cv2
import time
import os
from datetime import datetime

# === ZED Camera Configuration ===
class CameraConfig:
    RESOLUTION = sl.RESOLUTION.HD720  # HD720, HD1080, HD2K, VGA
    FPS = 30                          # 15, 30, 60, 100
    DEPTH_MODE = sl.DEPTH_MODE.NONE   # NONE, PERFORMANCE, QUALITY, ULTRA
    DEPTH_MIN = 300                   # mm
    DEPTH_MAX = 10000                 # mm
    COORDINATE_UNITS = sl.UNIT.MILLIMETER
    ENABLE_DEPTH_RECORDING = False
    VIDEO_CODEC = sl.SVO_COMPRESSION_MODE.H264
    OUTPUT_DIR = "recorded_data"

def init_camera(config=CameraConfig):
    zed = sl.Camera()
    init_params = sl.InitParameters()
    init_params.camera_resolution = config.RESOLUTION
    init_params.camera_fps = config.FPS
    init_params.depth_mode = config.DEPTH_MODE
    init_params.coordinate_units = config.COORDINATE_UNITS
    init_params.depth_minimum_distance = config.DEPTH_MIN
    init_params.depth_maximum_distance = config.DEPTH_MAX
    
    if zed.open(init_params) != sl.ERROR_CODE.SUCCESS:
        print("Failed to open ZED camera")
        return None
    return zed

def record_video(zed, output_path, config=CameraConfig):
    rec_params = sl.RecordingParameters()
    rec_params.compression_mode = config.VIDEO_CODEC
    rec_params.video_filename = output_path
    
    if config.ENABLE_DEPTH_RECORDING:
        rec_params.transcode_streaming_input = True
    
    err = zed.enable_recording(rec_params)
    if err != sl.ERROR_CODE.SUCCESS:
        print(f"Recording failed: {err}")
        return False
    return True

def run_recorder(config=CameraConfig, duration=None, output_dir=None):
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    if output_dir is None:
        output_dir = config.OUTPUT_DIR
    output_dir = os.path.join(output_dir, timestamp)
    os.makedirs(output_dir, exist_ok=True)
    
    output_file = os.path.join(output_dir, f"recording_{timestamp}.svo")
    
    zed = init_camera(config)
    if not zed:
        return None
    
    if not record_video(zed, output_file, config):
        zed.close()
        return None
    
    print(f"Recording to: {output_file}")
    print("Press 'q' to stop")
    
    image = sl.Mat()
    runtime_params = sl.RuntimeParameters()
    start_time = time.time()
    frame_count = 0
    
    try:
        while True:
            if zed.grab(runtime_params) == sl.ERROR_CODE.SUCCESS:
                zed.retrieve_image(image, sl.VIEW.LEFT)
                frame = image.get_data()
                
                elapsed = time.time() - start_time
                frame_count += 1
                cv2.putText(frame, f"REC {elapsed:.1f}s | Frame {frame_count}", 
                           (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                cv2.imshow("ZED Recording", frame)
                
                if duration and elapsed >= duration:
                    break
                
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
    except KeyboardInterrupt:
        pass
    finally:
        zed.disable_recording()
        cv2.destroyAllWindows()
        zed.close()
        print(f"Saved: {output_file} ({frame_count} frames, {time.time()-start_time:.1f}s)")
    
    return output_file

if __name__ == "__main__":
    run_recorder()
