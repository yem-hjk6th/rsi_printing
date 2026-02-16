import pyzed.sl as sl
import cv2
import numpy as np
import time

zed = sl.Camera()

init_params = sl.InitParameters()
init_params.camera_resolution = sl.RESOLUTION.HD720
init_params.camera_fps = 0
init_params.depth_mode = sl.DEPTH_MODE.PERFORMANCE
init_params.coordinate_units = sl.UNIT.MILLIMETER

if zed.open(init_params) != sl.ERROR_CODE.SUCCESS:
    print("Failed to open camera")
    exit(1)

image = sl.Mat()
depth = sl.Mat()
point_cloud = sl.Mat()
runtime_params = sl.RuntimeParameters()

print("Testing depth and point cloud performance...")
print("Press 'q' to quit")

fps_list = []
start_time = time.time()
frame_count = 0

while True:
    frame_start = time.time()
    
    if zed.grab(runtime_params) == sl.ERROR_CODE.SUCCESS:
        zed.retrieve_image(image, sl.VIEW.LEFT)
        zed.retrieve_measure(depth, sl.MEASURE.DEPTH)
        zed.retrieve_measure(point_cloud, sl.MEASURE.XYZRGBA)
        
        frame = image.get_data()[:, :, :3].copy()
        depth_map = depth.get_data().copy()
        
        depth_normalized = cv2.normalize(depth_map, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U)
        depth_colored = cv2.applyColorMap(depth_normalized, cv2.COLORMAP_JET)
        
        frame_time = time.time() - frame_start
        fps = 1.0 / frame_time if frame_time > 0 else 0
        fps_list.append(fps)
        frame_count += 1
        
        cv2.putText(frame, f"FPS: {fps:.1f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.putText(depth_colored, f"FPS: {fps:.1f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        
        combined = np.hstack([frame, depth_colored])
        cv2.imshow("RGB | Depth", combined)
        
        if frame_count % 30 == 0:
            avg_fps = np.mean(fps_list[-30:])
            print(f"Frames: {frame_count}, Avg FPS: {avg_fps:.1f}, Point Cloud Size: {point_cloud.get_width()}x{point_cloud.get_height()}")
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

total_time = time.time() - start_time
avg_fps = frame_count / total_time

print(f"\n=== Test Results ===")
print(f"Total Frames: {frame_count}")
print(f"Total Time: {total_time:.1f}s")
print(f"Average FPS: {avg_fps:.1f}")
print(f"Resolution: {image.get_width()}x{image.get_height()}")
print(f"Depth Mode: PERFORMANCE (CPU)")

cv2.destroyAllWindows()
zed.close()
