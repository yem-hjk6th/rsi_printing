import pyzed.sl as sl
import numpy as np
import cv2
import time

KNOWN_SIZE = None

def init_camera_with_imu():
    zed = sl.Camera()
    init_params = sl.InitParameters()
    init_params.camera_resolution = sl.RESOLUTION.HD720
    init_params.camera_fps = 30
    init_params.depth_mode = sl.DEPTH_MODE.PERFORMANCE
    init_params.coordinate_units = sl.UNIT.MILLIMETER
    init_params.sensors_required = True
    
    if zed.open(init_params) != sl.ERROR_CODE.SUCCESS:
        print("Failed to open ZED camera")
        return None
    
    print("Warming up IMU...")
    for i in range(50):
        zed.grab()
        time.sleep(0.02)
    
    return zed

def get_pose_and_pointcloud(zed):
    runtime_params = sl.RuntimeParameters()
    if zed.grab(runtime_params) != sl.ERROR_CODE.SUCCESS:
        return None, None, None
    
    sensors_data = sl.SensorsData()
    zed.get_sensors_data(sensors_data, sl.TIME_REFERENCE.IMAGE)
    imu_data = sensors_data.get_imu_data()
    
    pose = imu_data.get_pose()
    orientation = pose.get_orientation()
    rotation_matrix = pose.get_rotation_matrix()
    euler = pose.get_euler_angles()
    
    point_cloud = sl.Mat()
    zed.retrieve_measure(point_cloud, sl.MEASURE.XYZRGBA)
    
    image = sl.Mat()
    zed.retrieve_image(image, sl.VIEW.LEFT)
    
    return euler, rotation_matrix, point_cloud, image

def transform_to_world_coords(points_camera, rotation_matrix):
    R = rotation_matrix.r
    points_world = points_camera @ R.T
    return points_world

def detect_object_simple(frame, point_cloud_data):
    h, w = frame.shape[:2]
    center_y, center_x = h // 2, w // 2
    roi_size = 100
    
    y1, y2 = max(0, center_y - roi_size), min(h, center_y + roi_size)
    x1, x2 = max(0, center_x - roi_size), min(w, center_x + roi_size)
    
    roi_points = point_cloud_data[y1:y2, x1:x2].reshape(-1, 4)
    valid = np.isfinite(roi_points[:, 2]) & (roi_points[:, 2] > 0)
    valid_points = roi_points[valid, :3]
    
    return valid_points, (x1, y1, x2, y2)

def measure_object(points_3d):
    if len(points_3d) < 10:
        return None
    
    x_min, x_max = points_3d[:, 0].min(), points_3d[:, 0].max()
    y_min, y_max = points_3d[:, 1].min(), points_3d[:, 1].max()
    z_min, z_max = points_3d[:, 2].min(), points_3d[:, 2].max()
    
    size_x = x_max - x_min
    size_y = y_max - y_min
    size_z = z_max - z_min
    
    return size_x, size_y, size_z

def run_test(known_size=None):
    zed = init_camera_with_imu()
    if not zed:
        return
    
    print("\nIMU Calibration Test")
    print("Place object in center of frame")
    if known_size:
        print(f"Expected size: {known_size}mm")
    print("Press 'q' to quit\n")
    
    cv2.namedWindow("IMU Test", cv2.WINDOW_NORMAL)
    
    try:
        while True:
            result = get_pose_and_pointcloud(zed)
            if result[0] is None:
                continue
            
            euler, rotation_matrix, point_cloud, image = result
            pitch, roll, yaw = euler
            
            frame = image.get_data()[:, :, :3].copy()
            pc_data = point_cloud.get_data()
            
            valid_points, roi = detect_object_simple(frame, pc_data)
            
            if len(valid_points) > 0:
                points_world = transform_to_world_coords(valid_points, rotation_matrix)
                measurement = measure_object(points_world)
                
                if measurement:
                    size_x, size_y, size_z = measurement
                    
                    cv2.rectangle(frame, (roi[0], roi[1]), (roi[2], roi[3]), (0, 255, 0), 2)
                    
                    info_text = [
                        f"Pitch: {pitch:.1f}  Roll: {roll:.1f}  Yaw: {yaw:.1f}",
                        f"Size X: {size_x:.1f}mm  Y: {size_y:.1f}mm  Z: {size_z:.1f}mm",
                    ]
                    
                    if known_size:
                        error_x = abs(size_x - known_size)
                        error_y = abs(size_y - known_size)
                        info_text.append(f"Error: X={error_x:.1f}mm  Y={error_y:.1f}mm")
                    
                    for i, text in enumerate(info_text):
                        cv2.putText(frame, text, (10, 30 + i * 30), 
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            else:
                cv2.putText(frame, f"Pitch: {pitch:.1f}  Roll: {roll:.1f}  Yaw: {yaw:.1f}", 
                           (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
                cv2.putText(frame, "No valid points in ROI", 
                           (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            
            h, w = frame.shape[:2]
            cv2.line(frame, (w//2 - 100, h//2), (w//2 + 100, h//2), (255, 0, 0), 2)
            cv2.line(frame, (w//2, h//2 - 100), (w//2, h//2 + 100), (255, 0, 0), 2)
            
            cv2.imshow("IMU Test", frame)
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        zed.close()

if __name__ == "__main__":
    run_test(known_size=KNOWN_SIZE)
