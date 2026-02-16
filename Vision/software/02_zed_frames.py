import pyzed.sl as sl
import cv2
import numpy as np
import os
import time
from datetime import datetime

script_dir = os.path.dirname(os.path.abspath(__file__))
save_dir = os.path.join(script_dir, "t_file")
os.makedirs(save_dir, exist_ok=True)

zed = sl.Camera()

init_params = sl.InitParameters()
init_params.camera_resolution = sl.RESOLUTION.HD720
init_params.camera_fps = 30
init_params.depth_mode = sl.DEPTH_MODE.PERFORMANCE
init_params.coordinate_units = sl.UNIT.METER
init_params.sensors_required = True

if zed.open(init_params) != sl.ERROR_CODE.SUCCESS:
    exit(1)

image = sl.Mat()
depth = sl.Mat()
runtime_params = sl.RuntimeParameters()
sensors_data = sl.SensorsData()

print("Press 's' to capture 5 frames, 'q' to quit")

while True:
    if zed.grab(runtime_params) == sl.ERROR_CODE.SUCCESS:
        zed.retrieve_image(image, sl.VIEW.LEFT)
        frame = image.get_data()
        cv2.imshow("ZED", frame)
        
        key = cv2.waitKey(1) & 0xFF
        
        if key == ord('s'):
            print("\nCapturing 5 frames...")
            for i in range(5):
                if zed.grab(runtime_params) == sl.ERROR_CODE.SUCCESS:
                    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                    
                    zed.retrieve_image(image, sl.VIEW.LEFT)
                    rgb = image.get_data()
                    cv2.imwrite(f"{save_dir}/{timestamp}_frame{i+1}_rgb.png", rgb)
                    
                    zed.retrieve_measure(depth, sl.MEASURE.DEPTH)
                    depth_data = depth.get_data()
                    np.save(f"{save_dir}/{timestamp}_frame{i+1}_depth.npy", depth_data)
                    
                    zed.get_sensors_data(sensors_data, sl.TIME_REFERENCE.IMAGE)
                    imu = sensors_data.get_imu_data()
                    
                    with open(f"{save_dir}/{timestamp}_frame{i+1}_imu.txt", "w") as f:
                        f.write(f"Linear Acceleration: {imu.get_linear_acceleration()}\n")
                        f.write(f"Angular Velocity: {imu.get_angular_velocity()}\n")
                        f.write(f"Orientation: {imu.get_pose().get_orientation()}\n")
                    
                    print(f"Frame {i+1}/5 saved")
                    time.sleep(1)
            print("Capture complete!\n")
        
        elif key == ord('q'):
            break

cv2.destroyAllWindows()
zed.close()
