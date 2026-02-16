import pyzed.sl as sl

zed = sl.Camera()

init_params = sl.InitParameters()
init_params.camera_resolution = sl.RESOLUTION.HD720
init_params.depth_mode = sl.DEPTH_MODE.NONE

if zed.open(init_params) != sl.ERROR_CODE.SUCCESS:
    exit(1)

info = zed.get_camera_information()
calib = info.camera_configuration.calibration_parameters

print("=" * 60)
print("ZED 2i Camera Parameters")
print("=" * 60)

print("\n[Camera Info]")
print(f"Serial Number: {info.serial_number}")
print(f"Firmware: {info.camera_configuration.firmware_version}")
print(f"Model: {info.camera_model}")
print(f"Resolution: {info.camera_configuration.resolution.width}x{info.camera_configuration.resolution.height}")
print(f"FPS: {info.camera_configuration.fps}")

print("\n[Left Camera Intrinsics]")
print(f"Focal Length (fx, fy): ({calib.left_cam.fx:.2f}, {calib.left_cam.fy:.2f})")
print(f"Optical Center (cx, cy): ({calib.left_cam.cx:.2f}, {calib.left_cam.cy:.2f})")
print(f"FOV (H x V): {calib.left_cam.h_fov:.2f}° x {calib.left_cam.v_fov:.2f}°")
print(f"Distortion (k1, k2, k3): ({calib.left_cam.disto[0]:.6f}, {calib.left_cam.disto[1]:.6f}, {calib.left_cam.disto[4]:.6f})")
print(f"Distortion (p1, p2): ({calib.left_cam.disto[2]:.6f}, {calib.left_cam.disto[3]:.6f})")

print("\n[Right Camera Intrinsics]")
print(f"Focal Length (fx, fy): ({calib.right_cam.fx:.2f}, {calib.right_cam.fy:.2f})")
print(f"Optical Center (cx, cy): ({calib.right_cam.cx:.2f}, {calib.right_cam.cy:.2f})")
print(f"FOV (H x V): {calib.right_cam.h_fov:.2f}° x {calib.right_cam.v_fov:.2f}°")
print(f"Distortion (k1, k2, k3): ({calib.right_cam.disto[0]:.6f}, {calib.right_cam.disto[1]:.6f}, {calib.right_cam.disto[4]:.6f})")
print(f"Distortion (p1, p2): ({calib.right_cam.disto[2]:.6f}, {calib.right_cam.disto[3]:.6f})")

print("\n[Stereo Parameters]")
baseline = calib.get_camera_baseline()
print(f"Baseline: {baseline:.4f} mm")
translation = calib.stereo_transform.get_translation().get()
print(f"Stereo Transform (Tx, Ty, Tz): ({translation[0]:.4f}, {translation[1]:.4f}, {translation[2]:.4f}) mm")
orientation = calib.stereo_transform.get_orientation().get()
print(f"Stereo Orientation (x, y, z, w): ({orientation[0]:.4f}, {orientation[1]:.4f}, {orientation[2]:.4f}, {orientation[3]:.4f})")

print("\n[Sensors]")
sensors_data = sl.SensorsData()
if zed.get_sensors_data(sensors_data, sl.TIME_REFERENCE.CURRENT) == sl.ERROR_CODE.SUCCESS:
    imu = sensors_data.get_imu_data()
    print(f"IMU Data Available: Yes")
    print(f"Linear Acceleration: {imu.get_linear_acceleration()}")
    print(f"Angular Velocity: {imu.get_angular_velocity()}")
else:
    print(f"IMU Data Available: No")

print("\n" + "=" * 60)

zed.close()
