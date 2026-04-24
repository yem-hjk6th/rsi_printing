"""Open ZED 2i camera and print intrinsic parameters."""
import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
import zed_setup  # noqa: E402 — must precede pyzed
import pyzed.sl as sl

def main():
    zed = sl.Camera()

    init_params = sl.InitParameters()
    init_params.camera_resolution = sl.RESOLUTION.HD720
    init_params.camera_fps = 30
    init_params.depth_mode = sl.DEPTH_MODE.NONE

    status = zed.open(init_params)
    if status != sl.ERROR_CODE.SUCCESS:
        print(f"Failed to open ZED camera: {status}")
        return

    info = zed.get_camera_information()
    calib = info.camera_configuration.calibration_parameters
    res = info.camera_configuration.resolution

    print(f"\n=== ZED 2i Intrinsics ({res.width}x{res.height} @ {info.camera_configuration.fps} fps) ===")

    for side, cam in [("Left", calib.left_cam), ("Right", calib.right_cam)]:
        print(f"\n--- {side} Camera ---")
        print(f"  fx: {cam.fx:.4f}")
        print(f"  fy: {cam.fy:.4f}")
        print(f"  cx: {cam.cx:.4f}")
        print(f"  cy: {cam.cy:.4f}")
        print(f"  FOV (H x V): {cam.h_fov:.2f}° x {cam.v_fov:.2f}°")
        print(f"  Distortion: {[f'{d:.6f}' for d in cam.disto]}")

    baseline = calib.get_camera_baseline()
    print(f"\nBaseline: {baseline:.2f} mm")

    print("\nCamera opened successfully. Press Ctrl+C to exit.")

    image = sl.Mat()
    runtime = sl.RuntimeParameters()
    try:
        while True:
            if zed.grab(runtime) == sl.ERROR_CODE.SUCCESS:
                zed.retrieve_image(image, sl.VIEW.LEFT)
    except KeyboardInterrupt:
        pass
    finally:
        zed.close()
        print("Camera closed.")

if __name__ == "__main__":
    main()
