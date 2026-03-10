"""
mono_preview.py — 单目全分辨率预览，用于调整相机位置
按 'l'/'r' 切换左右目，按 'q' 退出
"""

import cv2
import numpy as np
import pyzed.sl as sl

def main():
    zed = sl.Camera()
    init_p = sl.InitParameters()
    init_p.camera_resolution = sl.RESOLUTION.HD2K
    init_p.camera_fps = 15
    init_p.depth_mode = sl.DEPTH_MODE.PERFORMANCE

    status = zed.open(init_p)
    if status != sl.ERROR_CODE.SUCCESS:
        print(f"ZED open failed: {status}")
        return

    info = zed.get_camera_information().camera_configuration
    W, H = info.resolution.width, info.resolution.height
    print(f"Resolution: {W}x{H}")
    print("Controls: 'l' = left eye, 'r' = right eye, 'q' = quit")

    image = sl.Mat()
    depth_map = sl.Mat()
    runtime = sl.RuntimeParameters()

    cx, cy = W // 2, H // 2
    diag = np.sqrt(W**2 + H**2)
    r13 = int(diag / 3)

    current_view = sl.VIEW.LEFT
    view_label = "LEFT"

    # 获取屏幕尺寸，计算缩放比
    cv2.namedWindow("Mono Preview", cv2.WINDOW_NORMAL)
    screen_w, screen_h = 1920, 1080  # 默认值
    try:
        from ctypes import windll
        screen_w = windll.user32.GetSystemMetrics(0)
        screen_h = windll.user32.GetSystemMetrics(1)
    except Exception:
        pass
    # 留出任务栏空间
    usable_h = screen_h - 60
    scale = min(screen_w / W, usable_h / H)
    win_w, win_h = int(W * scale), int(H * scale)
    cv2.resizeWindow("Mono Preview", win_w, win_h)

    while True:
        if zed.grab(runtime) != sl.ERROR_CODE.SUCCESS:
            continue

        zed.retrieve_image(image, current_view)
        zed.retrieve_measure(depth_map, sl.MEASURE.DEPTH)

        frame = image.get_data()[:, :, :3].copy()

        err, depth_val = depth_map.get_value(cx, cy)
        if np.isfinite(depth_val):
            depth_text = f"Depth: {depth_val:.0f}mm ({depth_val/1000:.2f}m)"
        else:
            depth_text = "Depth: N/A"

        cv2.drawMarker(frame, (cx, cy), (0, 0, 255), cv2.MARKER_CROSS, 40, 2)
        cv2.circle(frame, (cx, cy), r13, (0, 255, 255), 1)
        cv2.putText(frame, f"{view_label} | {W}x{H}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(frame, depth_text, (10, 65),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        cv2.putText(frame, "'l'=left  'r'=right  'q'=quit", (10, H - 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1)

        cv2.imshow("Mono Preview", frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('l'):
            current_view = sl.VIEW.LEFT
            view_label = "LEFT"
        elif key == ord('r'):
            current_view = sl.VIEW.RIGHT
            view_label = "RIGHT"

    zed.close()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
