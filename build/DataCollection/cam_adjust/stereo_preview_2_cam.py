"""
stereo_preview.py — 实时双目预览，用于调整相机位置
左目 + 右目 并排显示，按 q 退出
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
    print(f"Resolution: {W}x{H} | press 'q' to quit")

    img_left = sl.Mat()
    img_right = sl.Mat()
    depth_map = sl.Mat()
    runtime = sl.RuntimeParameters()

    cx, cy = W // 2, H // 2
    diag = np.sqrt(W**2 + H**2)
    r13 = int(diag / 3)

    while True:
        if zed.grab(runtime) != sl.ERROR_CODE.SUCCESS:
            continue

        zed.retrieve_image(img_left, sl.VIEW.LEFT)
        zed.retrieve_image(img_right, sl.VIEW.RIGHT)
        zed.retrieve_measure(depth_map, sl.MEASURE.DEPTH)

        left = img_left.get_data()[:, :, :3].copy()
        right = img_right.get_data()[:, :, :3].copy()

        # 读取中心点深度 (左目坐标系)
        err, depth_val = depth_map.get_value(cx, cy)
        if np.isfinite(depth_val):
            depth_mm = depth_val
            depth_text = f"Depth: {depth_mm:.0f}mm ({depth_mm/1000:.2f}m)"
        else:
            depth_text = "Depth: N/A"

        # 中心红色十字 + 1/3 对角线圆 + 深度信息
        for frame, label in [(left, "LEFT"), (right, "RIGHT")]:
            cv2.drawMarker(frame, (cx, cy), (0, 0, 255), cv2.MARKER_CROSS, 40, 2)
            cv2.circle(frame, (cx, cy), r13, (0, 255, 255), 1)
            cv2.putText(frame, label, (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            cv2.putText(frame, depth_text, (10, 65),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

        combined = np.hstack([left, right])
        scale = min(1.0, 1600 / combined.shape[1])
        if scale < 1.0:
            combined = cv2.resize(combined, None, fx=scale, fy=scale)

        cv2.imshow("Stereo Preview — press 'q' to quit", combined)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    zed.close()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
