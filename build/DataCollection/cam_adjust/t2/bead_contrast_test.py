"""
bead_contrast_test.py — 提取打印帧，手动标出 bead 大致区域
在 bead 线的不同 X 位置取垂直切面 profile
判断 bead 边缘在中心 vs 边角是否都能被检测
"""

import sys, os, cv2, numpy as np
import pyzed.sl as sl

SVO_PATH = r"C:\Users\dell\Desktop\RSI\recorded_data\20260310_183338\recording_20260310_183338.svo2"
OUT_DIR = os.path.join(os.path.dirname(__file__), "feasibility_output")

# 有效帧范围 500-4800，均匀取 3 帧
TEST_FRAMES = [1500, 2700, 4000]


def main():
    os.makedirs(OUT_DIR, exist_ok=True)

    zed = sl.Camera()
    init_p = sl.InitParameters()
    init_p.set_from_svo_file(SVO_PATH)
    init_p.svo_real_time_mode = False
    init_p.depth_mode = sl.DEPTH_MODE.NONE
    status = zed.open(init_p)
    if status != sl.ERROR_CODE.SUCCESS:
        print(f"ZED open failed: {status}")
        sys.exit(1)

    info = zed.get_camera_information().camera_configuration
    W, H = info.resolution.width, info.resolution.height
    print(f"Resolution: {W}x{H}")

    image_mat = sl.Mat()
    runtime = sl.RuntimeParameters()

    for fidx in TEST_FRAMES:
        zed.set_svo_position(fidx)
        if zed.grab(runtime) != sl.ERROR_CODE.SUCCESS:
            continue

        zed.retrieve_image(image_mat, sl.VIEW.LEFT)
        frame = image_mat.get_data()[:, :, :3].copy()
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # 找蓝色区域 (打印底板)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        blue_mask = cv2.inRange(hsv, np.array([100, 80, 50]), np.array([130, 255, 255]))
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (15, 15))
        blue_mask = cv2.morphologyEx(blue_mask, cv2.MORPH_CLOSE, kernel, iterations=3)

        # 在蓝色区域内做边缘检测
        gray_masked = gray.copy()
        gray_masked[blue_mask == 0] = 0

        # Sobel Y (水平方向的 bead 线，垂直梯度)
        sobel_y = cv2.Sobel(gray_masked, cv2.CV_64F, 0, 1, ksize=3)
        abs_sobel = np.abs(sobel_y).astype(np.uint8)

        # Canny
        blur = cv2.GaussianBlur(gray_masked, (3, 3), 0.5)
        canny = cv2.Canny(blur, 20, 60)
        canny[blue_mask == 0] = 0

        # 在蓝色区域横向取 7 个垂直切面
        ys, xs = np.where(blue_mask > 0)
        if len(xs) == 0:
            print(f"Frame {fidx}: no blue region found")
            continue

        x_min, x_max = xs.min(), xs.max()
        sample_xs = np.linspace(x_min + 50, x_max - 50, 7).astype(int)

        cx, cy = W // 2, H // 2
        diag = np.sqrt(W**2 + H**2)

        # 创建 profile 可视化
        n_samples = len(sample_xs)
        prof_h = 300
        prof_w = 160
        profile_canvas = np.zeros((prof_h, prof_w * n_samples, 3), dtype=np.uint8)

        vis = frame.copy()

        print(f"\nFrame {fidx}:")
        print(f"{'Col':>5} {'X':>5} {'dist%':>6} {'GradMax':>8} {'Contrast':>9} {'Width_est':>9}")
        print("-" * 55)

        for i, sx in enumerate(sample_xs):
            # 取该 X 列在蓝色区域内的范围
            col_mask = blue_mask[:, sx]
            rows = np.where(col_mask > 0)[0]
            if len(rows) < 20:
                continue
            y_top, y_bot = rows.min(), rows.max()

            # 灰度 profile
            profile = gray[y_top:y_bot, sx].astype(np.float64)
            if len(profile) < 20:
                continue

            # 平滑后求梯度
            smooth = cv2.GaussianBlur(profile.reshape(-1, 1), (5, 1), 1.0).flatten()
            grad = np.gradient(smooth)
            grad_max = np.max(np.abs(grad))

            # 对比度: profile 的标准差 (bead 如果存在会增大 std)
            contrast = np.std(profile)

            # 粗估 bead 宽度: 梯度超过 max/2 的连续区间
            threshold = grad_max * 0.5
            above = np.abs(grad) > threshold
            transitions = np.diff(above.astype(int))
            rises = np.where(transitions == 1)[0]
            falls = np.where(transitions == -1)[0]
            width_est = "-"
            if len(rises) >= 2:
                width_est = f"{rises[1] - rises[0]}px"
            elif len(rises) >= 1 and len(falls) >= 1:
                width_est = f"~{falls[0] - rises[0]}px"

            # 距中心
            dist = np.sqrt((sx - cx)**2 + ((y_top + y_bot)//2 - cy)**2) / diag

            print(f"  S{i}  {sx:5d}  {dist*100:5.1f}%  {grad_max:7.1f}  {contrast:8.1f}  {width_est:>9}")

            # 画垂直线
            color = (0, 255, 0) if dist <= 1/3 else (0, 128, 255)
            cv2.line(vis, (sx, y_top), (sx, y_bot), color, 1)
            cv2.putText(vis, f"S{i}", (sx-10, y_top-5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)

            # 画 profile
            p_x0 = i * prof_w
            # 归一化 profile 到 prof_h
            p_norm = ((profile - profile.min()) / (profile.max() - profile.min() + 1) * (prof_h - 40) + 20).astype(int)
            g_norm = ((np.abs(grad) / (grad_max + 1)) * (prof_h - 40) + 20).astype(int)

            step = max(1, len(p_norm) // (prof_h - 20))
            for r in range(0, len(p_norm) - 1, step):
                ry = int(r / len(p_norm) * (prof_h - 1))
                ry2 = int((r+1) / len(p_norm) * (prof_h - 1))
                # gray profile
                cv2.line(profile_canvas,
                         (p_x0 + p_norm[r] * prof_w // (prof_h + 1), ry),
                         (p_x0 + p_norm[r+1] * prof_w // (prof_h + 1), ry2),
                         (200, 200, 200), 1)
                # gradient
                cv2.line(profile_canvas,
                         (p_x0 + g_norm[r] * prof_w // (prof_h + 1), ry),
                         (p_x0 + g_norm[r+1] * prof_w // (prof_h + 1), ry2),
                         (0, 255, 0), 1)

            label = f"S{i} d={dist*100:.0f}%"
            cv2.putText(profile_canvas, label, (p_x0+5, 15),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0,255,255), 1)

        # 画 1/3 圆
        r13 = int(diag / 3)
        cv2.circle(vis, (cx, cy), r13, (0, 255, 255), 1)

        cv2.imwrite(os.path.join(OUT_DIR, f"bead_frame{fidx}_lines.png"), vis)
        cv2.imwrite(os.path.join(OUT_DIR, f"bead_frame{fidx}_profiles.png"), profile_canvas)

        # 额外: 保存 canny 叠加
        canny_vis = frame.copy()
        canny_vis[canny > 0] = [0, 255, 0]
        cv2.imwrite(os.path.join(OUT_DIR, f"bead_frame{fidx}_canny.png"), canny_vis)

    zed.close()
    print(f"\n输出: {OUT_DIR}")


if __name__ == "__main__":
    main()
