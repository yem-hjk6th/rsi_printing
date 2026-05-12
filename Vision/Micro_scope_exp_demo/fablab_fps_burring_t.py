"""fablab_fps_burring_t.py — Microscope preview with FPS & blur metrics."""
import cv2
import sys
import time
import numpy as np


def find_scope(max_idx=10):
    """Try indices starting from 2 (0=ZED, 1=webcam)."""
    for i in range(2, max_idx):
        cap = cv2.VideoCapture(i, cv2.CAP_DSHOW)
        if cap.isOpened():
            ret, frame = cap.read()
            if ret:
                w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
                h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
                name = cap.getBackendName()
                print(f"[idx {i}] {w}x{h} ({name})")
                return cap, i
            cap.release()
    return None, -1


def laplacian_variance(frame):
    """Blur metric: higher = sharper. <100 typically blurry."""
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    return cv2.Laplacian(gray, cv2.CV_64F).var()


def frame_diff(prev_gray, curr_gray):
    """Inter-frame pixel difference as motion/vibration indicator."""
    if prev_gray is None:
        return 0.0
    diff = cv2.absdiff(prev_gray, curr_gray)
    return diff.mean()


def main():
    cap, idx = find_scope()
    if cap is None:
        print("No microscope found")
        sys.exit(1)

    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    reported_fps = cap.get(cv2.CAP_PROP_FPS)
    print(f"Microscope idx={idx}: {w}x{h}, reported FPS={reported_fps:.1f}")

    cv2.namedWindow("Scope FPS+Blur", cv2.WINDOW_NORMAL)

    prev_gray = None
    fps_samples = []
    t_prev = time.perf_counter()
    fps_display = 0.0

    # rolling stats
    blur_history = []
    diff_history = []
    HIST_LEN = 60  # ~2s at 30fps

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        t_now = time.perf_counter()
        dt = t_now - t_prev
        t_prev = t_now

        # FPS (smoothed over last 30 frames)
        fps_samples.append(dt)
        if len(fps_samples) > 30:
            fps_samples.pop(0)
        fps_display = 1.0 / (sum(fps_samples) / len(fps_samples)) if fps_samples else 0

        # blur metric
        blur_val = laplacian_variance(frame)
        blur_history.append(blur_val)
        if len(blur_history) > HIST_LEN:
            blur_history.pop(0)

        # inter-frame diff (vibration proxy)
        curr_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        diff_val = frame_diff(prev_gray, curr_gray)
        diff_history.append(diff_val)
        if len(diff_history) > HIST_LEN:
            diff_history.pop(0)
        prev_gray = curr_gray

        # overlay text
        overlay = frame.copy()
        lines = [
            f"FPS: {fps_display:.1f}  (reported: {reported_fps:.0f})",
            f"Blur: {blur_val:.0f}  (avg: {np.mean(blur_history):.0f}  min: {np.min(blur_history):.0f})",
            f"Diff: {diff_val:.1f}  (avg: {np.mean(diff_history):.1f}  max: {np.max(diff_history):.1f})",
        ]

        # blur quality label
        if blur_val < 50:
            label, color = "VERY BLURRY", (0, 0, 255)
        elif blur_val < 150:
            label, color = "BLURRY", (0, 128, 255)
        elif blur_val < 500:
            label, color = "OK", (0, 200, 200)
        else:
            label, color = "SHARP", (0, 255, 0)
        lines.append(f"Quality: {label}")

        y0 = 30
        for i, line in enumerate(lines):
            c = color if i == 3 else (0, 255, 0)
            cv2.putText(overlay, line, (10, y0 + i * 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 3)
            cv2.putText(overlay, line, (10, y0 + i * 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, c, 2)

        cv2.imshow("Scope FPS+Blur", overlay)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('s'):
            fname = f"scope_snap_{int(time.time())}.png"
            cv2.imwrite(fname, frame)
            print(f"Saved {fname}")

    # summary
    print("\n--- Session Summary ---")
    print(f"Avg FPS: {fps_display:.1f}")
    if blur_history:
        print(f"Blur  — avg: {np.mean(blur_history):.0f}  min: {np.min(blur_history):.0f}  max: {np.max(blur_history):.0f}")
    if diff_history:
        print(f"Diff  — avg: {np.mean(diff_history):.1f}  max: {np.max(diff_history):.1f}")

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
