"""
TESLONG NTG100 Borescope - startup / preview demo
The NTG100 is a UVC device; OpenCV treats it as a standard webcam.
"""

import cv2
import sys


def find_borescope(max_index: int = 6) -> int:
    """Scan indices 0..max_index and return the first one that opens."""
    for idx in range(max_index):
        cap = cv2.VideoCapture(idx, cv2.CAP_DSHOW)
        if cap.isOpened():
            ret, _ = cap.read()
            cap.release()
            if ret:
                print(f"[borescope] found camera at index {idx}")
                return idx
    return -1


def main(camera_index: int = -1):
    if camera_index < 0:
        camera_index = find_borescope()
    if camera_index < 0:
        print("[borescope] ERROR: no camera found. Check USB connection.")
        sys.exit(1)

    cap = cv2.VideoCapture(camera_index, cv2.CAP_DSHOW)
    # NTG100 native resolution
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
    cap.set(cv2.CAP_PROP_FPS, 30)

    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = cap.get(cv2.CAP_PROP_FPS)
    print(f"[borescope] opened: {w}x{h} @ {fps:.1f} fps  (index={camera_index})")
    print("  Press  s  to save snapshot,  q / ESC  to quit")

    snap_count = 0
    while True:
        ret, frame = cap.read()
        if not ret:
            print("[borescope] frame read failed — camera disconnected?")
            break

        cv2.imshow("NTG100 Borescope", frame)
        key = cv2.waitKey(1) & 0xFF

        if key in (ord('q'), 27):          # q or ESC
            break
        elif key == ord('s'):
            fname = f"snapshot_{snap_count:04d}.png"
            cv2.imwrite(fname, frame)
            print(f"[borescope] saved {fname}")
            snap_count += 1

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    # Pass a specific index on the command line if auto-detect picks wrong one:
    # python demo1_init.py 2
    idx = int(sys.argv[1]) if len(sys.argv) > 1 else -1
    main(idx)
