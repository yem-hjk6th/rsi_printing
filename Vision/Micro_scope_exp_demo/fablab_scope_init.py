"""fablab_scope_init.py — Open USB digital microscope and preview."""
import cv2
import sys


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


def main():
    cap, idx = find_scope()
    if cap is None:
        print("No microscope found")
        sys.exit(1)

    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"Microscope opened on index {idx}: {w}x{h}")

    cv2.namedWindow("Microscope", cv2.WINDOW_NORMAL)

    while True:
        ret, frame = cap.read()
        if not ret:
            break
        cv2.imshow("Microscope", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
