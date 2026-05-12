import cv2

def detect_cameras(max_index=10):
    available = []
    for i in range(max_index):
        cap = cv2.VideoCapture(i, cv2.CAP_DSHOW)
        if cap.isOpened():
            w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            fps = cap.get(cv2.CAP_PROP_FPS)
            backend = cap.getBackendName()
            available.append((i, w, h, fps, backend))
            cap.release()
    return available

if __name__ == "__main__":
    cameras = detect_cameras()
    if not cameras:
        print("No cameras detected.")
    else:
        print(f"Found {len(cameras)} camera(s):")
        for idx, w, h, fps, backend in cameras:
            print(f"  Index {idx}: {w}x{h} @ {fps:.1f}fps  [{backend}]")
