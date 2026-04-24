import cv2

CAM_INDEX = 1  # Teslong endoscope index from cam_port.py

cap = cv2.VideoCapture(CAM_INDEX, cv2.CAP_DSHOW)
if not cap.isOpened():
    print(f"Failed to open camera at index {CAM_INDEX}")
    exit(1)

cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
print(f"Teslong endoscope opened: {w}x{h}")
print("Press 'q' to quit, 's' to save a snapshot.")

snapshot_count = 0
while True:
    ret, frame = cap.read()
    if not ret:
        print("Frame read failed.")
        break

    cv2.imshow("Teslong Endoscope", frame)
    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        break
    elif key == ord('s'):
        fname = f"snapshot_{snapshot_count:03d}.png"
        cv2.imwrite(fname, frame)
        print(f"Saved {fname}")
        snapshot_count += 1

cap.release()
cv2.destroyAllWindows()
