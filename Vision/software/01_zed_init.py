import pyzed.sl as sl
import cv2

zed = sl.Camera()

init_params = sl.InitParameters()
init_params.camera_resolution = sl.RESOLUTION.HD720
init_params.camera_fps = 30
init_params.depth_mode = sl.DEPTH_MODE.NONE

if zed.open(init_params) != sl.ERROR_CODE.SUCCESS:
    exit(1)

image = sl.Mat()
runtime_params = sl.RuntimeParameters()

print("Press 'q' to quit")

while True:
    if zed.grab(runtime_params) == sl.ERROR_CODE.SUCCESS:
        zed.retrieve_image(image, sl.VIEW.LEFT)
        frame = image.get_data()
        cv2.imshow("ZED", frame)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

cv2.destroyAllWindows()
zed.close()
