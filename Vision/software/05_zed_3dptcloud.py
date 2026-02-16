import pyzed.sl as sl
import numpy as np
import open3d as o3d
import cv2

zed = sl.Camera()

init_params = sl.InitParameters()
init_params.camera_resolution = sl.RESOLUTION.HD720
init_params.camera_fps = 30
init_params.depth_mode = sl.DEPTH_MODE.PERFORMANCE
init_params.coordinate_units = sl.UNIT.MILLIMETER

if zed.open(init_params) != sl.ERROR_CODE.SUCCESS:
    print("Failed to open camera")
    exit(1)

print("Warming up camera (10 frames)...")
runtime_params = sl.RuntimeParameters()
for i in range(10):
    zed.grab(runtime_params)

point_cloud = sl.Mat()
image = sl.Mat()
depth = sl.Mat()

vis = o3d.visualization.Visualizer()
vis.create_window(window_name="ZED Point Cloud", width=1280, height=720)
pcd = o3d.geometry.PointCloud()
vis.add_geometry(pcd)
render_opt = vis.get_render_option()
render_opt.point_size = 2.0
render_opt.background_color = np.array([0, 0, 0])

print("Press 'Q' in the window to quit")

cv2.namedWindow("Center Point Depth", cv2.WINDOW_NORMAL)

frame_count = 0
skip_frames = 2
xyz = None
colors = None
try:
    while True:
        if zed.grab(runtime_params) == sl.ERROR_CODE.SUCCESS:
            frame_count += 1
            
            zed.retrieve_image(image, sl.VIEW.LEFT)
            zed.retrieve_measure(depth, sl.MEASURE.DEPTH)
            
            img_data = image.get_data()
            depth_data = depth.get_data()
            
            h, w = depth_data.shape
            center_x, center_y = w // 2, h // 2
            center_depth = depth_data[center_y, center_x]
            
            rgb_frame = img_data[:, :, :3].copy()
            cv2.line(rgb_frame, (center_x - 20, center_y), (center_x + 20, center_y), (0, 255, 0), 2)
            cv2.line(rgb_frame, (center_x, center_y - 20), (center_x, center_y + 20), (0, 255, 0), 2)
            
            if np.isfinite(center_depth) and center_depth > 0:
                depth_text = f"CP: {center_depth:.0f} mm"
            else:
                depth_text = "CP: N/A"
            
            cv2.putText(rgb_frame, depth_text, (center_x + 30, center_y - 10), 
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            cv2.imshow("Center Point Depth", rgb_frame)
            
            if frame_count % skip_frames == 0:
                zed.retrieve_measure(point_cloud, sl.MEASURE.XYZRGBA)
                pc_data = point_cloud.get_data()
                
                pts = pc_data.reshape(-1, 4)
                colors_img = img_data.reshape(-1, 4)[:, :3]
                valid = np.isfinite(pts[:, 2]) & (pts[:, 2] > 0) & (pts[:, 2] < 10000)
                xyz = pts[valid, :3] / 1000.0
                colors = colors_img[valid, :3] / 255.0
                
                if frame_count % 30 == 0:
                    print(f"Frame {frame_count}: Valid points = {xyz.shape[0]}")
                
                if xyz.shape[0] > 100:
                    pcd.points = o3d.utility.Vector3dVector(xyz)
                    pcd.colors = o3d.utility.Vector3dVector(colors)
                    vis.update_geometry(pcd)
                    if frame_count == skip_frames:
                        vis.reset_view_point(True)
                vis.poll_events()
                vis.update_renderer()
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        else:
            vis.poll_events()
            vis.update_renderer()
finally:
    cv2.destroyAllWindows()
    vis.destroy_window()
    zed.close()
