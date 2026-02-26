"""
ROI_cut.py — Click to mark nozzle, extract 224x224 ROI, measure distance.

Usage:
  1. Run script → live ZED preview appears
  2. Left-click on nozzle → prints pixel coords, draws 224x224 ROI box
  3. Press 's' → saves the ROI crop as PNG
  4. Right-click a second point → measures 3D distance between the two clicks
  5. Press 'r' → reset all points
  6. Press 'q' → quit

Extrinsics: R6 best result (2.884 mm mean error)
"""

import time
from pathlib import Path

import cv2
import numpy as np
import pyzed.sl as sl

# ─── Configuration ────────────────────────────────────────────────────────────

RESOLUTION = sl.RESOLUTION.HD1080       # 1920x1080
FPS = 30
DEPTH_MODE = sl.DEPTH_MODE.ULTRA
ROI_SIZE = 224                          # standard CNN input (ResNet/VGG/MobileNet)
DEPTH_PATCH_RADIUS = 2
WINDOW_NAME = "ROI Cut  |  L-click=mark  R-click=measure  s=save  r=reset  q=quit"

EXTRINSICS_PATH = Path(__file__).parent / "extrinsics_R6.txt"

# R6 best T_cam2gripper (fallback if file missing)
T_CAM2GRIPPER_R6 = np.array([
    [ 0.047495,  0.811369,  0.582601, -0.306804],
    [ 0.998643, -0.051054, -0.010310, -0.077421],
    [ 0.021379,  0.582300, -0.812693,  0.388133],
    [ 0.0,       0.0,       0.0,       1.0      ]
], dtype=np.float64)


# ─── Globals ──────────────────────────────────────────────────────────────────

clicked_pt = None           # (u, v) of last left-click
measure_pts = []            # list of (u, v) for distance measurement (max 2)
frozen_frame = None         # frame snapshot at click time
roi_crop = None             # extracted 224x224 ROI


# ─── Helpers ──────────────────────────────────────────────────────────────────

def parse_extrinsics(path):
    """Load 4x4 matrix from text file (skip comment lines starting with #)."""
    path = Path(path)
    if not path.exists():
        return None
    rows = []
    for line in path.read_text(encoding="utf-8").splitlines():
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        vals = [float(x) for x in line.split()]
        if len(vals) == 4:
            rows.append(vals)
    return np.array(rows, dtype=np.float64) if len(rows) == 4 else None


def robust_xyz(point_cloud, u, v, patch_radius=2):
    """Median-filtered 3D point from point cloud with patch sampling."""
    h, w = point_cloud.get_height(), point_cloud.get_width()
    xs, ys, zs = [], [], []
    for dv in range(-patch_radius, patch_radius + 1):
        for du in range(-patch_radius, patch_radius + 1):
            uu = int(np.clip(u + du, 0, w - 1))
            vv = int(np.clip(v + dv, 0, h - 1))
            err, val = point_cloud.get_value(uu, vv)
            if err != sl.ERROR_CODE.SUCCESS:
                continue
            x, y, z = float(val[0]), float(val[1]), float(val[2])
            if np.isfinite(x) and np.isfinite(y) and np.isfinite(z) and z > 0:
                xs.append(x); ys.append(y); zs.append(z)
    if not xs:
        return None
    return np.array([np.median(xs), np.median(ys), np.median(zs)], dtype=np.float64)


def extract_roi(frame, cx, cy, size=ROI_SIZE):
    """Extract size×size ROI centered at (cx, cy), clamped to image bounds."""
    h, w = frame.shape[:2]
    half = size // 2

    # clamp center so ROI stays inside frame
    cx = max(half, min(cx, w - half - (size % 2 == 0)))
    cy = max(half, min(cy, h - half - (size % 2 == 0)))

    x1 = cx - half
    y1 = cy - half
    return frame[y1:y1 + size, x1:x1 + size].copy(), (x1, y1, x1 + size, y1 + size)


def draw_roi_box(frame, bbox, color=(0, 255, 0), thickness=2):
    """Draw rectangle for ROI region."""
    x1, y1, x2, y2 = bbox
    cv2.rectangle(frame, (x1, y1), (x2, y2), color, thickness)


def draw_crosshair(frame, pt, size=12, color=(0, 0, 255), thickness=2):
    """Draw crosshair at pixel location."""
    u, v = int(pt[0]), int(pt[1])
    cv2.line(frame, (u - size, v), (u + size, v), color, thickness)
    cv2.line(frame, (u, v - size), (u, v + size), color, thickness)
    cv2.circle(frame, (u, v), 3, color, -1)


# ─── Mouse callback ──────────────────────────────────────────────────────────

def on_mouse(event, x, y, flags, param):
    global clicked_pt, measure_pts, frozen_frame, roi_crop

    frame_snap = param.get("frame")
    if frame_snap is None:
        return

    if event == cv2.EVENT_LBUTTONDOWN:
        # Left click → mark nozzle, extract ROI
        clicked_pt = (x, y)
        frozen_frame = frame_snap.copy()
        roi_crop, bbox = extract_roi(frame_snap, x, y)
        print(f"[CLICK] pixel = ({x}, {y})  →  ROI {ROI_SIZE}×{ROI_SIZE}  bbox={bbox}")

        # Also set as first measurement point
        measure_pts = [(x, y)]

    elif event == cv2.EVENT_RBUTTONDOWN:
        # Right click → second measurement point
        if len(measure_pts) >= 2:
            measure_pts = [measure_pts[0]]  # keep first, replace second
        measure_pts.append((x, y))
        print(f"[MEASURE] point {len(measure_pts)} at pixel = ({x}, {y})")


# ─── Main ─────────────────────────────────────────────────────────────────────

def main():
    global clicked_pt, measure_pts, frozen_frame, roi_crop

    # Camera init
    zed = sl.Camera()
    init_p = sl.InitParameters()
    init_p.camera_resolution = RESOLUTION
    init_p.camera_fps = FPS
    init_p.depth_mode = DEPTH_MODE
    init_p.coordinate_units = sl.UNIT.METER
    status = zed.open(init_p)
    if status != sl.ERROR_CODE.SUCCESS:
        raise RuntimeError(f"Failed to open ZED: {status}")

    # Load extrinsics
    T = parse_extrinsics(EXTRINSICS_PATH)
    if T is None:
        T = T_CAM2GRIPPER_R6
        print(f"[WARN] Extrinsics file not found, using hardcoded R6 matrix")
    else:
        print(f"[INFO] Loaded extrinsics from {EXTRINSICS_PATH}")

    info = zed.get_camera_information()
    w = info.camera_configuration.resolution.width
    h = info.camera_configuration.resolution.height
    print(f"[INFO] Resolution: {w}×{h}")
    print(f"[INFO] ROI size: {ROI_SIZE}×{ROI_SIZE}")
    print(f"[INFO] Controls: L-click=mark nozzle, R-click=2nd point, s=save ROI, r=reset, q=quit")

    image = sl.Mat()
    point_cloud = sl.Mat()
    runtime = sl.RuntimeParameters()

    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
    cb_param = {"frame": None}
    cv2.setMouseCallback(WINDOW_NAME, on_mouse, cb_param)

    save_count = 0

    try:
        while True:
            if zed.grab(runtime) != sl.ERROR_CODE.SUCCESS:
                continue

            zed.retrieve_image(image, sl.VIEW.LEFT)
            zed.retrieve_measure(point_cloud, sl.MEASURE.XYZ)

            frame = image.get_data()
            if frame.shape[2] == 4:
                frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)

            # Update callback param with latest frame
            cb_param["frame"] = frame.copy()

            display = frame.copy()

            # Draw clicked point + ROI box
            if clicked_pt is not None:
                u, v = clicked_pt
                draw_crosshair(display, (u, v))
                _, bbox = extract_roi(frame, u, v)
                draw_roi_box(display, bbox)

                # Show coord text
                cv2.putText(display, f"({u}, {v})", (u + 15, v - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                cv2.putText(display, f"ROI {ROI_SIZE}x{ROI_SIZE}", (bbox[0], bbox[1] - 8),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

            # Draw measurement points and distance
            if len(measure_pts) >= 2:
                pt1, pt2 = measure_pts[0], measure_pts[1]
                draw_crosshair(display, pt1, color=(0, 0, 255))
                draw_crosshair(display, pt2, color=(255, 0, 0))
                cv2.line(display, pt1, pt2, (0, 255, 255), 2)

                p1_cam = robust_xyz(point_cloud, pt1[0], pt1[1], DEPTH_PATCH_RADIUS)
                p2_cam = robust_xyz(point_cloud, pt2[0], pt2[1], DEPTH_PATCH_RADIUS)

                if p1_cam is not None and p2_cam is not None:
                    dist_cam = np.linalg.norm(p1_cam - p2_cam) * 1000.0  # mm

                    # Transform to gripper frame
                    p1_g = (T @ np.append(p1_cam, 1.0))[:3]
                    p2_g = (T @ np.append(p2_cam, 1.0))[:3]
                    dist_g = np.linalg.norm(p1_g - p2_g) * 1000.0

                    mid = ((pt1[0] + pt2[0]) // 2, (pt1[1] + pt2[1]) // 2)
                    cv2.putText(display, f"{dist_cam:.1f} mm (cam)", (mid[0] + 10, mid[1] - 15),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                    cv2.putText(display, f"{dist_g:.1f} mm (gripper)", (mid[0] + 10, mid[1] + 15),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 200, 255), 2)

                    print(f"[DIST] cam={dist_cam:.1f}mm  gripper={dist_g:.1f}mm  "
                          f"p1_cam={p1_cam.round(4).tolist()}  p2_cam={p2_cam.round(4).tolist()}")
                else:
                    cv2.putText(display, "invalid depth", (10, 30),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

            elif len(measure_pts) == 1:
                draw_crosshair(display, measure_pts[0], color=(0, 0, 255))
                cv2.putText(display, "R-click 2nd point to measure", (10, h - 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1)

            # Show ROI preview in corner
            if roi_crop is not None:
                roi_display = cv2.resize(roi_crop, (160, 160))
                display[10:170, w - 170:w - 10] = roi_display
                cv2.rectangle(display, (w - 170, 10), (w - 10, 170), (0, 255, 0), 2)
                cv2.putText(display, f"ROI {ROI_SIZE}x{ROI_SIZE}", (w - 168, 188),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

            cv2.imshow(WINDOW_NAME, display)
            key = cv2.waitKey(1) & 0xFF

            if key == ord('q'):
                break
            elif key == ord('s') and roi_crop is not None:
                save_count += 1
                fname = f"roi_{ROI_SIZE}_{clicked_pt[0]}_{clicked_pt[1]}_{save_count}.png"
                cv2.imwrite(fname, roi_crop)
                print(f"[SAVE] {fname}  shape={roi_crop.shape}")
            elif key == ord('r'):
                clicked_pt = None
                measure_pts = []
                frozen_frame = None
                roi_crop = None
                print("[RESET] Cleared all points")

    finally:
        cv2.destroyAllWindows()
        zed.close()


if __name__ == "__main__":
    main()
