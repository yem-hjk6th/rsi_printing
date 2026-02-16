import cv2
import numpy as np
from skimage.morphology import skeletonize

class WidthMeasurement:
    """Encapsulates geometry, skeleton extraction, and width measurement methods."""
    def __init__(self, fx, config):
        """
        Initialize with camera focal length and configuration.

        Parameters:
            fx (float): Focal length of the ZED camera.
            config (dict): Dictionary containing parameters like control points and ROI offsets.
        """
        self.fx = fx
        self.p1 = (config['p1'][0], config['p1'][1])
        self.p2 = (config['p2'][0], config['p2'][1])

        # frame coordinates including ROI offset
        self.frame_p1 = (config['p1'][0]+config['x0'], config['p1'][1]+config['y0'])
        self.frame_p2 = (config['p2'][0]+config['x0'], config['p2'][1]+config['y0'])
        self.x0 = config['x0']
        self.y0 = config['y0']

    def should_process_width(self, contour_max):
        """Determine if a contour should be used for width measurement (only for video_1)."""
        xs = contour_max[:,0,0]
        ys = contour_max[:,0,1]
        x_min, y_min, x_max, y_max = xs.min(), ys.min(), xs.max(), ys.max()
        width, height = x_max - x_min, y_max - y_min
        vertical = height >= width
        process_width = vertical and width<30
        return process_width


    def create_mask_from_contour(self, mask_shape, contour):
        """Generate a binary mask from a contour."""
        mask = np.zeros(mask_shape, np.uint8)
        cv2.drawContours(mask, [contour], -1, 255, -1)
        return mask

    def extract_centerline(self, mask):
        """Compute the skeleton (centerline) from a binary mask."""
        skeleton = skeletonize(mask > 0)
        ys, xs = np.where(skeleton)
        points = list(zip(xs, ys))
        cv2.imshow("Skeleton", skeleton.astype(np.uint8)*255)
        return skeleton, points

    def trim_centerline(self, points, trim_ratio=0.1):
        """Trim the start and end of the centerline to avoid extreme points."""
        n = len(points)
        start = int(trim_ratio*n)
        end = int((1-trim_ratio)*n)
        return points[start:end]
        

    def compute_local_tangent(self, centerline_points, index, step_sampling):
        """Compute the local tangent vector and perpendicular at a given index of the centerline."""
        window = centerline_points[max(0,index*step_sampling-2): index*step_sampling+3]
        if len(window) < 2:
            return None, None
        dx = np.mean([window[j+1][0]-window[j][0] for j in range(len(window)-1)])
        dy = np.mean([window[j+1][1]-window[j][1] for j in range(len(window)-1)])
        tangent = np.array([dx, dy], float)
        tangent /= np.linalg.norm(tangent)
        perp = np.array([-tangent[1], tangent[0]])
        return tangent, perp

    def traverse_perpendicular(self, mask, x_c, y_c, perp):
        """Traverse along the perpendicular direction from a point until reaching the mask boundary."""
        step = 1.0
        pos1 = np.array([x_c, y_c], float)
        pos2 = pos1.copy()
        while 0 <= int(pos1[1]) < mask.shape[0] and 0 <= int(pos1[0]) < mask.shape[1] and mask[int(pos1[1]), int(pos1[0])]>0:
            pos1 += perp*step
        while 0 <= int(pos2[1]) < mask.shape[0] and 0 <= int(pos2[0]) < mask.shape[1] and mask[int(pos2[1]), int(pos2[0])]>0:
            pos2 -= perp*step
        return pos1, pos2

    def compute_width_mm(self, pos1_global, pos2_global, center_global, depth):
        """Convert pixel distance to millimeters using depth at the center point."""
        width_px = np.linalg.norm(pos1_global - pos2_global)
        mean_depth = depth[int(center_global[1]), int(center_global[0])]
        return width_px * mean_depth / self.fx

    def width_control_point(self, mask, skeleton):
        """Find the intersection point between a horizontal line and the skeleton as a control point."""
        skeleton = skeleton.astype(np.uint8)
        line_mask = np.zeros_like(mask, np.uint8)
        cv2.line(line_mask, (0, 75), (mask.shape[1]-1, 75), 255, 1)

        # Compute intersection
        intersection_mask = cv2.bitwise_and(skeleton, line_mask)

        # Get intersection coordinates
        ys, xs = np.where(intersection_mask > 0)
        intersections = list(zip(xs, ys))

        if intersections:
            return intersections[0]
        else: 
            return []

    def measure_perpendicular_widths(self, image, mask, contour_max, centerline_points, depth):
        """
        Measure widths along multiple perpendicular lines along the centerline.
        Returns a list of width values in millimeters.
        """
        widths_mm = []
        step_sampling = max(1, len(centerline_points) // 10)
        font = cv2.FONT_HERSHEY_SIMPLEX
        contour_global = contour_max + np.array([self.x0, self.y0])

        for i, (x_c, y_c) in enumerate(centerline_points[::step_sampling]):
            tangent, perp = self.compute_local_tangent(centerline_points, i, step_sampling)
            if tangent is None:
                continue

            pos1, pos2 = self.traverse_perpendicular(mask, x_c, y_c, perp)

            # Convert to global coordinates
            pos1_global = pos1 + np.array([self.x0, self.y0])
            pos2_global = pos2 + np.array([self.x0, self.y0])
            center_global = np.array([x_c + self.x0, y_c + self.y0])

            # Compute width
            width_mm = self.compute_width_mm(pos1_global, pos2_global, center_global, depth)
            widths_mm.append(width_mm)

            # Draw segment 
            self.draw_segment(image, pos1_global, pos2_global, center_global, width_mm)
        
        cv2.drawContours(image, [contour_global], 0, (0, 0, 255), 2)
        return widths_mm

    def measure_width_at_control_point(self, image, mask, skeleton, depth, window_radius=3):
        """
        Measure width at a specific control point.
        Uses local tangent, perpendicular traversal and drawing functions.
        """
        # Get control point in the skeleton 
        point = self.width_control_point(mask, skeleton)
        if not point:
            return -1.0
        x_c, y_c = point

        # Build a small local neighborhood on skeleton for tangent computation 
        ys, xs = np.where(skeleton > 0)
        distances = np.sqrt((xs - x_c)**2 + (ys - y_c)**2)
        nearest_idx = np.argsort(distances)[:window_radius*2 + 1]
        window_points = np.column_stack((xs[nearest_idx], ys[nearest_idx]))
        if len(window_points) < 2:
            raise ValueError("Not enough points to compute tangent")

        dx = window_points[-1, 0] - window_points[0, 0]
        dy = window_points[-1, 1] - window_points[0, 1]
        tangent = np.array([dx, dy], dtype=float)
        tangent /= np.linalg.norm(tangent)
        perp = np.array([-tangent[1], tangent[0]])

        # Traverse perpendicular
        pos1, pos2 = self.traverse_perpendicular(mask, x_c, y_c, perp)

        # Convert to global coordinates
        pos1_global = pos1 + np.array([self.x0, self.y0])
        pos2_global = pos2 + np.array([self.x0, self.y0])
        center_global = np.array([x_c + self.x0, y_c + self.y0])

        # Compute width in control point
        width_mm = self.compute_width_mm(pos1_global, pos2_global, center_global, depth)

        # Draw result 
        self.draw_control_segment(image, pos1_global, pos2_global, center_global, width_mm)
        return width_mm

    def draw_segment(self, image, pos1_global, pos2_global, center_global, width_mm):
        cv2.line(image, tuple(pos1_global.astype(int)), tuple(pos2_global.astype(int)), (0,255,0), 2)
        cv2.putText(image, f"{width_mm:.2f} mm", tuple(center_global.astype(int)+[20,0]), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,0,0), 1)

    def draw_control_segment(self, image, pos1_global, pos2_global, center_global, width_mm):
        cv2.line(image, tuple(pos1_global.astype(int)), tuple(pos2_global.astype(int)), (0,0,0), 2)
        cv2.putText(image, f"{width_mm:.2f} mm", tuple(center_global.astype(int)+[100,-5]), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,255), 2)

    def draw_measurement_line(self, img):
            num_points = 20
            for i in np.linspace(0, 1, num_points):
                x = int(self.frame_p1[0] + i * (self.frame_p2[0] - self.frame_p1[0]))
                y = int(self.frame_p1[1] + i * (self.frame_p2[1] - self.frame_p1[1]))
                cv2.circle(img, (x, y), 2, (0,255,255), -1)

