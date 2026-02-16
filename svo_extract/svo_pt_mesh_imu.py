import argparse
from datetime import datetime
from pathlib import Path

import numpy as np
import open3d as o3d
import pyzed.sl as sl

# === Edit these defaults for quick use ===
DEFAULT_SVO_PATH = "recorded_data/20260203_154812/recording_20260203_154812.svo2"
DEFAULT_OUT_DIR = str(Path(__file__).resolve().parent / "clip")
DEFAULT_START_FRAME = 7506
DEFAULT_END_FRAME = 13743
DEFAULT_FRAME_BASE = 0  # 0 for 0-based frame index, 1 if using ZED Studio 1-based numbers
DEFAULT_TARGET_FRAMES = 180  # ~6s at 30 fps
DEFAULT_POINT_STRIDE = 4  # sample every N points to reduce density
DEFAULT_VOXEL_SIZE_MM = 5.0  # millimeters
DEFAULT_POISSON_DEPTH = 8
DEFAULT_SAVE_FRAMES = False
DEFAULT_SAVE_FUSED = True
DEFAULT_VISUALIZE = True
DEFAULT_VIS_UPDATE_EVERY = 10
DEFAULT_ENABLE_TRACKING = True
DEFAULT_SHOW_CURRENT_FRAME = True
DEFAULT_PRINT_TRACKING = True


def parse_args():
    parser = argparse.ArgumentParser(
        description="Offline mesh reconstruction from sampled SVO frames with tracking."
    )
    parser.add_argument("--svo", default=DEFAULT_SVO_PATH, help="Path to the SVO file")
    parser.add_argument(
        "--out-dir",
        default=DEFAULT_OUT_DIR,
        help="Output base directory (timestamped files will be created)",
    )
    parser.add_argument(
        "--start",
        type=int,
        default=DEFAULT_START_FRAME,
        help="Start frame index",
    )
    parser.add_argument(
        "--end",
        type=int,
        default=DEFAULT_END_FRAME,
        help="End frame index",
    )
    parser.add_argument(
        "--frame-base",
        type=int,
        choices=[0, 1],
        default=DEFAULT_FRAME_BASE,
        help="Frame number base: 0 for 0-based, 1 for 1-based (ZED Studio)",
    )
    parser.add_argument(
        "--frames",
        type=int,
        default=DEFAULT_TARGET_FRAMES,
        help="Number of frames to sample for reconstruction",
    )
    parser.add_argument(
        "--stride",
        type=int,
        default=DEFAULT_POINT_STRIDE,
        help="Point sampling stride (larger = fewer points)",
    )
    parser.add_argument(
        "--voxel",
        type=float,
        default=DEFAULT_VOXEL_SIZE_MM,
        help="Voxel size for downsampling (millimeters)",
    )
    parser.add_argument(
        "--poisson-depth",
        type=int,
        default=DEFAULT_POISSON_DEPTH,
        help="Poisson reconstruction depth",
    )
    parser.add_argument(
        "--save-frames",
        action="store_true",
        default=DEFAULT_SAVE_FRAMES,
        help="Save each sampled frame point cloud as PLY",
    )
    parser.add_argument(
        "--save-fused",
        action="store_true",
        default=DEFAULT_SAVE_FUSED,
        help="Save fused point cloud as PLY",
    )
    parser.add_argument(
        "--visualize",
        action="store_true",
        default=DEFAULT_VISUALIZE,
        help="Show live point cloud preview",
    )
    parser.add_argument(
        "--vis-update-every",
        type=int,
        default=DEFAULT_VIS_UPDATE_EVERY,
        help="Visualization update interval (frames)",
    )
    parser.add_argument(
        "--enable-tracking",
        action="store_true",
        default=DEFAULT_ENABLE_TRACKING,
        help="Enable ZED positional tracking for pose-based fusion",
    )
    parser.add_argument(
        "--show-current",
        action="store_true",
        default=DEFAULT_SHOW_CURRENT_FRAME,
        help="Preview current frame point cloud instead of fused cloud",
    )
    parser.add_argument(
        "--print-tracking",
        action="store_true",
        default=DEFAULT_PRINT_TRACKING,
        help="Print positional tracking state",
    )
    return parser.parse_args()


def validate_range(start, end):
    if start < 0 or end < 0:
        raise ValueError("Frame range must be non-negative")
    if end <= start:
        raise ValueError("End frame must be greater than start frame")


def open_svo(svo_path):
    zed = sl.Camera()
    init_params = sl.InitParameters()
    init_params.svo_real_time_mode = False
    init_params.depth_mode = sl.DEPTH_MODE.PERFORMANCE
    init_params.coordinate_units = sl.UNIT.MILLIMETER
    input_type = sl.InputType()
    input_type.set_from_svo_file(svo_path)
    init_params.input = input_type

    if zed.open(init_params) != sl.ERROR_CODE.SUCCESS:
        raise RuntimeError("Cannot open SVO file.")

    runtime_params = sl.RuntimeParameters()
    return zed, runtime_params


def get_pose_matrix(pose):
    transform = pose.pose_data()
    mat = np.array(transform.m, dtype=np.float64)
    return mat


def main():
    args = parse_args()
    start = args.start - args.frame_base
    end = args.end - args.frame_base
    validate_range(start, end)

    svo_path = Path(args.svo).resolve()
    if not svo_path.exists():
        raise FileNotFoundError(f"SVO not found: {svo_path}")

    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    time_tag = datetime.now().strftime("%Y%m%d_%H%M%S")
    mesh_path = out_dir / f"mesh_{time_tag}.ply"
    fused_path = out_dir / f"fused_{time_tag}.ply"
    frames_dir = out_dir / f"frames_{time_tag}"
    if args.save_frames:
        frames_dir.mkdir(parents=True, exist_ok=True)

    zed, runtime_params = open_svo(str(svo_path))
    point_cloud = sl.Mat()

    if args.enable_tracking:
        tracking_params = sl.PositionalTrackingParameters()
        tracking_params.set_floor_as_origin = False
        if zed.enable_positional_tracking(tracking_params) != sl.ERROR_CODE.SUCCESS:
            print("Warning: positional tracking not available. Using raw point clouds.")
            args.enable_tracking = False

    pose = sl.Pose()

    total_frames = zed.get_svo_number_of_frames()
    if total_frames <= 0:
        raise RuntimeError("Failed to read total frame count from SVO.")
    if start >= total_frames:
        raise ValueError(f"Start frame beyond total frames: {start} >= {total_frames}")
    if end >= total_frames:
        end = total_frames - 1

    sample_count = min(args.frames, end - start + 1)
    frame_indices = np.linspace(start, end, sample_count)
    frame_indices = np.unique(frame_indices.astype(int))

    pcd_all = o3d.geometry.PointCloud()
    pcd_preview = o3d.geometry.PointCloud()
    has_preview = False
    vis = None
    if args.visualize:
        vis = o3d.visualization.Visualizer()
        vis.create_window(window_name="SVO Point Cloud Preview", width=1280, height=720)
        vis.add_geometry(pcd_preview if args.show_current else pcd_all)

    for idx, frame_idx in enumerate(frame_indices, start=1):
        zed.set_svo_position(int(frame_idx))
        if zed.grab(runtime_params) != sl.ERROR_CODE.SUCCESS:
            continue

        zed.retrieve_measure(point_cloud, sl.MEASURE.XYZRGBA)
        pc_data = point_cloud.get_data()
        points = pc_data.reshape(-1, 4)
        points = points[:: max(1, args.stride)]

        xyz = points[:, :3]
        valid = np.isfinite(xyz).all(axis=1)
        xyz = xyz[valid]
        if xyz.size == 0:
            continue

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(xyz)
        pcd = pcd.voxel_down_sample(args.voxel)

        tracking_state = None
        if args.enable_tracking:
            tracking_state = zed.get_position(pose, sl.REFERENCE_FRAME.WORLD)
            if tracking_state == sl.POSITIONAL_TRACKING_STATE.OK:
                try:
                    pose_mat = get_pose_matrix(pose)
                    pcd.transform(pose_mat)
                except Exception:
                    pass

        if args.print_tracking and tracking_state is not None and idx % max(1, len(frame_indices) // 10) == 0:
            print(f"Tracking state: {tracking_state}")

        pcd_all += pcd

        if args.save_frames:
            frame_path = frames_dir / f"frame_{int(frame_idx):06d}.ply"
            o3d.io.write_point_cloud(str(frame_path), pcd, write_ascii=False)

        if vis is not None and idx % max(1, args.vis_update_every) == 0:
            if args.show_current:
                pcd_preview.points = pcd.points
                vis.update_geometry(pcd_preview)
                if not has_preview:
                    vis.reset_view_point(True)
                    has_preview = True
            else:
                vis.update_geometry(pcd_all)
                if not has_preview:
                    vis.reset_view_point(True)
                    has_preview = True
            vis.poll_events()
            vis.update_renderer()

        if idx % max(1, len(frame_indices) // 10) == 0:
            print(f"Progress: {int(idx * 100 / len(frame_indices))}%")

    if vis is not None:
        vis.destroy_window()
    zed.close()

    if len(pcd_all.points) == 0:
        raise RuntimeError("No valid points collected for reconstruction.")

    pcd_all = pcd_all.voxel_down_sample(args.voxel)
    pcd_all.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.02, max_nn=30)
    )

    if args.save_fused:
        o3d.io.write_point_cloud(str(fused_path), pcd_all, write_ascii=False)

    mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
        pcd_all, depth=args.poisson_depth
    )

    densities = np.asarray(densities)
    density_threshold = np.quantile(densities, 0.02)
    vertices_to_remove = densities < density_threshold
    mesh.remove_vertices_by_mask(vertices_to_remove)

    o3d.io.write_triangle_mesh(str(mesh_path), mesh)
    print(f"Mesh saved: {mesh_path}")


if __name__ == "__main__":
    main()
