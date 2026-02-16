import argparse
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np
import pyzed.sl as sl

# === Edit these defaults for quick use ===
DEFAULT_SVO_PATH = "recorded_data/20260203_154812/recording_20260203_154812.svo2"
DEFAULT_OUT_DIR = str(Path(__file__).resolve().parent / "clip")
DEFAULT_DEPTH_START_FRAME = 7506
DEFAULT_DEPTH_END_FRAME = 13743
DEFAULT_FRAME_BASE = 0  # 0 for 0-based frame index, 1 if using ZED Studio 1-based numbers
DEFAULT_DEPTH_VIS_VIDEO = True  # save colorized depth preview video


def parse_args():
    parser = argparse.ArgumentParser(
        description=(
            "Extract depth frames from a ZED SVO file over specified frame ranges."
        )
    )
    parser.add_argument("--svo", default=DEFAULT_SVO_PATH, help="Path to the SVO file")
    parser.add_argument(
        "--out-dir",
        default=DEFAULT_OUT_DIR,
        help="Output base directory (timestamped subfolders/files will be created)",
    )
    parser.add_argument(
        "--depth-start",
        type=int,
        default=DEFAULT_DEPTH_START_FRAME,
        help="Depth start frame index",
    )
    parser.add_argument(
        "--depth-end",
        type=int,
        default=DEFAULT_DEPTH_END_FRAME,
        help="Depth end frame index",
    )
    parser.add_argument(
        "--frame-base",
        type=int,
        choices=[0, 1],
        default=DEFAULT_FRAME_BASE,
        help="Frame number base: 0 for 0-based, 1 for 1-based (ZED Studio)",
    )
    parser.add_argument(
        "--depth-vis-video",
        action="store_true",
        default=DEFAULT_DEPTH_VIS_VIDEO,
        help="Also save colorized depth preview video",
    )
    return parser.parse_args()


def validate_range(start, end, label):
    if start < 0 or end < 0:
        raise ValueError(f"{label} range must be non-negative")
    if end <= start:
        raise ValueError(f"{label} end time must be greater than start time")


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


def main():
    args = parse_args()
    print("=== Reminder: key parameters ===")
    print("- SVO path: update DEFAULT_SVO_PATH or pass --svo")
    print("- Frame ranges: --depth-start/--depth-end")
    print("- Frame base: --frame-base 0 or 1 (ZED Studio is usually 1-based)")
    print("- Depth preview video: --depth-vis-video")
    print("- Output: --out-dir (files named by current time)")
    print("===============================")

    depth_start = args.depth_start - args.frame_base
    depth_end = args.depth_end - args.frame_base
    validate_range(depth_start, depth_end, "Depth")

    svo_path = Path(args.svo).resolve()
    if not svo_path.exists():
        raise FileNotFoundError(f"SVO not found: {svo_path}")
    print(f"SVO path: {svo_path}")
    print(f"SVO size (MB): {svo_path.stat().st_size / (1024 * 1024):.1f}")

    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    time_tag = datetime.now().strftime("%Y%m%d_%H%M%S")
    zed, runtime_params = open_svo(str(svo_path))
    depth = sl.Mat()
    info = zed.get_camera_information().camera_configuration
    width = info.resolution.width
    height = info.resolution.height
    fps = float(info.fps) if info.fps > 0 else 30.0
    depth_vis_writer = None
    if args.depth_vis_video:
        depth_vis_path = out_dir / f"depth_vis_{time_tag}.mp4"
        fourcc = cv2.VideoWriter_fourcc(*"mp4v")
        depth_vis_writer = cv2.VideoWriter(str(depth_vis_path), fourcc, fps, (width, height))
        if not depth_vis_writer.isOpened():
            raise RuntimeError("Failed to open depth preview video writer.")
    total_frames = zed.get_svo_number_of_frames()
    print(f"SVO total frames: {total_frames}")
    if total_frames <= 0:
        raise RuntimeError("Failed to read total frame count from SVO.")

    min_start = depth_start
    max_end = depth_end

    if min_start >= total_frames:
        raise ValueError(
            "Start frame is beyond total frames. "
            f"min_start={min_start}, total_frames={total_frames}"
        )
    if max_end >= total_frames:
        print(
            "Warning: end frame exceeds total frames. "
            f"Clipping to {total_frames - 1}."
        )
        max_end = total_frames - 1

    processed_count = 0
    total_to_process = max_end - min_start + 1
    next_pct = 10

    zed.set_svo_position(min_start)

    while zed.grab(runtime_params) == sl.ERROR_CODE.SUCCESS:
        frame_idx = zed.get_svo_position()
        if frame_idx > max_end:
            break

        if depth_start <= frame_idx <= depth_end:
            zed.retrieve_measure(depth, sl.MEASURE.DEPTH)
            depth_map = depth.get_data().copy()
            depth_map[~np.isfinite(depth_map)] = 0
            depth_vis = cv2.normalize(depth_map, None, 0, 255, cv2.NORM_MINMAX)
            depth_vis = depth_vis.astype(np.uint8)
            depth_vis = cv2.applyColorMap(depth_vis, cv2.COLORMAP_JET)
            if depth_vis_writer is not None:
                depth_vis_writer.write(depth_vis)

        processed_count += 1
        progress_pct = int(processed_count * 100 / total_to_process)
        if progress_pct >= next_pct:
            print(f"Progress: {progress_pct}%")
            next_pct += 10

    if depth_vis_writer is not None:
        depth_vis_writer.release()
    zed.close()

    print("Done.")
    print("Depth preview video written." if depth_vis_writer is not None else "No video output.")


if __name__ == "__main__":
    main()
