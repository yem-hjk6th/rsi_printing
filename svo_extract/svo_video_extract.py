import argparse
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np
import pyzed.sl as sl

# === Edit these defaults for quick use ===
DEFAULT_SVO_PATH = "recorded_data/20260203_154812/recording_20260203_154812.svo2"
DEFAULT_OUT_DIR = str(Path(__file__).resolve().parent / "clip")
DEFAULT_RGB_START_FRAME = 7506
DEFAULT_RGB_END_FRAME = 13743
DEFAULT_FRAME_BASE = 0  # 0 for 0-based frame index, 1 if using ZED Studio 1-based numbers


def parse_args():
	parser = argparse.ArgumentParser(
		description=(
			"Extract RGB video and depth frames from a ZED SVO file over specified frame ranges."
		)
	)
	parser.add_argument("--svo", default=DEFAULT_SVO_PATH, help="Path to the SVO file")
	parser.add_argument(
		"--out-dir",
		default=DEFAULT_OUT_DIR,
		help="Output base directory (timestamped subfolders/files will be created)",
	)
	parser.add_argument(
		"--rgb-start",
		type=int,
		default=DEFAULT_RGB_START_FRAME,
		help="RGB start frame index",
	)
	parser.add_argument(
		"--rgb-end",
		type=int,
		default=DEFAULT_RGB_END_FRAME,
		help="RGB end frame index",
	)
	parser.add_argument(
		"--frame-base",
		type=int,
		choices=[0, 1],
		default=DEFAULT_FRAME_BASE,
		help="Frame number base: 0 for 0-based, 1 for 1-based (ZED Studio)",
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


def build_writer(rgb_out, width, height, fps):
	fourcc = cv2.VideoWriter_fourcc(*"mp4v")
	writer = cv2.VideoWriter(rgb_out, fourcc, fps, (width, height))
	if not writer.isOpened():
		raise RuntimeError("Failed to open RGB video writer.")
	return writer


def main():
	args = parse_args()
	print("=== Reminder: key parameters ===")
	print("- SVO path: update DEFAULT_SVO_PATH or pass --svo")
	print("- Frame ranges: --rgb-start/--rgb-end")
	print("- Frame base: --frame-base 0 or 1 (ZED Studio is usually 1-based)")
	print("- Output: --out-dir (files named by current time)")
	print("===============================")
	rgb_start = args.rgb_start - args.frame_base
	rgb_end = args.rgb_end - args.frame_base
	validate_range(rgb_start, rgb_end, "RGB")

	svo_path = Path(args.svo).resolve()
	if not svo_path.exists():
		raise FileNotFoundError(f"SVO not found: {svo_path}")
	print(f"SVO path: {svo_path}")
	print(f"SVO size (MB): {svo_path.stat().st_size / (1024 * 1024):.1f}")

	out_dir = Path(args.out_dir)
	out_dir.mkdir(parents=True, exist_ok=True)

	time_tag = datetime.now().strftime("%Y%m%d_%H%M%S")
	rgb_out_path = out_dir / f"rgb_{time_tag}.mp4"
	zed, runtime_params = open_svo(str(svo_path))
	image = sl.Mat()
	total_frames = zed.get_svo_number_of_frames()
	print(f"SVO total frames: {total_frames}")
	if total_frames <= 0:
		raise RuntimeError("Failed to read total frame count from SVO.")

	min_start = rgb_start
	max_end = rgb_end

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

	info = zed.get_camera_information().camera_configuration
	width = info.resolution.width
	height = info.resolution.height
	fps = float(info.fps) if info.fps > 0 else 30.0
	rgb_writer = build_writer(str(rgb_out_path), width, height, fps)

	rgb_count = 0
	processed_count = 0
	total_to_process = max_end - min_start + 1
	next_pct = 10

	zed.set_svo_position(min_start)

	while zed.grab(runtime_params) == sl.ERROR_CODE.SUCCESS:
		frame_idx = zed.get_svo_position()
		if frame_idx > max_end:
			break

		if rgb_start <= frame_idx <= rgb_end:
			zed.retrieve_image(image, sl.VIEW.LEFT)
			frame = image.get_data()[:, :, :3]
			frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGBA2BGR)
			rgb_writer.write(frame_bgr)
			rgb_count += 1

		processed_count += 1
		progress_pct = int(processed_count * 100 / total_to_process)
		if progress_pct >= next_pct:
			print(f"Progress: {progress_pct}%")
			next_pct += 10

	rgb_writer.release()
	zed.close()

	print("Done.")
	print(f"RGB frames written: {rgb_count}")


if __name__ == "__main__":
	main()
