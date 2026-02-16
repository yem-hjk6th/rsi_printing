import os
import time
import argparse
from datetime import datetime

import cv2
import numpy as np
import pyzed.sl as sl

# === Header Config ===
# Default input dir (auto-scan .svo/.svo2)
DEFAULT_SVO_DIR = os.path.join("recorded_data", "res_test")
# Default output dir (auto-generate if None)
DEFAULT_OUT_DIR = None
# Depth visualization range (mm)
DEFAULT_MIN_DEPTH = 400
DEFAULT_MAX_DEPTH = 30000
# Save raw depth .npy
DEFAULT_SAVE_DEPTH_NPY = False
# Save extracted frames
DEFAULT_SAVE_FRAMES = False


def ensure_dir(path: str) -> None:
    os.makedirs(path, exist_ok=True)


def normalize_depth_to_8bit(depth_mm: np.ndarray, min_mm=300, max_mm=10000) -> np.ndarray:
    depth_clipped = np.clip(depth_mm, min_mm, max_mm)
    depth_norm = (depth_clipped - min_mm) / (max_mm - min_mm)
    depth_8u = (depth_norm * 255).astype(np.uint8)
    return depth_8u


def compute_depth_metrics(depth_mm: np.ndarray) -> dict:
    valid = np.isfinite(depth_mm) & (depth_mm > 0)
    valid_ratio = float(np.count_nonzero(valid)) / depth_mm.size if depth_mm.size else 0.0

    if np.count_nonzero(valid) == 0:
        return {
            "valid_ratio": valid_ratio,
            "mean_mm": None,
            "median_mm": None,
            "std_mm": None,
            "p10_mm": None,
            "p90_mm": None,
            "min_mm": None,
            "max_mm": None,
        }

    vals = depth_mm[valid]
    return {
        "valid_ratio": valid_ratio,
        "mean_mm": float(np.mean(vals)),
        "median_mm": float(np.median(vals)),
        "std_mm": float(np.std(vals)),
        "p10_mm": float(np.percentile(vals, 10)),
        "p90_mm": float(np.percentile(vals, 90)),
        "min_mm": float(np.min(vals)),
        "max_mm": float(np.max(vals)),
    }


def main():
    parser = argparse.ArgumentParser(description="Split SVO2 into RGB/depth frames and report metrics.")
    parser.add_argument("--svo-dir", default=DEFAULT_SVO_DIR, help="Folder containing .svo/.svo2 files")
    parser.add_argument("--out", default=DEFAULT_OUT_DIR, help="Output directory for extracted frames")
    parser.add_argument("--save-depth-npy", action="store_true", help="Save raw depth as .npy")
    parser.add_argument("--save-frames", action="store_true", help="Save extracted RGB/depth frames")
    parser.add_argument("--min-depth", type=float, default=DEFAULT_MIN_DEPTH, help="Min depth for visualization (mm)")
    parser.add_argument("--max-depth", type=float, default=DEFAULT_MAX_DEPTH, help="Max depth for visualization (mm)")
    args = parser.parse_args()

    if args.out is None:
        args.out = DEFAULT_OUT_DIR
    if args.save_depth_npy is False:
        args.save_depth_npy = DEFAULT_SAVE_DEPTH_NPY
    if args.save_frames is False:
        args.save_frames = DEFAULT_SAVE_FRAMES

    if not os.path.isdir(args.svo_dir):
        print(f"SVO directory not found: {args.svo_dir}")
        return

    svo_files = [
        os.path.join(args.svo_dir, f)
        for f in os.listdir(args.svo_dir)
        if f.lower().endswith((".svo", ".svo2"))
    ]
    if not svo_files:
        print(f"No .svo/.svo2 files found in: {args.svo_dir}")
        return

    summary_rows = []

    for svo_path in sorted(svo_files):
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        if args.out is None:
            out_base = os.path.join("recorded_data", "res_test", f"svo2_extract_{timestamp}")
        else:
            out_base = args.out
        svo_name = os.path.splitext(os.path.basename(svo_path))[0]
        out_dir = os.path.join(out_base, svo_name)
        if args.save_frames or args.save_depth_npy:
            ensure_dir(out_dir)

        rgb_dir = os.path.join(out_dir, "rgb")
        depth_dir = os.path.join(out_dir, "depth")
        depth_raw_dir = os.path.join(out_dir, "depth_raw")
        if args.save_frames:
            ensure_dir(rgb_dir)
            ensure_dir(depth_dir)
        if args.save_depth_npy:
            ensure_dir(depth_raw_dir)

        init_params = sl.InitParameters()
        init_params.set_from_svo_file(svo_path)
        init_params.svo_real_time_mode = False
        init_params.depth_mode = sl.DEPTH_MODE.ULTRA
        init_params.coordinate_units = sl.UNIT.MILLIMETER

        zed = sl.Camera()
        if zed.open(init_params) != sl.ERROR_CODE.SUCCESS:
            print(f"Failed to open SVO file: {svo_path}")
            continue

        runtime_params = sl.RuntimeParameters()
        image = sl.Mat()
        depth = sl.Mat()
        depth_image = sl.Mat() if args.save_frames else None

        frame_count = 0
        metrics_accum = []
        start = time.time()

        while True:
            if zed.grab(runtime_params) != sl.ERROR_CODE.SUCCESS:
                break

            zed.retrieve_image(image, sl.VIEW.LEFT)
            zed.retrieve_measure(depth, sl.MEASURE.DEPTH)
            if args.save_frames:
                zed.retrieve_image(depth_image, sl.VIEW.DEPTH, sl.MEM.CPU)

            rgb = image.get_data()
            depth_mm = depth.get_data()
            depth_vis = depth_image.get_data() if args.save_frames else None

            h, w = rgb.shape[:2]
            pixel_total = w * h

            if args.save_frames:
                rgb_path = os.path.join(rgb_dir, f"frame_{frame_count:06d}.png")
                cv2.imwrite(rgb_path, rgb)

            if args.save_frames:
                if depth_vis is None or depth_vis.size == 0:
                    depth_vis = normalize_depth_to_8bit(depth_mm, args.min_depth, args.max_depth)
                depth_vis_path = os.path.join(depth_dir, f"depth_{frame_count:06d}.png")
                cv2.imwrite(depth_vis_path, depth_vis)

            if args.save_depth_npy:
                depth_raw_path = os.path.join(depth_raw_dir, f"depth_{frame_count:06d}.npy")
                np.save(depth_raw_path, depth_mm)

            metrics = compute_depth_metrics(depth_mm)
            metrics_accum.append(metrics)

            frame_count += 1

        zed.close()

        if frame_count == 0:
            print(f"No frames extracted from: {svo_path}")
            continue

        valid_ratios = [m["valid_ratio"] for m in metrics_accum]
        means = [m["mean_mm"] for m in metrics_accum if m["mean_mm"] is not None]
        medians = [m["median_mm"] for m in metrics_accum if m["median_mm"] is not None]
        stds = [m["std_mm"] for m in metrics_accum if m["std_mm"] is not None]

        summary = {
            "svo_file": os.path.basename(svo_path),
            "frames": frame_count,
            "resolution": f"{w}x{h}",
            "pixel_total": pixel_total,
            "avg_valid_ratio": float(np.mean(valid_ratios)) if valid_ratios else 0.0,
            "avg_mean_mm": float(np.mean(means)) if means else None,
            "avg_median_mm": float(np.mean(medians)) if medians else None,
            "avg_std_mm": float(np.mean(stds)) if stds else None,
            "elapsed_s": round(time.time() - start, 2),
        }
        summary_rows.append(summary)

        if args.save_frames or args.save_depth_npy:
            summary_path = os.path.join(out_dir, "summary.txt")
            with open(summary_path, "w", encoding="utf-8") as f:
                for k, v in summary.items():
                    f.write(f"{k}: {v}\n")

        print("Summary:")
        for k, v in summary.items():
            print(f"{k}: {v}")
        if args.save_frames or args.save_depth_npy:
            print(f"Saved summary to: {summary_path}")

    if summary_rows:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        if args.out is None:
            out_base = os.path.join("recorded_data", "res_test", f"svo2_extract_{timestamp}")
        else:
            out_base = args.out
        ensure_dir(out_base)
        summary_csv = os.path.join(out_base, "summary_all.csv")
        header = [
            "svo_file",
            "frames",
            "resolution",
            "pixel_total",
            "avg_valid_ratio",
            "avg_mean_mm",
            "avg_median_mm",
            "avg_std_mm",
            "elapsed_s",
        ]
        with open(summary_csv, "w", encoding="utf-8") as f:
            f.write(",".join(header) + "\n")
            for row in summary_rows:
                f.write(",".join(str(row.get(k, "")) for k in header) + "\n")
        print(f"Saved summary CSV to: {summary_csv}")


if __name__ == "__main__":
    main()
