"""
Depth-based layer detection from SVO video.

Uses depth map from ZED camera to identify printed region and detect layers
by tracking height growth over time.

Applies to exp08_cube_5.src: 10 layers, layer height 1.5mm, 50×50mm square.
"""

import os
import sys
from pathlib import Path

os.add_dll_directory(r"C:\Program Files (x86)\ZED SDK\bin")

import cv2
import numpy as np
import pyzed.sl as sl

try:
    import matplotlib
    for _font in ["Microsoft YaHei", "SimHei", "SimSun"]:
        try:
            matplotlib.rc("font", family=_font)
            break
        except Exception:
            continue
    matplotlib.rcParams["axes.unicode_minus"] = False
    import matplotlib.pyplot as plt
    HAS_MPL = True
except ImportError:
    HAS_MPL = False

# ─── Config ────────────────────────────────────────────────
SVO_PATH = "recorded_data/20260324_174844/recording_20260324_174844.svo2"
RSI_PATH = "rsi_data/rsi_data_20260324_174844.csv"
OUT_DIR  = "svo_extract/layer_depth_detect"
REF_FRAME = 10          # reference frame (before printing)
SAMPLE_STEP = 10        # sample every N frames
LAYER_HEIGHT_MM = 1.5   # from exp08_cube_5.src
N_LAYERS = 10           # from exp08_cube_5.src
DEPTH_GROW_THRESHOLD = 3.0  # mm, min depth decrease to count as "grown"


def load_rsi_csv(filepath):
    """Load RSI CSV to get print timing."""
    import csv
    with open(filepath, newline="") as f:
        rows = list(csv.DictReader(f))
    data = {
        "timestamp": np.array([float(r["timestamp"]) for r in rows]),
        "z_mm": np.array([float(r["z_mm"]) for r in rows]),
    }
    return data


def open_svo(svo_path):
    zed = sl.Camera()
    init = sl.InitParameters()
    init.svo_real_time_mode = False
    init.depth_mode = sl.DEPTH_MODE.NEURAL
    init.coordinate_units = sl.UNIT.MILLIMETER
    inp = sl.InputType()
    inp.set_from_svo_file(svo_path)
    init.input = inp
    err = zed.open(init)
    if err != sl.ERROR_CODE.SUCCESS:
        raise RuntimeError(f"Cannot open SVO: {err}")
    return zed


def find_print_roi(zed, ref_frame, late_frame):
    """
    Find the ROI where the printed object appears by comparing
    a reference frame (no print) to a late frame (print complete).
    Returns (row_min, row_max, col_min, col_max) of the printing area.
    """
    depth_mat = sl.Mat()
    runtime = sl.RuntimeParameters()

    # Reference depth
    zed.set_svo_position(ref_frame)
    zed.grab(runtime)
    zed.retrieve_measure(depth_mat, sl.MEASURE.DEPTH)
    ref = depth_mat.get_data().copy()

    # Late frame depth
    zed.set_svo_position(late_frame)
    zed.grab(runtime)
    zed.retrieve_measure(depth_mat, sl.MEASURE.DEPTH)
    late = depth_mat.get_data().copy()

    # Depth difference: positive = object grew closer (printed)
    diff = ref - late
    valid = np.isfinite(diff)
    diff[~valid] = 0

    # The printed object should show as a cluster of pixels with diff > threshold
    mask = diff > DEPTH_GROW_THRESHOLD
    # Morphological cleanup
    kernel = np.ones((7, 7), np.uint8)
    mask_u8 = mask.astype(np.uint8) * 255
    mask_u8 = cv2.morphologyEx(mask_u8, cv2.MORPH_CLOSE, kernel, iterations=2)
    mask_u8 = cv2.morphologyEx(mask_u8, cv2.MORPH_OPEN, kernel, iterations=1)

    # Find contours and get the largest one (the printed object)
    contours, _ = cv2.findContours(mask_u8, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        print("Warning: no printed region detected, using full frame center")
        h, w = ref.shape
        return h // 3, h * 2 // 3, w // 3, w * 2 // 3, ref

    # Sort by area, pick largest
    contours = sorted(contours, key=cv2.contourArea, reverse=True)
    biggest = contours[0]
    x, y, bw, bh = cv2.boundingRect(biggest)

    # Add margin
    margin = 20
    h, w = ref.shape
    r0 = max(0, y - margin)
    r1 = min(h, y + bh + margin)
    c0 = max(0, x - margin)
    c1 = min(w, x + bw + margin)

    area = cv2.contourArea(biggest)
    print(f"Detected print region: rows [{r0}:{r1}], cols [{c0}:{c1}], "
          f"area={area:.0f}px, bbox={bw}x{bh}px")

    return r0, r1, c0, c1, ref


def scan_depth_over_time(zed, ref_depth, roi, n_frames, sample_step):
    """
    Scan through SVO and measure depth change in the ROI over time.
    Returns arrays of (frame_idx, time_sec, mean_height_mm, max_height_mm, n_grown_pixels).
    """
    r0, r1, c0, c1 = roi
    ref_roi = ref_depth[r0:r1, c0:c1]

    depth_mat = sl.Mat()
    runtime = sl.RuntimeParameters()

    frames = []
    mean_heights = []
    max_heights = []
    pct95_heights = []
    n_pixels = []

    for fi in range(0, n_frames, sample_step):
        zed.set_svo_position(fi)
        if zed.grab(runtime) != sl.ERROR_CODE.SUCCESS:
            continue

        zed.retrieve_measure(depth_mat, sl.MEASURE.DEPTH)
        d = depth_mat.get_data()
        d_roi = d[r0:r1, c0:c1]

        diff = ref_roi - d_roi  # positive = grew closer
        valid = np.isfinite(diff)
        grown = valid & (diff > DEPTH_GROW_THRESHOLD)

        n_grown = grown.sum()
        if n_grown > 10:
            height_vals = diff[grown]
            mean_h = height_vals.mean()
            max_h = height_vals.max()
            pct95_h = np.percentile(height_vals, 95)
        else:
            mean_h = 0.0
            max_h = 0.0
            pct95_h = 0.0

        frames.append(fi)
        mean_heights.append(mean_h)
        max_heights.append(max_h)
        pct95_heights.append(pct95_h)
        n_pixels.append(n_grown)

        if fi % (sample_step * 20) == 0:
            print(f"  Frame {fi:5d}/{n_frames}: n_grown={n_grown:5d}, "
                  f"mean_h={mean_h:.1f}, p95_h={pct95_h:.1f}, max_h={max_h:.1f} mm")

    return (np.array(frames), np.array(mean_heights),
            np.array(max_heights), np.array(pct95_heights), np.array(n_pixels))


def detect_layer_steps(frames, heights, fps, layer_height, n_layers):
    """
    Detect layer transitions from height-over-time signal.
    Uses step detection: height quantized to nearest layer_height.
    """
    # Smooth the height signal
    if len(heights) < 5:
        return []

    from scipy.ndimage import median_filter
    h_smooth = median_filter(heights, size=5)

    # Quantize to layer number
    layer_nums = np.clip(np.round(h_smooth / layer_height), 0, n_layers).astype(int)

    # Find transitions
    transitions = []
    prev_layer = 0
    for i in range(1, len(layer_nums)):
        if layer_nums[i] != prev_layer and layer_nums[i] > prev_layer:
            transitions.append({
                "frame": int(frames[i]),
                "time_s": frames[i] / fps,
                "from_layer": prev_layer,
                "to_layer": int(layer_nums[i]),
                "height_mm": float(h_smooth[i]),
            })
            prev_layer = layer_nums[i]

    return transitions


def save_keyframes(zed, transitions, ref_depth, roi, out_dir):
    """Save annotated keyframes at each layer transition."""
    r0, r1, c0, c1 = roi
    image_mat = sl.Mat()
    depth_mat = sl.Mat()
    runtime = sl.RuntimeParameters()

    for tr in transitions:
        fi = tr["frame"]
        zed.set_svo_position(fi)
        if zed.grab(runtime) != sl.ERROR_CODE.SUCCESS:
            continue

        # RGB image
        zed.retrieve_image(image_mat, sl.VIEW.LEFT)
        img = image_mat.get_data()[:, :, :3].copy()

        # Depth diff heatmap
        zed.retrieve_measure(depth_mat, sl.MEASURE.DEPTH)
        d = depth_mat.get_data()
        diff = ref_depth - d
        diff[~np.isfinite(diff)] = 0
        diff = np.clip(diff, 0, N_LAYERS * LAYER_HEIGHT_MM + 5)

        # Normalize diff to colormap
        diff_norm = (diff / (N_LAYERS * LAYER_HEIGHT_MM + 5) * 255).astype(np.uint8)
        diff_color = cv2.applyColorMap(diff_norm, cv2.COLORMAP_JET)

        # Draw ROI rectangle on both
        cv2.rectangle(img, (c0, r0), (c1, r1), (0, 255, 0), 2)
        cv2.rectangle(diff_color, (c0, r0), (c1, r1), (0, 255, 0), 2)

        # Label
        label = f"Layer {tr['to_layer']} | t={tr['time_s']:.1f}s | h={tr['height_mm']:.1f}mm"
        cv2.putText(img, label, (c0, r0 - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(diff_color, label, (c0, r0 - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        # Combine side by side
        combined = np.hstack([img, diff_color])
        path = os.path.join(out_dir, f"layer_{tr['to_layer']:02d}_frame{fi:05d}.png")
        cv2.imwrite(path, combined)

    print(f"Saved {len(transitions)} keyframe images to {out_dir}")


def plot_results(frames, mean_h, max_h, pct95_h, n_pix, transitions, fps, out_dir):
    """Plot height growth and layer transitions."""
    if not HAS_MPL:
        print("matplotlib not available, skipping plots")
        return

    t = frames / fps

    fig, axes = plt.subplots(2, 1, figsize=(14, 8), sharex=True)
    fig.suptitle(f"Depth-based Layer Detection — 0324_174844\n"
                 f"exp08_cube_5: {N_LAYERS} layers × {LAYER_HEIGHT_MM}mm", fontsize=13)

    # Height plot
    ax1 = axes[0]
    ax1.plot(t, mean_h, label="mean height", color="blue", linewidth=0.8)
    ax1.plot(t, pct95_h, label="p95 height", color="orange", linewidth=0.8)
    ax1.plot(t, max_h, label="max height", color="red", alpha=0.4, linewidth=0.5)

    # Draw layer levels
    for i in range(1, N_LAYERS + 1):
        lh = i * LAYER_HEIGHT_MM
        ax1.axhline(lh, color="gray", linestyle="--", alpha=0.4, linewidth=0.5)
        ax1.text(t[-1] * 1.01, lh, f"L{i}", fontsize=7, va="center", alpha=0.6)

    # Mark transitions
    for tr in transitions:
        ax1.axvline(tr["time_s"], color="green", alpha=0.5, linewidth=1)
        ax1.annotate(f"L{tr['to_layer']}", (tr["time_s"], tr["height_mm"]),
                     fontsize=7, color="green", ha="center",
                     xytext=(0, 10), textcoords="offset points")

    ax1.set_ylabel("Height growth (mm)")
    ax1.set_title("Printed object height from depth difference")
    ax1.legend(loc="upper left", fontsize=8)
    ax1.grid(True, alpha=0.3)

    # Pixel count plot
    ax2 = axes[1]
    ax2.fill_between(t, n_pix, alpha=0.4, color="steelblue")
    ax2.plot(t, n_pix, linewidth=0.5, color="steelblue")
    for tr in transitions:
        ax2.axvline(tr["time_s"], color="green", alpha=0.5, linewidth=1)
    ax2.set_xlabel("Time (s)")
    ax2.set_ylabel("Grown pixels (count)")
    ax2.set_title("Number of pixels with depth decrease > threshold")
    ax2.grid(True, alpha=0.3)

    plt.tight_layout()
    path = os.path.join(out_dir, "layer_depth_detection.png")
    fig.savefig(path, dpi=200, bbox_inches="tight")
    print(f"Plot saved: {path}")
    plt.close(fig)


def main():
    out_dir = Path(OUT_DIR)
    out_dir.mkdir(parents=True, exist_ok=True)

    svo_path = str(Path(SVO_PATH).resolve())
    print(f"Opening SVO: {svo_path}")
    zed = open_svo(svo_path)
    n_frames = zed.get_svo_number_of_frames()
    fps = zed.get_camera_information().camera_configuration.fps
    print(f"  Frames: {n_frames}, FPS: {fps}, Duration: {n_frames/fps:.1f}s")

    # Step 1: Find printing ROI
    print("\n[1] Detecting print region (ref vs late frame)...")
    # Use a late frame where some printing has happened
    late_frame = int(n_frames * 0.85)
    r0, r1, c0, c1, ref_depth = find_print_roi(zed, REF_FRAME, late_frame)

    # Step 2: Scan depth over time
    print(f"\n[2] Scanning depth in ROI [{r0}:{r1}, {c0}:{c1}]...")
    frames, mean_h, max_h, pct95_h, n_pix = scan_depth_over_time(
        zed, ref_depth, (r0, r1, c0, c1), n_frames, SAMPLE_STEP
    )

    # Step 3: Detect layer transitions
    print("\n[3] Detecting layer transitions...")
    transitions = detect_layer_steps(frames, pct95_h, fps, LAYER_HEIGHT_MM, N_LAYERS)

    print(f"\n{'='*60}")
    print(f"  Layer Transitions Detected: {len(transitions)}")
    print(f"{'='*60}")
    for tr in transitions:
        print(f"  Layer {tr['to_layer']:2d} | frame {tr['frame']:5d} | "
              f"t={tr['time_s']:6.1f}s | height={tr['height_mm']:.1f}mm")

    # Step 4: Save keyframe images
    print("\n[4] Saving keyframe images...")
    save_keyframes(zed, transitions, ref_depth, (r0, r1, c0, c1), str(out_dir))

    # Step 5: Plot results
    print("\n[5] Plotting...")
    plot_results(frames, mean_h, max_h, pct95_h, n_pix, transitions, fps, str(out_dir))

    # Step 6: Compare with RSI data if available
    rsi_path = Path(RSI_PATH)
    if rsi_path.exists():
        print("\n[6] Cross-referencing with RSI data...")
        rsi = load_rsi_csv(str(rsi_path))
        rsi_t0 = rsi["timestamp"][0]
        # RSI layer transitions: where Z changes by > 1mm
        z = rsi["z_mm"]
        z_q = np.floor(z / LAYER_HEIGHT_MM) * LAYER_HEIGHT_MM
        changes = np.where(np.diff(z_q) > 0)[0]
        # Group consecutive into single events
        if len(changes) > 0:
            events = [changes[0]]
            for c in changes[1:]:
                if c - events[-1] > 5:
                    events.append(c)
            print(f"  RSI layer changes: {len(events)}")
            for ev in events:
                t_sec = rsi["timestamp"][ev] - rsi_t0
                print(f"    t={t_sec:6.1f}s  Z={z[ev]:.1f} -> {z[ev+1]:.1f}")

    zed.close()
    print("\nDone!")


if __name__ == "__main__":
    main()
