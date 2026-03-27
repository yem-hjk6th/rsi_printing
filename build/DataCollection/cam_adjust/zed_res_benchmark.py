"""
zed_res_benchmark.py — ZED 2i resolution & depth mode benchmark
Tests all resolution × FPS × depth-mode combos, reports which ones
the current GPU (RTX 5070 Ti) can sustain.
Usage:  python zed_res_benchmark.py
"""

import os
import time

if os.name == "nt":
    for p in [
        r"C:\Program Files (x86)\ZED SDK\bin",
        r"C:\Program Files (x86)\ZED SDK\dependencies\bin",
        r"C:\Program Files\NVIDIA GPU Computing Toolkit\CUDA\v12.8\bin",
    ]:
        if os.path.isdir(p):
            os.add_dll_directory(p)

import pyzed.sl as sl

# ── All combos to test ────────────────────────────────────────────────────────
RESOLUTIONS = [
    ("HD2K",   sl.RESOLUTION.HD2K),    # 2208×1242
    ("HD1200", sl.RESOLUTION.HD1200),   # 1920×1200  (ZED X only, expect fail)
    ("HD1080", sl.RESOLUTION.HD1080),   # 1920×1080
    ("HD720",  sl.RESOLUTION.HD720),    # 1280×720
    ("SVGA",   sl.RESOLUTION.SVGA),     # 960×600
    ("VGA",    sl.RESOLUTION.VGA),      # 672×376
]

FPS_OPTIONS = [100, 60, 30, 15]

DEPTH_MODES = [
    ("NEURAL_PLUS", sl.DEPTH_MODE.NEURAL_PLUS),
    ("NEURAL",      sl.DEPTH_MODE.NEURAL),
    ("ULTRA",       sl.DEPTH_MODE.ULTRA),
    ("QUALITY",     sl.DEPTH_MODE.QUALITY),
    ("PERFORMANCE", sl.DEPTH_MODE.PERFORMANCE),
    ("NONE",        sl.DEPTH_MODE.NONE),          # RGB only, no depth
]

WARMUP_FRAMES = 10
BENCH_FRAMES  = 50


def run_bench(res_name, res_enum, fps, depth_name, depth_enum):
    """Return (actual_fps, W, H) or None on failure."""
    zed = sl.Camera()
    init_p = sl.InitParameters()
    init_p.camera_resolution = res_enum
    init_p.camera_fps = fps
    init_p.depth_mode = depth_enum
    init_p.depth_stabilization = 0

    status = zed.open(init_p)
    if status != sl.ERROR_CODE.SUCCESS:
        return None

    info = zed.get_camera_information().camera_configuration
    W = info.resolution.width
    H = info.resolution.height
    actual_set_fps = zed.get_init_parameters().camera_fps

    runtime = sl.RuntimeParameters()
    img = sl.Mat()
    depth = sl.Mat()

    # warmup
    for _ in range(WARMUP_FRAMES):
        zed.grab(runtime)

    # benchmark
    t0 = time.perf_counter()
    ok = 0
    for _ in range(BENCH_FRAMES):
        if zed.grab(runtime) == sl.ERROR_CODE.SUCCESS:
            zed.retrieve_image(img, sl.VIEW.LEFT)
            if depth_enum != sl.DEPTH_MODE.NONE:
                zed.retrieve_measure(depth, sl.MEASURE.DEPTH)
            ok += 1
    elapsed = time.perf_counter() - t0
    zed.close()

    if ok == 0:
        return None
    measured_fps = ok / elapsed
    return measured_fps, W, H, actual_set_fps


def main():
    print("=" * 80)
    print("  ZED 2i  Resolution & Depth-Mode Benchmark")
    print("=" * 80)

    # Check camera is connected
    zed = sl.Camera()
    init_p = sl.InitParameters()
    init_p.camera_resolution = sl.RESOLUTION.VGA
    init_p.depth_mode = sl.DEPTH_MODE.NONE
    st = zed.open(init_p)
    if st != sl.ERROR_CODE.SUCCESS:
        print(f"\n[ERROR] Cannot open ZED camera: {st}")
        print("        Please connect ZED 2i and retry.")
        return
    model = zed.get_camera_information().camera_model
    serial = zed.get_camera_information().serial_number
    fw = zed.get_camera_information().camera_configuration.firmware_version
    print(f"\nCamera  : {model}  (S/N {serial}, FW {fw})")
    zed.close()

    results = []

    for res_name, res_enum in RESOLUTIONS:
        for fps in FPS_OPTIONS:
            for depth_name, depth_enum in DEPTH_MODES:
                tag = f"{res_name:>7s} @ {fps:>3d}fps | depth={depth_name:<14s}"
                ret = run_bench(res_name, res_enum, fps, depth_name, depth_enum)
                if ret is None:
                    print(f"  {tag}  -->  FAILED / unsupported")
                    results.append((res_name, fps, depth_name, None, None, None))
                else:
                    mfps, W, H, set_fps = ret
                    status = "OK" if mfps >= set_fps * 0.9 else "SLOW"
                    print(f"  {tag}  -->  {W}x{H}  set={set_fps}fps  "
                          f"measured={mfps:5.1f}fps  [{status}]")
                    results.append((res_name, fps, depth_name, W, H, mfps))

    # ── Summary ───────────────────────────────────────────────────────────────
    print("\n" + "=" * 80)
    print("  SUMMARY — Successful configurations")
    print("=" * 80)
    print(f"  {'Resolution':<10s} {'FPS':>4s}  {'Depth':<14s}  {'WxH':<12s}  {'Measured':>8s}")
    print("  " + "-" * 60)
    for res, fps, dm, W, H, mfps in results:
        if mfps is not None:
            print(f"  {res:<10s} {fps:>4d}  {dm:<14s}  {W}x{H:<5d}  {mfps:>7.1f} fps")

    # Best RGB-only
    rgb_best = [(r, f, m) for r, f, d, W, H, m in results
                if d == "NONE" and m is not None]
    if rgb_best:
        best = max(rgb_best, key=lambda x: (x[0] == "HD2K", x[2]))
        print(f"\n  >> Best RGB stereo : {best[0]} @ {best[1]}fps  ({best[2]:.1f} fps measured)")

    # Best depth
    depth_best = [(r, f, d, m) for r, f, d, W, H, m in results
                  if d != "NONE" and m is not None]
    if depth_best:
        best = max(depth_best, key=lambda x: (x[0] == "HD2K", x[3]))
        print(f"  >> Best Depth      : {best[0]} @ {best[1]}fps  depth={best[2]}  ({best[3]:.1f} fps measured)")

    print()


if __name__ == "__main__":
    main()
