"""
zed_res_bench_quick.py — Quick benchmark (no NEURAL download)
"""
import os, time, sys

if os.name == "nt":
    import glob as _g
    _cuda_bin = next(iter(sorted(
        _g.glob(r"C:\Program Files\NVIDIA GPU Computing Toolkit\CUDA\v*\bin"),
        reverse=True)), "")
    for p in [
        r"C:\Program Files (x86)\ZED SDK\bin",
        r"C:\Program Files (x86)\ZED SDK\dependencies\bin",
        _cuda_bin,
    ]:
        if p and os.path.isdir(p):
            os.add_dll_directory(p)

import pyzed.sl as sl

RESOLUTIONS = [
    ("HD2K",   sl.RESOLUTION.HD2K),
    ("HD1080", sl.RESOLUTION.HD1080),
    ("HD720",  sl.RESOLUTION.HD720),
    ("SVGA",   sl.RESOLUTION.SVGA),
    ("VGA",    sl.RESOLUTION.VGA),
]
FPS_OPTIONS = [100, 60, 30, 15]
DEPTH_MODES = [
    # ("NEURAL_PLUS", sl.DEPTH_MODE.NEURAL_PLUS),  # needs model download
    # ("NEURAL",      sl.DEPTH_MODE.NEURAL),        # needs model download
    ("ULTRA",       sl.DEPTH_MODE.ULTRA),
    ("QUALITY",     sl.DEPTH_MODE.QUALITY),
    ("PERFORMANCE", sl.DEPTH_MODE.PERFORMANCE),
    ("NONE",        sl.DEPTH_MODE.NONE),
]
WARMUP, BENCH = 10, 50

def run_bench(res_e, fps, dm_e):
    zed = sl.Camera()
    p = sl.InitParameters()
    p.camera_resolution = res_e
    p.camera_fps = fps
    p.depth_mode = dm_e
    p.depth_stabilization = 0
    if zed.open(p) != sl.ERROR_CODE.SUCCESS:
        return None
    info = zed.get_camera_information().camera_configuration
    W, H = info.resolution.width, info.resolution.height
    set_fps = zed.get_init_parameters().camera_fps
    rt = sl.RuntimeParameters(); img = sl.Mat(); dep = sl.Mat()
    for _ in range(WARMUP):
        zed.grab(rt)
    t0 = time.perf_counter(); ok = 0
    for _ in range(BENCH):
        if zed.grab(rt) == sl.ERROR_CODE.SUCCESS:
            zed.retrieve_image(img, sl.VIEW.LEFT)
            if dm_e != sl.DEPTH_MODE.NONE:
                zed.retrieve_measure(dep, sl.MEASURE.DEPTH)
            ok += 1
    elapsed = time.perf_counter() - t0
    zed.close()
    return (ok / elapsed, W, H, set_fps) if ok else None

def main():
    print("=" * 75)
    print("  ZED 2i Quick Benchmark (no NEURAL)")
    print("=" * 75)
    zed = sl.Camera()
    p = sl.InitParameters(); p.camera_resolution = sl.RESOLUTION.VGA; p.depth_mode = sl.DEPTH_MODE.NONE
    if zed.open(p) != sl.ERROR_CODE.SUCCESS:
        print("[ERROR] Cannot open ZED camera"); return
    ci = zed.get_camera_information()
    print(f"  Camera: {ci.camera_model}  S/N {ci.serial_number}")
    zed.close()

    results = []
    total = len(RESOLUTIONS) * len(FPS_OPTIONS) * len(DEPTH_MODES)
    i = 0
    for rn, re in RESOLUTIONS:
        for fps in FPS_OPTIONS:
            for dn, de in DEPTH_MODES:
                i += 1
                tag = f"[{i}/{total}] {rn:>6s}@{fps:>3d} {dn:<12s}"
                sys.stdout.write(f"  {tag} ... ")
                sys.stdout.flush()
                ret = run_bench(re, fps, de)
                if ret is None:
                    print("FAIL")
                    results.append((rn, fps, dn, None, None, None))
                else:
                    mfps, W, H, sf = ret
                    st = "OK" if mfps >= sf * 0.9 else "SLOW"
                    print(f"{W}x{H} set={sf} measured={mfps:.1f}fps [{st}]")
                    results.append((rn, fps, dn, W, H, mfps))

    print("\n" + "=" * 75)
    print("  RESULTS")
    print("=" * 75)
    print(f"  {'Res':<8s} {'FPS':>4s}  {'Depth':<12s} {'WxH':<12s} {'Actual':>8s}")
    print("  " + "-" * 52)
    for r, f, d, W, H, m in results:
        if m is not None:
            print(f"  {r:<8s} {f:>4d}  {d:<12s} {W}x{H:<5d} {m:>7.1f}")

    ok_rgb = [(r,f,m) for r,f,d,W,H,m in results if d=="NONE" and m]
    ok_dep = [(r,f,d,m) for r,f,d,W,H,m in results if d!="NONE" and m]
    if ok_rgb:
        b = max(ok_rgb, key=lambda x: (x[0]=="HD2K", x[2]))
        print(f"\n  >> Best RGB:   {b[0]}@{b[1]}fps ({b[2]:.1f}fps)")
    if ok_dep:
        b = max(ok_dep, key=lambda x: (x[0]=="HD2K", x[3]))
        print(f"  >> Best Depth: {b[0]}@{b[1]}fps depth={b[2]} ({b[3]:.1f}fps)")

if __name__ == "__main__":
    main()
