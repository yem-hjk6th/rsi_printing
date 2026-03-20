"""sam2_timing.py — SAM2 single-frame timing benchmark"""
import time, cv2, numpy as np, torch
from sam2.build_sam import build_sam2
from sam2.automatic_mask_generator import SAM2AutomaticMaskGenerator
import pyzed.sl as sl

CKPT = r"C:\Users\dell\Desktop\RSI\build\DataCollection\cam_adjust\t2\sam2_checkpoints\sam2.1_hiera_small.pt"
SVO  = r"C:\Users\dell\Desktop\RSI\recorded_data\20260310_183338\recording_20260310_183338.svo2"

# 1) 模型加载
t0 = time.perf_counter()
sam2 = build_sam2("configs/sam2.1/sam2.1_hiera_s.yaml", CKPT, device="cuda", apply_postprocessing=False)
gen = SAM2AutomaticMaskGenerator(sam2, points_per_side=32, points_per_batch=64,
    pred_iou_thresh=0.7, stability_score_thresh=0.85, min_mask_region_area=50)
t_load = time.perf_counter() - t0
print(f"Model load: {t_load:.2f}s")

# 2) 取帧
zed = sl.Camera()
p = sl.InitParameters()
p.set_from_svo_file(SVO)
p.svo_real_time_mode = False
p.depth_mode = sl.DEPTH_MODE.NONE
zed.open(p)
zed.set_svo_position(2000)
mat = sl.Mat()
zed.grab()
zed.retrieve_image(mat, sl.VIEW.LEFT)
bgr = mat.get_data()[:, :, :3].copy()
bgr = cv2.cvtColor(bgr, cv2.COLOR_RGB2BGR)
zed.close()

roi = bgr[300:950, 100:1800]
rgb = cv2.cvtColor(roi, cv2.COLOR_BGR2RGB)
print(f"Input: {rgb.shape[1]}x{rgb.shape[0]} ({rgb.shape[1]*rgb.shape[0]/1e6:.2f} Mpx)")

# 3) Benchmark (warmup = first run, then 2 more)
times = []
for i in range(3):
    torch.cuda.empty_cache()
    torch.cuda.synchronize()
    t1 = time.perf_counter()
    with torch.inference_mode():
        masks = gen.generate(rgb)
    torch.cuda.synchronize()
    dt = time.perf_counter() - t1
    times.append(dt)
    print(f"  Run {i+1}: {dt:.3f}s  ({len(masks)} masks)")

print(f"\nWarmup (run1): {times[0]:.3f}s")
if len(times) > 1:
    avg = np.mean(times[1:])
    print(f"Average (run2+): {avg:.3f}s per frame")
print(f"VRAM peak: {torch.cuda.max_memory_allocated()/1024**2:.0f} MB")
