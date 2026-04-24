"""Calculate ZED 2i minimum stereo overlap distance."""
import math

B = 120.0652  # baseline in mm

resolutions = {
    'HD2K':   (2208, 1242, 60.1131, 1907.9009),
    'HD1080': (1920, 1080, 53.3170, 1912.2614),
    'HD720':  (1280, 720,  67.8289, 951.9376),
    'VGA':    (672,  376,  70.4789, 475.6327),
}

print("=" * 72)
print("ZED 2i Stereo Overlap Minimum Distance Calculation")
print("Camera: ZED 2i  S/N: 37529394  Firmware: 1523")
print(f"Baseline (B) = {B:.4f} mm")
print("=" * 72)

print()
print("Geometry:")
print("  Two parallel cameras separated by baseline B.")
print("  Each has horizontal half-FOV angle alpha = h_fov / 2.")
print("  At distance d, left cam right boundary  = d * tan(alpha)")
print("  At distance d, right cam left boundary   = B - d * tan(alpha)")
print("  Overlap exists when: d * tan(alpha) > B - d * tan(alpha)")
print("  => d_min = B / (2 * tan(alpha)) = B / (2 * tan(h_fov/2))")
print()

header = f"{'Resolution':<10} {'Size':<12} {'h_fov':<10} {'h_fov/2':<10} {'tan(h/2)':<12} {'d_min(mm)':<12} {'d_min(cm)':<10}"
print(header)
print("-" * len(header))

for name, (w, h, h_fov, fx) in resolutions.items():
    half_fov_deg = h_fov / 2
    half_fov_rad = math.radians(half_fov_deg)
    tan_half = math.tan(half_fov_rad)
    d_min = B / (2 * tan_half)
    size_str = f"{w}x{h}"
    print(f"{name:<10} {size_str:<12} {h_fov:<10.4f} {half_fov_deg:<10.4f} {tan_half:<12.6f} {d_min:<12.2f} {d_min/10:<10.2f}")

print()
print("--- Detailed Calculation for HD720 (commonly used) ---")
hfov = 67.8289
alpha = hfov / 2
alpha_rad = math.radians(alpha)
tan_a = math.tan(alpha_rad)
d = B / (2 * tan_a)
print(f"  h_fov = {hfov:.4f} deg")
print(f"  alpha = h_fov/2 = {alpha:.4f} deg")
print(f"  tan(alpha) = tan({alpha:.4f}) = {tan_a:.6f}")
print(f"  d_min = B / (2 * tan(alpha))")
print(f"        = {B:.4f} / (2 * {tan_a:.6f})")
print(f"        = {B:.4f} / {2*tan_a:.6f}")
print(f"        = {d:.2f} mm")
print(f"        = {d/10:.2f} cm")

print()
print("--- Physical meaning ---")
print(f"  At d_min = {d:.1f} mm, overlap width = 0 (theoretical single point).")
print(f"  Practical stereo matching needs some overlap width.")
print()
print("  Overlap width at various distances (HD720):")
print(f"  {'Distance':<12} {'Overlap width':<16} {'Single cam width':<18} {'Overlap ratio':<14}")
print(f"  {'-'*56}")
for d_ex in [100, 150, 200, 300, 500, 1000, 2000]:
    ow = 2 * d_ex * tan_a - B
    if ow < 0:
        print(f"  {d_ex:>6} mm   {'NO OVERLAP':<16} {2*d_ex*tan_a:>8.1f} mm        {'0%':<14}")
    else:
        total_w = 2 * d_ex * tan_a
        pct = ow / total_w * 100
        print(f"  {d_ex:>6} mm   {ow:>8.1f} mm      {total_w:>8.1f} mm          {pct:>5.1f}%")
