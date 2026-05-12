"""
Check whether the ZED 2i camera can see the full 100mm ArUco marker
at each of the 35 calibration poses in aut_cal2.src.

Uses T_cam2gripper from R6 Daniilidis, Tool3 TCP offset,
and estimated ZED 2i HD1080 intrinsics.
"""
import numpy as np

# ── T_cam2gripper from R6 Daniilidis (units: meters) ──────────
T_c2g = np.array([
    [ 0.047495,  0.811369,  0.582601, -0.306804],
    [ 0.998643, -0.051054, -0.010310, -0.077421],
    [ 0.021379,  0.582300, -0.812693,  0.388133],
    [ 0.0,       0.0,       0.0,       1.0      ]
])
R_c2g = T_c2g[:3, :3]
t_c2g_mm = T_c2g[:3, 3] * 1000.0   # → mm

# ── Tool3 TCP offset in flange frame (mm) ─────────────────────
TCP_OFFSET = np.array([225.55, -0.6, 72.83])

# ── ZED 2i HD1080 approximate intrinsics ──────────────────────
fx = fy = 1068.0
cx, cy  = 960.0, 540.0
IMG_W, IMG_H = 1920, 1080

# ── ArUco marker half-size (mm) ───────────────────────────────
HALF_MARKER = 50.0   # 100 mm / 2


def euler_zyx(a_deg, b_deg, c_deg):
    """KUKA ZYX  R = Rz(A)·Ry(B)·Rx(C)  →  flange-to-world rotation."""
    a, b, c = np.radians([a_deg, b_deg, c_deg])
    ca, sa = np.cos(a), np.sin(a)
    cb, sb = np.cos(b), np.sin(b)
    cc, sc = np.cos(c), np.sin(c)
    Rz = np.array([[ ca, -sa, 0], [ sa,  ca, 0], [0, 0, 1]])
    Ry = np.array([[ cb,  0, sb], [  0,  1, 0], [-sb, 0, cb]])
    Rx = np.array([[ 1,  0,  0], [  0,  cc, -sc], [0, sc, cc]])
    return Rz @ Ry @ Rx


def check(tcp_xyz, abc, marker_xyz):
    """Return (u, v, half_px, cam_dist_mm, fully_visible)."""
    R_w = euler_zyx(*abc)
    flange = tcp_xyz - R_w @ TCP_OFFSET
    cam_pos = flange + R_w @ t_c2g_mm
    R_cam2w = R_w @ R_c2g
    d_world = marker_xyz - cam_pos
    d_cam   = R_cam2w.T @ d_world          # camera-frame vector
    if d_cam[2] <= 0:
        return None
    u  = fx * d_cam[0] / d_cam[2] + cx
    v  = fy * d_cam[1] / d_cam[2] + cy
    hp = fx * HALF_MARKER / d_cam[2]       # half-marker in pixels
    ok = (u - hp >= 0 and u + hp <= IMG_W and
          v - hp >= 0 and v + hp <= IMG_H)
    return u, v, hp, d_cam[2], ok


# ── 35 poses from aut_cal2.src (after offset to new bed) ──────
POSES = [
    # ( X,    Y,    Z,    A,    B,   C )
    (760, -410,  30,    0,  90,   0),   # P01
    (760, -410,  30,    5,  75,  -6),   # P02
    (760, -410,  30,   -5,  85,   6),   # P03
    (760, -410,  30,   18,  78,  10),   # P04
    (760, -410,  30,    8,  82,  18),   # P05
    (760, -410,  30,   25,  72, -12),   # P06
    (760, -410,  30,  -15,  80,  -8),   # P07
    (760, -410,  30,  172,  82, 162),   # P08
    (760, -410,  30, -170,  78,-168),   # P09
    (760, -410,  30,  160,  85, 165),   # P10
    (760, -410,  10,   10,  88,   5),   # P11
    (760, -410,  50,    8,  65,  -8),   # P12
    (760, -410,  40,   15,  70,  -8),   # P13
    (760, -410,  40,   10,  74,  15),   # P14
    (760, -410,  60,    5,  62,  -5),   # P15
    (830, -310,  10,  168,  85, 172),   # P16
    (830, -590,  10,   12,  82,   8),   # P17
    (830, -450,  10,   20,  86,  18),   # P18
    (830, -380,  10,  155,  88, 158),   # P19
    (830, -530,  10,  165,  84, 170),   # P20
    (690, -330,  20,   10,  72,   6),   # P21
    (690, -570,  20,  170,  73, 173),   # P22
    (690, -450,  20,   22,  78,  20),   # P23
    (670, -410,  40,   -8,  68,   5),   # P24
    (690, -390,  25, -165,  76,-170),   # P25
    (810, -410,   0, -172,  75, 175),   # P26
    (780, -510,  10,  175,  74, 174),   # P27
    (740, -370,  20,    8,  76,  -5),   # P28
    (710, -530,  25,   15,  78,  10),   # P29
    (770, -350,  15,  165,  80, 168),   # P30
    (790, -550,   5,   30,  84,  25),   # P31
    (750, -390,  35,  -12,  70, -10),   # P32
    (720, -450,  30,   35,  82,  28),   # P33
    (800, -330,  10, -155,  78,-160),   # P34
    (760, -410,  55,   12,  63,  -6),   # P35
]

# ── Also check old R6 poses for comparison ────────────────────
OLD_POSES = [
    (850,  40, 460,   0,  90,   0),   # P01
    (850,  40, 460,   5,  75,  -6),   # P02
    (850,  40, 460,  -5, 108,   6),   # P03
    (850,  40, 480,   8,  65,  -8),   # P04
    (850,  40, 480,  -8, 115,   8),   # P05
    (850,  40, 460,  18,  78,  10),   # P06
    (850,  40, 460, -18, 102, -10),   # P07
    (850,  40, 470,  15,  70,  -8),   # P08
    (850,  40, 470, -15, 110,   8),   # P09
    (850,  40, 460,   8,  82,  18),   # P10
    (850,  40, 460,  -8,  98, -18),   # P11
    (850,  40, 470,  10,  74,  15),   # P12
    (850,  40, 470, -10, 106, -15),   # P13
    (920, 140, 440, -12,  95,  -8),   # P14
    (920,-140, 440,  12,  82,   8),   # P15
    (780, 120, 450,  10,  72,   6),   # P16
    (780,-120, 450, -10, 108,  -6),   # P17
    (900,  40, 430,   8, 105,  -5),   # P18
    (760,  40, 470,  -8,  68,   5),   # P19
]


def run_check(label, poses, marker_z):
    marker = np.array([760.0, -410.0, marker_z])  # XY at nominal center
    # For old poses, adjust marker XY to old center
    if "OLD" in label:
        marker = np.array([850.0,  40.0, marker_z])

    print(f"\n{'='*80}")
    print(f"  {label}")
    print(f"  Marker at Z = {marker_z:.1f} mm   (marker XY = {marker[0]:.0f}, {marker[1]:.0f})")
    print(f"{'='*80}")
    print(f"{'Pose':>5} {'TCP_Z':>6} {'B':>4} | {'u':>7} {'v':>7} {'half':>5} "
          f"{'dist':>6} {'v_bot':>6} | {'Status'}")
    print("-" * 80)
    ok_count = 0
    for i, (x, y, z, a, b, c) in enumerate(poses):
        tcp = np.array([x, y, z], dtype=float)
        # Use the actual marker XY for spatial poses
        if "OLD" not in label:
            mrk = np.array([760.0, -410.0, marker_z])
        else:
            mrk = np.array([850.0,  40.0, marker_z])
        res = check(tcp, (a, b, c), mrk)
        if res is None:
            print(f"  P{i+1:02d}  {z:5.0f}  {b:3.0f} | {'behind camera':>40} | FAIL")
            continue
        u, v, hp, dist, vis = res
        v_bot = v + hp
        status = "  OK " if vis else " FAIL"
        if not vis and v_bot > IMG_H:
            status = " FAIL (bottom cut)"
        elif not vis and v - hp < 0:
            status = " FAIL (top cut)"
        elif not vis and u + hp > IMG_W:
            status = " FAIL (right cut)"
        elif not vis and u - hp < 0:
            status = " FAIL (left cut)"
        if vis:
            ok_count += 1
        print(f"  P{i+1:02d}  {z:5.0f}  {b:3.0f} | {u:7.1f} {v:7.1f} {hp:5.1f} "
              f"{dist:6.0f} {v_bot:6.1f} | {status}")
    print(f"\nVisible: {ok_count}/{len(poses)}")
    return ok_count


print("=" * 80)
print("  ArUco Marker Visibility Check for aut_cal2.src")
print("  Camera: ZED 2i HD1080 (fx=fy~1068, 1920x1080)")
print("  T_cam2gripper: R6 Daniilidis")
print("  Tool3 TCP: (225.55, -0.6, 72.83) mm")
print("  Marker: 100mm x 100mm")
print("=" * 80)

# ── Scenario A: Marker on bed surface ─────────────────────────
run_check("NEW poses – Marker on BED (Z = -12.5)", POSES, -12.5)

# ── Scenario B: Marker raised on a stand ──────────────────────
for stand_h in [50, 100, 130, 150, 200]:
    mz = -12.5 + stand_h
    run_check(f"NEW poses – Marker on STAND +{stand_h}mm (Z = {mz:.1f})", POSES, mz)

# ── Old R6 poses for comparison ───────────────────────────────
# Try several marker heights to find what worked
for mz in [0, 200, 400, 460, 500, 520]:
    run_check(f"OLD R6 poses – Marker at Z = {mz}", OLD_POSES, mz)

# ── Find minimum stand height for ALL new poses visible ───────
print("\n" + "=" * 80)
print("  Sweep: minimum stand height for all 35 new poses to see the full marker")
print("=" * 80)
for stand_h in range(0, 301, 5):
    mz = -12.5 + stand_h
    marker = np.array([760.0, -410.0, mz])
    all_ok = True
    for (x, y, z, a, b, c) in POSES:
        res = check(np.array([x, y, z], dtype=float), (a, b, c), marker)
        if res is None or not res[4]:
            all_ok = False
            break
    if all_ok:
        print(f"  ✓ All 35 visible at stand_height = {stand_h} mm  (marker Z = {mz:.1f})")
        break
else:
    # Find how many are visible at each height
    print("  Could not find stand height ≤300mm for all 35 visible.")
    print("  Checking counts:")
    for stand_h in range(0, 301, 10):
        mz = -12.5 + stand_h
        marker = np.array([760.0, -410.0, mz])
        cnt = 0
        for (x, y, z, a, b, c) in POSES:
            res = check(np.array([x, y, z], dtype=float), (a, b, c), marker)
            if res is not None and res[4]:
                cnt += 1
        print(f"    stand={stand_h:3d}mm (Z={mz:6.1f}):  {cnt}/35 visible")
