"""
quality_check.py — fast data-quality gate for a capture session.

Run this RIGHT AFTER capture, before committing to a full reconstruction.
It answers "is this data good enough, and if not, which frames do I re-record?"
in ~10-20 s instead of waiting out a 1-2 min recon that may crash anyway.

Two layers of checks:

  Per-view  (instant) — from view_NNN_depth.npy:
      * valid point count           (was view_001's 86k vs ~400k the failure mode)
      * ROI fill ratio              (selected ROI but half of it has no depth?)
      * depth p10/p50/p90           (subject actually inside DEPTH_MIN..MAX?)
      * right image present         (needed for --ffs)

  Per-pair  (a few seconds each) — fast pairwise registration probe on the
  ROI point clouds (voxel downsample -> FPFH+RANSAC -> point-to-plane ICP):
      * fitness                     (predicts whether full registration breaks)
      * inter-view translation / rotation  (jump too big => will be dropped)

  This pairwise probe is deliberately lighter than register.pairwise() — no
  colored ICP — so it never throws the "No correspondences found" error; a bad
  pair simply shows up as low fitness.

Verdict: PASS / WARN / FAIL, with a per-frame punch list of what to re-record.
Writes quality_report.json into the capture dir.

Usage:
    python Vision/ffs_poisson_detail/pipeline.py qc <capture_dir>
    python Vision/ffs_poisson_detail/quality_check.py <capture_dir> [--no-pairs]
"""

import os, sys, json, time, argparse
from pathlib import Path
from concurrent.futures import ProcessPoolExecutor

os.environ.setdefault("KMP_DUPLICATE_LIB_OK", "TRUE")

if sys.platform == "win32":
    try:
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
        sys.stderr.reconfigure(encoding="utf-8", errors="replace")
    except Exception:
        pass

import numpy as np
import open3d as o3d

sys.path.insert(0, str(Path(__file__).resolve().parent))
import config


# ── verdict helpers ──────────────────────────────────────────────────────────
_RANK = {"OK": 0, "PASS": 0, "WARN": 1, "FAIL": 2}
_INV  = {0: "PASS", 1: "WARN", 2: "FAIL"}


def _worst(*verdicts):
    return _INV[max(_RANK[v] for v in verdicts)]


def _mark(v):
    return {"OK": "[ OK ]", "PASS": "[ OK ]", "WARN": "[WARN]", "FAIL": "[FAIL]"}[v]


# ══════════════════════════════════════════════════════════════════════════════
#  Per-view checks
# ══════════════════════════════════════════════════════════════════════════════

def _check_view(depth_path):
    """Per-view stats straight from depth.npy. Returns a dict."""
    stem  = depth_path.name.replace("_depth.npy", "")          # view_NNN
    data  = np.load(str(depth_path)).astype(np.float32)
    finite = np.isfinite(data)
    n_pts  = int(finite.sum())

    rec = {"view": stem, "n_points": n_pts}

    if n_pts == 0:
        rec.update(roi_fill=0.0, depth_p10=None, depth_p50=None, depth_p90=None,
                   depth_in_range=False,
                   has_right=(depth_path.parent / f"{stem}_right.png").exists(),
                   verdict="FAIL", notes=["empty depth — no valid pixels"])
        return rec

    ys, xs = np.where(finite)
    bbox_w = int(xs.max() - xs.min() + 1)
    bbox_h = int(ys.max() - ys.min() + 1)
    roi_fill = n_pts / float(bbox_w * bbox_h)

    vals = data[finite]
    p10, p50, p90 = (float(np.percentile(vals, q)) for q in (10, 50, 90))
    in_range = (config.DEPTH_MIN_M <= p10) and (p90 <= config.DEPTH_MAX_M)
    has_right = (depth_path.parent / f"{stem}_right.png").exists()

    notes = []
    verdict = "OK"
    if n_pts < config.QC_FAIL_POINTS:
        verdict = "FAIL"; notes.append(
            f"only {n_pts:,} pts (<{config.QC_FAIL_POINTS:,}) — ROI too small / bad frame")
    elif n_pts < config.QC_MIN_POINTS:
        verdict = "WARN"; notes.append(
            f"{n_pts:,} pts (<{config.QC_MIN_POINTS:,}) — sparse, may register poorly")
    if roi_fill < config.QC_MIN_ROI_FILL:
        verdict = _worst(verdict, "WARN"); notes.append(
            f"ROI only {roi_fill*100:.0f}% filled — low-texture / reflective surface?")
    if not in_range:
        verdict = _worst(verdict, "WARN"); notes.append(
            f"depth p10/p90 = {p10:.2f}/{p90:.2f} m outside "
            f"[{config.DEPTH_MIN_M:.2f}, {config.DEPTH_MAX_M:.2f}] m")

    rec.update(roi_bbox=[bbox_w, bbox_h], roi_fill=round(roi_fill, 3),
               depth_p10=round(p10, 3), depth_p50=round(p50, 3),
               depth_p90=round(p90, 3), depth_in_range=in_range,
               has_right=has_right, verdict=verdict, notes=notes)
    return rec


# ══════════════════════════════════════════════════════════════════════════════
#  Per-pair fast registration probe
# ══════════════════════════════════════════════════════════════════════════════

def _prep(pcd, voxel):
    """Downsample + normals + FPFH for the fast probe."""
    down = pcd.voxel_down_sample(voxel)
    down.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=voxel * 2.5, max_nn=30))
    fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        down, o3d.geometry.KDTreeSearchParamHybrid(radius=voxel * 5.0, max_nn=100))
    return down, fpfh


def _probe_pair(pcd_a, pcd_b, voxel, ransac_iter):
    """
    Lightweight pairwise registration: FPFH+RANSAC global, then point-to-plane
    ICP refine. Returns (fitness, rmse_m, trans_m, rot_deg). No colored ICP, so
    this never raises — a bad pair just comes back with low fitness.
    """
    if len(pcd_a.points) < 100 or len(pcd_b.points) < 100:
        return 0.0, 0.0, 0.0, 0.0

    a, fa = _prep(pcd_a, voxel)
    b, fb = _prep(pcd_b, voxel)

    dist = voxel * 1.5
    ransac = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
        a, b, fa, fb, mutual_filter=True, max_correspondence_distance=dist,
        estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPoint(False),
        ransac_n=3,
        checkers=[o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),
                  o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(dist)],
        criteria=o3d.pipelines.registration.RANSACConvergenceCriteria(int(ransac_iter), 0.999),
    )
    icp = o3d.pipelines.registration.registration_icp(
        a, b, dist, ransac.transformation,
        o3d.pipelines.registration.TransformationEstimationPointToPlane(),
        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=50),
    )
    T = icp.transformation
    trans = float(np.linalg.norm(T[:3, 3]))
    cos_a = (np.trace(T[:3, :3]) - 1.0) / 2.0
    rot   = float(np.degrees(np.arccos(np.clip(cos_a, -1.0, 1.0))))
    return float(icp.fitness), float(icp.inlier_rmse), trans, rot


def _pair_worker(task):
    """
    ProcessPoolExecutor worker. Loads the two ROI point clouds from disk and
    runs the probe. Takes/returns only picklable primitives so it parallelises
    cleanly across processes.

    task   = (stem_a, ply_path_a, stem_b, ply_path_b, voxel, ransac_iter)
    return = (stem_a, stem_b, fitness, rmse_m, trans_m, rot_deg)
    """
    stem_a, ply_a, stem_b, ply_b, voxel, ransac_iter = task
    pcd_a = o3d.io.read_point_cloud(ply_a) if Path(ply_a).exists() \
            else o3d.geometry.PointCloud()
    pcd_b = o3d.io.read_point_cloud(ply_b) if Path(ply_b).exists() \
            else o3d.geometry.PointCloud()
    fit, rmse, trans, rot = _probe_pair(pcd_a, pcd_b, voxel, ransac_iter)
    return (stem_a, stem_b, fit, rmse, trans, rot)


def _pair_verdict(stem_a, stem_b, fit, rmse, trans, rot):
    """Apply thresholds to raw probe metrics. Runs in the main process."""
    notes = []
    verdict = "OK"
    if fit < config.QC_PAIR_FAIL_FITNESS:
        verdict = "FAIL"; notes.append(
            f"fitness {fit:.2f} (<{config.QC_PAIR_FAIL_FITNESS}) — "
            f"pair will break registration, re-record the {stem_a}->{stem_b} transition")
    elif fit < config.QC_PAIR_MIN_FITNESS:
        verdict = "WARN"; notes.append(
            f"fitness {fit:.2f} (<{config.QC_PAIR_MIN_FITNESS}) — weak overlap")
    if rot > config.QC_PAIR_MAX_ROT_DEG:
        verdict = _worst(verdict, "WARN"); notes.append(
            f"inter-view rotation {rot:.0f} deg (>{config.QC_PAIR_MAX_ROT_DEG:.0f}) — "
            f"camera jumped too far between frames")

    return {"pair": f"{stem_a}->{stem_b}", "fitness": round(fit, 3),
            "rmse_mm": round(rmse * 1000, 2), "trans_mm": round(trans * 1000, 1),
            "rot_deg": round(rot, 1), "verdict": verdict, "notes": notes}


def _resolve_workers():
    """QC_WORKERS=0 → auto: min(cpu//2, 8), at least 1."""
    if config.QC_WORKERS and config.QC_WORKERS > 0:
        return int(config.QC_WORKERS)
    return max(1, min((os.cpu_count() or 2) // 2, 8))


# ══════════════════════════════════════════════════════════════════════════════
#  Top-level
# ══════════════════════════════════════════════════════════════════════════════

def check_dir(data_dir, probe_pairs=True):
    """
    Run the full quality check on a capture directory.
    Returns a report dict and also writes quality_report.json into data_dir.
    """
    data_dir = Path(data_dir).resolve()
    t0 = time.perf_counter()

    depth_files = sorted(data_dir.glob("view_*_depth.npy"))
    if not depth_files:
        print(f"[qc] No view_*_depth.npy in {data_dir}")
        return {"verdict": "FAIL", "reason": "no views"}

    print(f"\n{'='*66}")
    print(f"Data-quality check — {data_dir.name}  ({len(depth_files)} views)")
    print('='*66)

    # ── per-view ──────────────────────────────────────────────────────────
    print(f"\nPer-view  (point count / ROI fill / depth range)")
    print(f"  {'view':<11} {'points':>9}  {'ROI fill':>8}  {'depth p10/50/90 (m)':>22}  right  verdict")
    view_recs = []
    for df in depth_files:
        r = _check_view(df)
        view_recs.append(r)
        if r["n_points"] == 0:
            depthstr = "  --  /  --  /  --"
        else:
            depthstr = f"{r['depth_p10']:.2f} / {r['depth_p50']:.2f} / {r['depth_p90']:.2f}"
        rightstr = "yes" if r.get("has_right") else "NO "
        print(f"  {_mark(r['verdict'])} {r['view']:<5} {r['n_points']:>9,}  "
              f"{r['roi_fill']*100:>7.0f}%  {depthstr:>22}   {rightstr}")
        for note in r["notes"]:
            print(f"         - {note}")

    # ── per-pair ──────────────────────────────────────────────────────────
    # Each consecutive pair is an independent FPFH+RANSAC+ICP probe, so they
    # parallelise across processes. Workers load their own PLYs from disk.
    pair_recs = []
    if probe_pairs and len(depth_files) >= 2:
        stems = [df.name.replace("_depth.npy", "") for df in depth_files]
        tasks = [
            (sa, str(data_dir / f"{sa}.ply"), sb, str(data_dir / f"{sb}.ply"),
             config.QC_PROBE_VOXEL, config.QC_PROBE_RANSAC_ITER)
            for sa, sb in zip(stems[:-1], stems[1:])
        ]
        workers = _resolve_workers()
        print(f"\nPer-pair  (fast registration probe — {len(tasks)} pairs, "
              f"{workers} worker{'s' if workers > 1 else ''})")
        print(f"  {'pair':<24} {'fitness':>7}  {'rmse':>7}  {'trans':>8}  {'rot':>7}  verdict")
        t_pair = time.perf_counter()
        if workers > 1:
            with ProcessPoolExecutor(max_workers=workers) as ex:
                results = list(ex.map(_pair_worker, tasks))
        else:
            results = [_pair_worker(t) for t in tasks]
        for (sa, sb, fit, rmse, trans, rot) in results:
            pr = _pair_verdict(sa, sb, fit, rmse, trans, rot)
            pair_recs.append(pr)
            print(f"  {_mark(pr['verdict'])} {pr['pair']:<22} {pr['fitness']:>7.2f}  "
                  f"{pr['rmse_mm']:>5.1f}mm  {pr['trans_mm']:>6.0f}mm  "
                  f"{pr['rot_deg']:>5.0f}deg")
            for note in pr["notes"]:
                print(f"         - {note}")
        print(f"  (pairwise probe: {time.perf_counter()-t_pair:.1f}s)")
    elif not probe_pairs:
        print("\nPer-pair  (skipped — --no-pairs)")

    # ── verdict ───────────────────────────────────────────────────────────
    all_verdicts = [r["verdict"] for r in view_recs] + [r["verdict"] for r in pair_recs]
    overall = _worst("PASS", *all_verdicts) if all_verdicts else "FAIL"

    bad_views = [r["view"] for r in view_recs if r["verdict"] == "FAIL"]
    bad_pairs = [r["pair"] for r in pair_recs if r["verdict"] == "FAIL"]
    warn_views = [r["view"] for r in view_recs if r["verdict"] == "WARN"]
    warn_pairs = [r["pair"] for r in pair_recs if r["verdict"] == "WARN"]

    print(f"\n{'-'*66}")
    print(f"VERDICT: {overall}    ({time.perf_counter()-t0:.1f}s)")
    if overall == "PASS":
        print("  Data looks good — proceed to recon.")
    if bad_views:
        print(f"  RE-RECORD these views (unusable): {', '.join(bad_views)}")
    if bad_pairs:
        print(f"  RE-RECORD these transitions (will break recon): {', '.join(bad_pairs)}")
    if warn_views:
        print(f"  Marginal views (recon may still work): {', '.join(warn_views)}")
    if warn_pairs:
        print(f"  Marginal transitions (FTM may drop a frame here): {', '.join(warn_pairs)}")
    if overall != "PASS":
        print("  Tip: capture --mode auto with slow, steady motion gives smaller")
        print("       inter-frame jumps and denser ROIs than manual SPACE-clicking.")
    print('='*66)

    report = {
        "data_dir": str(data_dir),
        "n_views": len(depth_files),
        "verdict": overall,
        "elapsed_s": round(time.perf_counter() - t0, 1),
        "views": view_recs,
        "pairs": pair_recs,
        "re_record_views": bad_views,
        "re_record_pairs": bad_pairs,
    }
    out = data_dir / "quality_report.json"
    out.write_text(json.dumps(report, indent=2))
    print(f"Report: {out}")
    return report


def main():
    ap = argparse.ArgumentParser(description="Fast capture data-quality check")
    ap.add_argument("data_dir", help="Capture directory (contains view_*_depth.npy)")
    ap.add_argument("--no-pairs", action="store_true",
                    help="Skip the pairwise registration probe (per-view stats only)")
    args = ap.parse_args()
    report = check_dir(args.data_dir, probe_pairs=not args.no_pairs)
    sys.exit(0 if report["verdict"] != "FAIL" else 1)


if __name__ == "__main__":
    main()
