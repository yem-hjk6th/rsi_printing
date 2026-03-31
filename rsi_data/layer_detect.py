"""
Interlayer Detection for RSI Printing Trajectory Data (0326)

从 RSI 位置数据中识别打印分层 (interlayer line), 根据 Z 轴离散台阶变化
检测每一层的边界, 标注层号, 并生成可视化图表。

用法:
    python layer_detect.py                          # 默认分析 202106
    python layer_detect.py --file rsi_data_20260326_213929.csv
    python layer_detect.py --no-show                # 仅保存图片，不弹窗
"""

import argparse
import csv
import os
from datetime import datetime
from pathlib import Path

import matplotlib
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.collections import LineCollection

# CJK 字体设置 (Windows)
for _font in ["Microsoft YaHei", "SimHei", "SimSun", "DejaVu Sans"]:
    try:
        matplotlib.rc("font", family=_font)
        break
    except Exception:
        continue
matplotlib.rcParams["axes.unicode_minus"] = False

# ─── 默认参数 ───────────────────────────────────────────────
DEFAULT_CSV = "rsi_data_20260326_202106.csv"
Z_APPROACH_THRESHOLD = 100.0  # Z > 此值视为接近/撤退阶段 (mm)
Z_ROUND_RESOLUTION = 1.0     # 层高分辨率 (mm), 用于量化 Z
MIN_LAYER_SAMPLES = 30       # 至少 N 个采样点才认为是有效层 (去除噪声)


def parse_args():
    parser = argparse.ArgumentParser(description="RSI 打印轨迹分层检测")
    parser.add_argument(
        "--file",
        default=DEFAULT_CSV,
        help="RSI CSV 文件名 (在 rsi_data/ 目录下) 或完整路径",
    )
    parser.add_argument(
        "--z-threshold",
        type=float,
        default=Z_APPROACH_THRESHOLD,
        help="Z 阈值: 大于此值视为接近/撤退阶段 (mm)",
    )
    parser.add_argument(
        "--layer-height",
        type=float,
        default=Z_ROUND_RESOLUTION,
        help="层高分辨率 (mm)",
    )
    parser.add_argument(
        "--min-samples",
        type=int,
        default=MIN_LAYER_SAMPLES,
        help="有效层的最少采样点数",
    )
    parser.add_argument(
        "--no-show",
        action="store_true",
        help="不弹出图形窗口 (仅保存)",
    )
    return parser.parse_args()


def load_rsi_csv(filepath):
    """加载 RSI CSV 数据, 返回结构化 numpy 数组。"""
    with open(filepath, newline="") as f:
        reader = csv.DictReader(f)
        rows = list(reader)

    n = len(rows)
    dtype = [
        ("timestamp", "f8"),
        ("ipoc", "i8"),
        ("x_mm", "f8"),
        ("y_mm", "f8"),
        ("z_mm", "f8"),
        ("a_deg", "f8"),
        ("b_deg", "f8"),
        ("c_deg", "f8"),
        ("override", "i4"),
        ("vel", "i4"),
    ]
    data = np.zeros(n, dtype=dtype)
    for i, r in enumerate(rows):
        data[i] = (
            float(r["timestamp"]),
            int(r["ipoc"]),
            float(r["x_mm"]),
            float(r["y_mm"]),
            float(r["z_mm"]),
            float(r["a_deg"]),
            float(r["b_deg"]),
            float(r["c_deg"]),
            int(r["override"]),
            int(r["vel"]),
        )
    return data


def detect_layers(data, z_threshold, layer_resolution, min_samples):
    """
    检测打印分层。

    算法:
      1. 对 Z 施加中值滤波消除瞬时噪声
      2. 将滤波后 Z 量化到最近 layer_resolution (对齐到 x.5 if needed)
      3. 连续相同量化值的区间 = 一个 segment
      4. 把短 segment (< min_samples) 吸收到前后相邻层
      5. 按时间顺序编号

    返回:
        layers: list of dict, 每层信息
        layer_ids: ndarray, 每个数据点所属的层号 (-1 表示非打印阶段)
    """
    z = data["z_mm"]
    t = data["timestamp"]
    n = len(z)

    # 1) 识别打印阶段 (排除接近/撤退)
    printing_mask = z < z_threshold
    layer_ids = np.full(n, -1, dtype=int)

    printing_indices = np.where(printing_mask)[0]
    if len(printing_indices) == 0:
        return [], layer_ids

    z_raw = z[printing_indices]

    # 2) 中值滤波 (窗口 15 ~ 1.5s) 消除瞬时偏差 (±0.1mm)
    from scipy.ndimage import median_filter
    z_filt = median_filter(z_raw, size=15)

    # 3) 量化: 用 floor 避免 banker's rounding 合并相邻层
    #    Z values are at x.5 positions (e.g. -10.5, -9.5, ...56.5, 57.5...)
    #    floor gives distinct integer for each layer
    z_quantized = np.floor(z_filt)

    # 4) 找到 Z 变化的断点
    change_mask = np.diff(z_quantized) != 0
    change_idx = np.where(change_mask)[0] + 1

    seg_starts = np.concatenate([[0], change_idx])
    seg_ends = np.concatenate([change_idx, [len(z_quantized)]])

    # 构建初始 segment 列表
    segments = []
    for s, e in zip(seg_starts, seg_ends):
        z_level = z_quantized[s]
        count = e - s
        orig_start = printing_indices[s]
        orig_end = printing_indices[min(e - 1, len(printing_indices) - 1)]
        segments.append({
            "z_level": z_level,
            "count": count,
            "orig_start": orig_start,
            "orig_end": orig_end,
        })

    # 5) 合并: 连续同一 Z 水平的 segment 合并; 短 segment 吸收到相邻层
    #    第一轮: 合并相同 Z (严格匹配, floor 已确保各层 z 不同)
    merged = []
    for seg in segments:
        if merged and seg["z_level"] == merged[-1]["z_level"]:
            merged[-1]["count"] += seg["count"]
            merged[-1]["orig_end"] = seg["orig_end"]
        else:
            merged.append(dict(seg))

    #    第二轮: 吸收过短 segment 到前一层 (过渡区间)
    absorbed = []
    for seg in merged:
        if seg["count"] < min_samples and absorbed:
            # 吸收到前一层
            absorbed[-1]["count"] += seg["count"]
            absorbed[-1]["orig_end"] = seg["orig_end"]
        else:
            absorbed.append(seg)
    # 最后一个若太短也合并到前一个
    if len(absorbed) > 1 and absorbed[-1]["count"] < min_samples:
        absorbed[-2]["count"] += absorbed[-1]["count"]
        absorbed[-2]["orig_end"] = absorbed[-1]["orig_end"]
        absorbed.pop()

    # 6) 构建层信息, 过滤掉接近/撤退阶段残余 (z_std 极大的段)
    layers = []
    real_layer_num = 0
    for seg in absorbed:
        i_start = seg["orig_start"]
        i_end = seg["orig_end"]
        t_start = t[i_start]
        t_end = t[i_end]
        z_slice = z[i_start : i_end + 1]
        z_mean = z_slice.mean()
        z_std = z_slice.std()

        # 跳过 Z 标准差 > 2mm 的段 (接近/撤退过渡)
        if z_std > 2.0:
            continue

        layers.append({
            "layer": real_layer_num,
            "z_level": seg["z_level"],
            "z_mean": z_mean,
            "z_std": z_std,
            "row_start": i_start,
            "row_end": i_end,
            "n_samples": i_end - i_start + 1,
            "t_start": t_start,
            "t_end": t_end,
            "duration_s": t_end - t_start,
        })

        layer_ids[i_start : i_end + 1] = real_layer_num
        real_layer_num += 1

    return layers, layer_ids


def print_layer_summary(layers, t0):
    """打印层信息摘要。"""
    print(f"\n{'='*80}")
    print(f"  打印轨迹分层检测结果  (Interlayer Detection)")
    print(f"{'='*80}")
    print(
        f"{'层号':>4s}  {'Z高度(mm)':>10s}  {'Z均值±std':>14s}  "
        f"{'起始时间(s)':>10s}  {'持续时间(s)':>10s}  {'采样点数':>8s}  "
        f"{'层间距(mm)':>10s}"
    )
    print("-" * 80)

    for i, lyr in enumerate(layers):
        dz = ""
        if i > 0:
            gap = lyr["z_level"] - layers[i - 1]["z_level"]
            dz = f"{gap:+.1f}"

        print(
            f"  {lyr['layer']:3d}  "
            f"  {lyr['z_level']:8.1f}  "
            f"  {lyr['z_mean']:6.2f}±{lyr['z_std']:.2f}  "
            f"  {lyr['t_start'] - t0:8.1f}  "
            f"  {lyr['duration_s']:8.1f}  "
            f"  {lyr['n_samples']:6d}  "
            f"  {dz:>8s}"
        )

    print("-" * 80)
    total_dur = layers[-1]["t_end"] - layers[0]["t_start"]
    z_total = layers[-1]["z_level"] - layers[0]["z_level"]
    print(f"  总层数: {len(layers)},  总打印高度: {z_total:.1f} mm,  总耗时: {total_dur:.1f}s ({total_dur/60:.1f}min)")
    print()


def save_layer_csv(layers, out_path, t0):
    """保存层信息到 CSV。"""
    with open(out_path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow([
            "layer", "z_level_mm", "z_mean_mm", "z_std_mm",
            "row_start", "row_end", "n_samples",
            "t_offset_s", "duration_s", "interlayer_gap_mm",
        ])
        for i, lyr in enumerate(layers):
            gap = ""
            if i > 0:
                gap = f"{lyr['z_level'] - layers[i-1]['z_level']:.2f}"
            writer.writerow([
                lyr["layer"],
                f"{lyr['z_level']:.1f}",
                f"{lyr['z_mean']:.2f}",
                f"{lyr['z_std']:.2f}",
                lyr["row_start"],
                lyr["row_end"],
                lyr["n_samples"],
                f"{lyr['t_start'] - t0:.2f}",
                f"{lyr['duration_s']:.2f}",
                gap,
            ])
    print(f"层信息已保存: {out_path}")


def plot_results(data, layers, layer_ids, tag, out_dir, show):
    """生成四幅可视化图表。"""
    t = data["timestamp"]
    t0 = t[0]
    t_rel = t - t0
    z = data["z_mm"]
    x = data["x_mm"]
    y = data["y_mm"]

    n_layers = len(layers)
    cmap = plt.cm.turbo
    colors = [cmap(i / max(1, n_layers - 1)) for i in range(n_layers)]

    fig = plt.figure(figsize=(20, 14))
    fig.suptitle(f"RSI 打印轨迹分层检测 — {tag}\n共 {n_layers} 层", fontsize=14)

    # ─── 图 1: Z vs Time, 标注层边界 ─────────────────────────────
    ax1 = fig.add_subplot(2, 2, 1)
    ax1.plot(t_rel, z, color="gray", linewidth=0.3, alpha=0.5, label="原始 Z")

    for lyr in layers:
        mask = (layer_ids == lyr["layer"])
        c = colors[lyr["layer"]]
        ax1.scatter(
            t_rel[mask], z[mask],
            s=1, c=[c], alpha=0.7,
        )
        # 层标签
        t_mid = (lyr["t_start"] + lyr["t_end"]) / 2 - t0
        ax1.annotate(
            f"L{lyr['layer']}",
            (t_mid, lyr["z_level"]),
            fontsize=5, alpha=0.8, ha="center",
            color=c,
        )

    # 画层间分界线 (interlayer lines)
    for i in range(1, n_layers):
        t_boundary = layers[i]["t_start"] - t0
        ax1.axvline(t_boundary, color="red", linewidth=0.3, alpha=0.4)

    ax1.set_xlabel("时间 (s)")
    ax1.set_ylabel("Z (mm)")
    ax1.set_title("Z 高度 vs 时间 (层着色)")
    ax1.grid(True, alpha=0.3)

    # ─── 图 2: XY 轨迹, 按层着色 ────────────────────────────────
    ax2 = fig.add_subplot(2, 2, 2)

    for lyr in layers:
        mask = (layer_ids == lyr["layer"])
        c = colors[lyr["layer"]]
        ax2.plot(
            x[mask], y[mask],
            linewidth=0.5, color=c, alpha=0.6,
        )

    ax2.set_xlabel("X (mm)")
    ax2.set_ylabel("Y (mm)")
    ax2.set_title("XY 打印轨迹 (按层着色)")
    ax2.set_aspect("equal")
    ax2.grid(True, alpha=0.3)

    # 添加色条
    sm = plt.cm.ScalarMappable(cmap=cmap, norm=plt.Normalize(0, n_layers - 1))
    sm.set_array([])
    cbar = fig.colorbar(sm, ax=ax2, shrink=0.8, label="层号")

    # ─── 图 3: 打印区 Z 高度台阶图 ──────────────────────────────
    ax3 = fig.add_subplot(2, 2, 3)
    # 只画打印区域
    print_mask = layer_ids >= 0
    if print_mask.any():
        t_print = t_rel[print_mask]
        z_print = z[print_mask]
        lid_print = layer_ids[print_mask]

        for lyr in layers:
            m = lid_print == lyr["layer"]
            c = colors[lyr["layer"]]
            ax3.fill_between(
                t_print[m],
                lyr["z_level"] - 0.4,
                lyr["z_level"] + 0.4,
                color=c, alpha=0.4,
            )
            ax3.plot(t_print[m], z_print[m], color=c, linewidth=0.5)

    ax3.set_xlabel("时间 (s)")
    ax3.set_ylabel("Z (mm)")
    ax3.set_title("打印区 Z 台阶 (层高可视化)")
    ax3.grid(True, alpha=0.3)

    # ─── 图 4: 3D 轨迹 ──────────────────────────────────────────
    ax4 = fig.add_subplot(2, 2, 4, projection="3d")

    for lyr in layers:
        mask = (layer_ids == lyr["layer"])
        c = colors[lyr["layer"]]
        ax4.plot(
            x[mask], y[mask], z[mask],
            linewidth=0.4, color=c, alpha=0.6,
        )

    ax4.set_xlabel("X (mm)")
    ax4.set_ylabel("Y (mm)")
    ax4.set_zlabel("Z (mm)")
    ax4.set_title("3D 打印轨迹 (按层着色)")

    plt.tight_layout()

    out_path = os.path.join(out_dir, f"layer_detect_{tag}.png")
    fig.savefig(out_path, dpi=200, bbox_inches="tight")
    print(f"图表已保存: {out_path}")

    if show:
        plt.show()
    else:
        plt.close(fig)

    # ─── 额外: 层间距统计图 ──────────────────────────────────────
    if n_layers > 1:
        fig2, (ax_a, ax_b) = plt.subplots(1, 2, figsize=(14, 5))
        fig2.suptitle(f"层间距与层时间分析 — {tag}", fontsize=12)

        gaps = [layers[i]["z_level"] - layers[i - 1]["z_level"] for i in range(1, n_layers)]
        durations = [lyr["duration_s"] for lyr in layers]
        layer_nums = [lyr["layer"] for lyr in layers]

        ax_a.bar(range(1, n_layers), gaps, color=[colors[i] for i in range(1, n_layers)], alpha=0.7)
        ax_a.axhline(np.mean(gaps), color="red", linestyle="--", label=f"均值 {np.mean(gaps):.2f} mm")
        ax_a.set_xlabel("层号")
        ax_a.set_ylabel("层间距 (mm)")
        ax_a.set_title("Interlayer Gap")
        ax_a.legend()
        ax_a.grid(True, alpha=0.3)

        ax_b.bar(layer_nums, durations, color=colors, alpha=0.7)
        ax_b.axhline(np.mean(durations), color="red", linestyle="--", label=f"均值 {np.mean(durations):.1f}s")
        ax_b.set_xlabel("层号")
        ax_b.set_ylabel("持续时间 (s)")
        ax_b.set_title("每层打印时间")
        ax_b.legend()
        ax_b.grid(True, alpha=0.3)

        plt.tight_layout()
        out_path2 = os.path.join(out_dir, f"layer_stats_{tag}.png")
        fig2.savefig(out_path2, dpi=200, bbox_inches="tight")
        print(f"统计图已保存: {out_path2}")

        if show:
            plt.show()
        else:
            plt.close(fig2)


def main():
    args = parse_args()

    # 解析文件路径
    script_dir = Path(__file__).resolve().parent
    csv_path = Path(args.file)
    if not csv_path.is_absolute():
        csv_path = script_dir / csv_path
    if not csv_path.exists():
        raise FileNotFoundError(f"找不到 CSV 文件: {csv_path}")

    # 从文件名提取时间标签
    stem = csv_path.stem
    tag = stem.replace("rsi_data_", "")

    print(f"加载数据: {csv_path}")
    data = load_rsi_csv(str(csv_path))
    print(f"  数据点: {len(data)},  时间跨度: {data['timestamp'][-1] - data['timestamp'][0]:.1f}s")

    # 检测分层
    layers, layer_ids = detect_layers(
        data,
        z_threshold=args.z_threshold,
        layer_resolution=args.layer_height,
        min_samples=args.min_samples,
    )

    if not layers:
        print("未检测到有效打印层!")
        return

    t0 = data["timestamp"][0]
    print_layer_summary(layers, t0)

    # 保存层信息 CSV
    out_csv = script_dir / f"layers_{tag}.csv"
    save_layer_csv(layers, str(out_csv), t0)

    # 可视化
    plot_results(
        data, layers, layer_ids, tag,
        out_dir=str(script_dir),
        show=not args.no_show,
    )


if __name__ == "__main__":
    main()
