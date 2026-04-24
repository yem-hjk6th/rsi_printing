"""
bead_texture_compare.py — 光滑 vs 粗糙 bead 表面纹理特征对比
  输入: 左右并排的对比图 (左=光滑, 右=粗糙)
  输出: 多种特征的量化对比 + 可视化

  用法: python bead_texture_compare.py <image_path>
"""

import sys, cv2
import numpy as np
from pathlib import Path
from scipy.ndimage import gaussian_filter
from skimage.feature import local_binary_pattern, graycomatrix, graycoprops

# ═══════════════════════════════════════════════════════════════════════════════
#  CONFIG
# ═══════════════════════════════════════════════════════════════════════════════

# 分割点: None = 自动按中线切
SPLIT_X = None

# LBP 参数
LBP_RADIUS = 2
LBP_NPOINTS = 8 * LBP_RADIUS

# GLCM 参数
GLCM_DISTANCES = [1, 3, 5]
GLCM_ANGLES = [0, np.pi / 4, np.pi / 2, 3 * np.pi / 4]
GLCM_LEVELS = 64

# 边缘轮廓分析: 多项式拟合阶数
POLY_ORDER = 3


# ═══════════════════════════════════════════════════════════════════════════════
#  FEATURE EXTRACTION
# ═══════════════════════════════════════════════════════════════════════════════

def split_image(bgr, split_x=None):
    """将左右拼接图分为两张."""
    h, w = bgr.shape[:2]
    if split_x is None:
        split_x = w // 2
    left = bgr[:, :split_x]
    right = bgr[:, split_x:]
    return left, right


def extract_bead_roi(gray):
    """提取 bead 区域 (排除黑色背景)."""
    _, mask = cv2.threshold(gray, 40, 255, cv2.THRESH_BINARY)
    # 去噪
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    return mask


def feat_edge_roughness(gray, mask):
    """边缘轮廓粗糙度: 提取最长轮廓 → 拟合多项式 → 残差 RMS."""
    edges = cv2.Canny(gray, 30, 100)
    edges = edges & mask
    contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL,
                                    cv2.CHAIN_APPROX_NONE)
    if not contours:
        return {"edge_residual_rms": 0, "edge_curvature_std": 0}

    # 取最长轮廓
    longest = max(contours, key=lambda c: len(c))
    pts = longest.squeeze()
    if pts.ndim != 2 or len(pts) < 10:
        return {"edge_residual_rms": 0, "edge_curvature_std": 0}

    xs, ys = pts[:, 0].astype(float), pts[:, 1].astype(float)

    # 按 y 排序 (bead 大致竖直)
    order = np.argsort(ys)
    xs, ys = xs[order], ys[order]

    # 多项式拟合
    coeffs = np.polyfit(ys, xs, POLY_ORDER)
    xs_fit = np.polyval(coeffs, ys)
    residuals = xs - xs_fit
    rms = np.sqrt(np.mean(residuals ** 2))

    # 曲率: 用差分近似
    dx = np.gradient(xs)
    dy = np.gradient(ys)
    ddx = np.gradient(dx)
    ddy = np.gradient(dy)
    denom = (dx ** 2 + dy ** 2) ** 1.5
    denom[denom < 1e-6] = 1e-6
    curvature = np.abs(dx * ddy - dy * ddx) / denom
    curv_std = np.std(curvature)

    return {"edge_residual_rms": rms, "edge_curvature_std": curv_std}


def feat_frequency(gray, mask):
    """频域特征: 高频能量占比."""
    roi = gray.copy().astype(float)
    roi[mask == 0] = np.mean(roi[mask > 0]) if np.any(mask > 0) else 128

    f = np.fft.fft2(roi)
    fshift = np.fft.fftshift(f)
    mag = np.abs(fshift)

    h, w = mag.shape
    cy, cx = h // 2, w // 2
    Y, X = np.ogrid[:h, :w]
    r = np.sqrt((X - cx) ** 2 + (Y - cy) ** 2)

    r_max = min(cx, cy)
    total_energy = np.sum(mag ** 2)
    if total_energy < 1e-6:
        return {"hf_energy_ratio": 0, "mf_energy_ratio": 0}

    hf_mask = r > r_max * 0.5
    mf_mask = (r > r_max * 0.2) & (r <= r_max * 0.5)
    hf_ratio = np.sum(mag[hf_mask] ** 2) / total_energy
    mf_ratio = np.sum(mag[mf_mask] ** 2) / total_energy

    return {"hf_energy_ratio": hf_ratio, "mf_energy_ratio": mf_ratio}


def feat_glcm(gray, mask):
    """GLCM 纹理特征: contrast, dissimilarity, homogeneity, energy, correlation."""
    # 量化到 GLCM_LEVELS
    roi = gray.copy()
    roi[mask == 0] = 0
    if np.any(mask > 0):
        vmin, vmax = roi[mask > 0].min(), roi[mask > 0].max()
        if vmax > vmin:
            roi = ((roi.astype(float) - vmin) / (vmax - vmin) *
                   (GLCM_LEVELS - 1)).astype(np.uint8)
        else:
            roi = np.zeros_like(roi)
    roi = np.clip(roi, 0, GLCM_LEVELS - 1).astype(np.uint8)

    glcm = graycomatrix(roi, distances=GLCM_DISTANCES,
                         angles=GLCM_ANGLES, levels=GLCM_LEVELS,
                         symmetric=True, normed=True)

    feats = {}
    for prop in ["contrast", "dissimilarity", "homogeneity", "energy",
                 "correlation"]:
        vals = graycoprops(glcm, prop)
        feats[f"glcm_{prop}"] = float(np.mean(vals))
    return feats


def feat_lbp(gray, mask):
    """LBP 纹理特征: 直方图分布的均匀性."""
    lbp = local_binary_pattern(gray, LBP_NPOINTS, LBP_RADIUS, method="uniform")
    n_bins = LBP_NPOINTS + 2
    roi_vals = lbp[mask > 0]
    if len(roi_vals) == 0:
        return {"lbp_entropy": 0, "lbp_uniformity": 0}

    hist, _ = np.histogram(roi_vals, bins=n_bins, range=(0, n_bins), density=True)
    hist = hist + 1e-10
    entropy = -np.sum(hist * np.log2(hist))
    uniformity = np.sum(hist ** 2)

    return {"lbp_entropy": entropy, "lbp_uniformity": uniformity,
            "lbp_image": lbp}


def feat_gradient_variance(gray, mask):
    """梯度方差: 粗糙表面梯度变化大."""
    gx = cv2.Sobel(gray, cv2.CV_64F, 1, 0, ksize=3)
    gy = cv2.Sobel(gray, cv2.CV_64F, 0, 1, ksize=3)
    mag = np.sqrt(gx ** 2 + gy ** 2)
    roi_vals = mag[mask > 0]
    if len(roi_vals) == 0:
        return {"grad_mean": 0, "grad_std": 0, "grad_image": mag}
    return {"grad_mean": float(np.mean(roi_vals)),
            "grad_std": float(np.std(roi_vals)),
            "grad_image": mag}


def feat_specular(gray, mask):
    """高光碎片化: 高亮区域的连通域数量和平均面积."""
    thresh = np.percentile(gray[mask > 0], 90) if np.any(mask > 0) else 200
    _, bright = cv2.threshold(gray, int(thresh), 255, cv2.THRESH_BINARY)
    bright = bright & mask
    n_labels, labels, stats, _ = cv2.connectedComponentsWithStats(bright)
    # 排除背景 label 0
    if n_labels <= 1:
        return {"specular_count": 0, "specular_avg_area": 0,
                "specular_image": bright}
    areas = stats[1:, cv2.CC_STAT_AREA]
    return {"specular_count": n_labels - 1,
            "specular_avg_area": float(np.mean(areas)),
            "specular_image": bright}


# ═══════════════════════════════════════════════════════════════════════════════
#  VISUALIZATION
# ═══════════════════════════════════════════════════════════════════════════════

def visualize_comparison(left_bgr, right_bgr, left_feats, right_feats,
                         left_extras, right_extras, save_path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(4, 2, figsize=(10, 16))
    labels = ["Smooth (left)", "Rough (right)"]

    # Row 0: 原图
    for i, (bgr, label) in enumerate([(left_bgr, labels[0]),
                                       (right_bgr, labels[1])]):
        axes[0, i].imshow(cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB))
        axes[0, i].set_title(label)
        axes[0, i].axis("off")

    # Row 1: 梯度幅值
    for i, ext in enumerate([left_extras, right_extras]):
        im = axes[1, i].imshow(ext["grad_image"], cmap="hot")
        axes[1, i].set_title(f"Gradient mag\nmean={ext['grad_mean']:.1f}, "
                              f"std={ext['grad_std']:.1f}")
        axes[1, i].axis("off")

    # Row 2: LBP
    for i, ext in enumerate([left_extras, right_extras]):
        axes[2, i].imshow(ext["lbp_image"], cmap="gray")
        axes[2, i].set_title(f"LBP\nentropy={ext['lbp_entropy']:.2f}, "
                              f"uniformity={ext['lbp_uniformity']:.3f}")
        axes[2, i].axis("off")

    # Row 3: 高光碎片
    for i, ext in enumerate([left_extras, right_extras]):
        axes[3, i].imshow(ext["specular_image"], cmap="gray")
        axes[3, i].set_title(f"Specular highlights\n"
                              f"count={ext['specular_count']}, "
                              f"avg_area={ext['specular_avg_area']:.1f}")
        axes[3, i].axis("off")

    plt.tight_layout()
    fig.savefig(str(save_path), dpi=150, bbox_inches="tight")
    print(f"Saved: {save_path}")
    plt.close(fig)


def print_comparison(left_feats, right_feats):
    """打印特征对比表."""
    print(f"\n{'Feature':<25s} {'Smooth':>10s} {'Rough':>10s} {'Ratio':>8s}")
    print("-" * 56)
    for key in left_feats:
        lv = left_feats[key]
        rv = right_feats[key]
        ratio = rv / lv if abs(lv) > 1e-8 else float("inf")
        print(f"{key:<25s} {lv:10.4f} {rv:10.4f} {ratio:8.2f}")


# ═══════════════════════════════════════════════════════════════════════════════
#  MAIN
# ═══════════════════════════════════════════════════════════════════════════════

def main():
    if len(sys.argv) > 1:
        img_path = Path(sys.argv[1])
    else:
        img_path = Path(r"recorded_data/phone_record/nozzle_ROI_texture.jpeg")

    bgr = cv2.imread(str(img_path))
    if bgr is None:
        print(f"[ERROR] Cannot read: {img_path}")
        sys.exit(1)

    print(f"Image: {img_path.name}  size: {bgr.shape[1]}x{bgr.shape[0]}")

    left_bgr, right_bgr = split_image(bgr, SPLIT_X)
    print(f"Split: left={left_bgr.shape[1]}x{left_bgr.shape[0]}, "
          f"right={right_bgr.shape[1]}x{right_bgr.shape[0]}")

    results = {}
    for name, roi_bgr in [("smooth", left_bgr), ("rough", right_bgr)]:
        gray = cv2.cvtColor(roi_bgr, cv2.COLOR_BGR2GRAY)
        mask = extract_bead_roi(gray)
        pct = mask.sum() / 255 / mask.size * 100
        print(f"  {name}: bead ROI = {pct:.1f}% pixels")

        feats = {}
        extras = {}

        f = feat_edge_roughness(gray, mask)
        feats.update(f)

        f = feat_frequency(gray, mask)
        feats.update(f)

        f = feat_glcm(gray, mask)
        feats.update(f)

        f = feat_lbp(gray, mask)
        extras["lbp_image"] = f.pop("lbp_image")
        extras["lbp_entropy"] = f["lbp_entropy"]
        extras["lbp_uniformity"] = f["lbp_uniformity"]
        feats.update(f)

        f = feat_gradient_variance(gray, mask)
        extras["grad_image"] = f.pop("grad_image")
        extras["grad_mean"] = f["grad_mean"]
        extras["grad_std"] = f["grad_std"]
        feats.update(f)

        f = feat_specular(gray, mask)
        extras["specular_image"] = f.pop("specular_image")
        extras["specular_count"] = f["specular_count"]
        extras["specular_avg_area"] = f["specular_avg_area"]
        feats.update(f)

        results[name] = {"feats": feats, "extras": extras}

    print_comparison(results["smooth"]["feats"], results["rough"]["feats"])

    out_path = img_path.parent / f"{img_path.stem}_features.png"
    visualize_comparison(left_bgr, right_bgr,
                         results["smooth"]["feats"],
                         results["rough"]["feats"],
                         results["smooth"]["extras"],
                         results["rough"]["extras"],
                         out_path)


if __name__ == "__main__":
    main()
