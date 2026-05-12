# Artec Eva/Spider Pipeline — 分析与对比

> 供 agent 读取，记录 Artec 手持扫描仪的核心算法与我们自研 pipeline 的对比。

---

## Artec 为什么精度高

Artec 的"Registration"按钮背后是两个成熟算法的组合，二者均已在学术界公开：

### 1. 全局位姿图优化（Pose Graph Optimization）

- 所有帧的位姿同时优化，带闭环约束
- 算法：Kümmerle et al. g2o / Fischler & Bolles RANSAC + Levenberg-Marquardt
- Open3D 等价：`o3d.pipelines.registration.global_optimization` + `PoseGraph`
- 关键优势：误差不累积——链式 ICP 的第 7 帧误差 = 前 6 次的叠加，Pose Graph 是全局最小化

### 2. TSDF 体素融合（Truncated Signed Distance Function）

- 每帧深度图直接积分进共享 3D 体素网格，重叠区域取加权平均
- 算法：Curless & Levoy 1996，KinectFusion 2012
- Open3D 等价：`o3d.pipelines.integration.ScalableTSDFVolume`
- 关键优势：消除点云叠加的双层/模糊——不是把点云相加，而是在体素空间融合

---

## 我们的 pipeline 与 Artec 的差距

| 步骤 | Artec | demo1 (Colored ICP) | demo2 (TSDF, 目标) |
|------|-------|--------------------|--------------------|
| 配准 | 全局 Pose Graph 优化 | 链式 Colored ICP（误差累积） | 链式 Colored ICP + Pose Graph 优化 |
| 融合 | TSDF 体素积分 | 点云相加（双层模糊） | ScalableTSDFVolume 积分 |
| 网格化 | Sharp Fusion（自研） | Poisson（开源） | Poisson / MarchingCubes |
| 输入数据 | 连续 RGBD 流 | 离散 PLY（已损失深度图） | 离散 depth.npy + color.png |

---

## TSDF 融合的关键参数

```python
voxel_length = 0.001   # 1mm — 决定最终网格分辨率，越小越细但越慢
sdf_trunc    = 0.004   # 4mm — 截断距离，一般设 4-5x voxel_length
color_type   = TSDFVolumeColorType.RGB8
```

- `voxel_length=1mm` 对应打印层高约 0.2mm 的物体，细节可见
- `voxel_length=2mm` 适合快速预览，与 Artec Eva 精度接近（标称 0.05mm 但实际软件输出约 0.1-0.5mm）

---

## 数据需求对比

| | demo1 (Colored ICP) | demo2 (TSDF) |
|--|---------------------|--------------|
| 需要 PLY | ✓ | 可选 |
| 需要 depth.npy | ✗（已损失） | **✓ 必须** |
| 需要 color.png | ✗（颜色在 PLY 里） | **✓ 必须** |
| 能处理 multiview_20260505_142502 | ✓ | ✗（没存 depth/color） |
| 需要重新采集 | — | **是**，用 demo11_tsdf_capture.py |

---

## Open3D reconstruction_system 参考

完整参考实现（BundleFusion 风格，fragment-based）：
```
open3d/examples/python/reconstruction_system/
  run_system.py
  make_fragments.py       ← fragment 内 RGBD Odometry
  register_fragments.py   ← fragment 间全局配准
  refine_registration.py  ← ICP 精化
  integrate_scene.py      ← TSDF 融合
```

我们的 demo11/demo12 是简化版（无 fragment，直接帧级配准），适合离散视角采集。

---

## 能否达到 Artec Eva 量级

理论上可以接近（Artec Eva 标称精度 0.1mm，实际出口 0.2-0.5mm）：
- 传感器限制：ZED 2i 在 800mm 处 RMSE=2.3mm，比 Artec 结构光差约 10x
- 算法限制：TSDF+Pose Graph 可以消除融合误差，但消除不了传感器本身的深度噪声
- 可达目标：3mm 以内的表面重建误差（当前 Colored ICP chain 约 2-4mm rmse）

如需更高精度，需换传感器（结构光，如 Intel RealSense L515 / Azure Kinect）或引入 NeRF/3DGS。

---

## 文件索引

| 文件 | 用途 |
|------|------|
| `reconstruction/multiview_capture.py` | 离散视角采集（存 PLY，供 demo1 使用） |
| `zed_prin_exp_demo/demo11_tsdf_capture.py` | TSDF 采集（存 depth.npy + color.png + PLY） |
| `reconstruction/demo1_recon_coloredICP.py` | Colored ICP 链式配准融合 |
| `zed_prin_exp_demo/demo12_tsdf_recon.py` | TSDF 融合 + Pose Graph 优化（主脚本） |
| `reconstruction/demo2_recon_tsdf.py` | 同 demo12，reconstruction 目录副本 |
