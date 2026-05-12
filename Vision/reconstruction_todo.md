# ZED2i Reconstruction — TO-DO

## 未实现的 Pipeline / Methods

### 1. TSDF Fusion (Open3D)
- **适用场景**：多视角 + 薄壁结构，比 Poisson 对不完整点云更鲁棒
- **参考**：`open3d.pipelines.integration.ScalableTSDFVolume`
- **入口**：每个机位采一帧深度图 + 相机位姿 → 逐帧集成
- **需要**：camera-to-world 位姿矩阵（已有手眼标定结果可用）

### 2. NeRF / 3D Gaussian Splatting
- **适用场景**：纹理丰富的物体，渲染质量高，但对静止场景要求严格
- **候选实现**：
  - Instant-NGP（快，几分钟内）
  - 3D-GS（gaussian-splatting 官方实现）
  - nerfstudio（pipeline 完整，支持 COLMAP 位姿估计）
- **需要**：多视角 RGB 图 + 位姿（COLMAP 或已知机器人位姿）
- **注意**：当前 `ffs`/`zedenv` 环境均未安装，需独立 env

### 3. ICP 多视角配准融合
- **适用场景**：多机位点云对齐合并，得到完整 360° 模型
- **参考**：`open3d.pipelines.registration.registration_icp`
- **流程**：
  1. 每个机位 → 一个 PLY 点云（demo8 已能产出）
  2. 粗配准：FPFH + RANSAC
  3. 精配准：Point-to-Plane ICP
  4. 合并：`open3d.geometry.PointCloud` + voxel downsample
- **需要**：至少 30% overlap 的相邻机位；机器人 FK 位姿可作为初始变换

## 优先级建议
1. ICP 融合（基于现有 demo8 输出，改动最小）
2. TSDF（利用手眼标定位姿，质量最稳定）
3. NeRF/3DGS（最后，环境成本最高）
